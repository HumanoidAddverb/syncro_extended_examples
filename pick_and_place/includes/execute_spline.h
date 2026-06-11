/**
 * @file execute_spline.h
 * @brief Header-only library for executing cubic splines with external velocity control
 * @version 1.0
 * @date 2026-02-11
 */

#ifndef EXECUTE_SPLINE_H
#define EXECUTE_SPLINE_H

#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <algorithm>

#include "system_manager.h"
#include "cubic_spline.h"
#include "timer.h"

namespace SplineExecution
{

    /**
     * @brief Trajectory segment durations
     * Configure these values to change the speed of the robot
     */
    struct SegmentDurations
    {
        double home_to_pick = 10.0;   // Duration in seconds
        double pick_to_place = 20.0;
        double place_to_home = 10.0;
        // detailed segments if needed
    };

    /**
     * @brief Configuration for spline execution
     * Configure these values to change execution behavior
     */
    struct ExecutionConfig
    {
        double sample_rate = 0.002;            // Control loop rate in seconds (default 2ms = 500Hz)
        double position_error_threshold = 0.01; // Stop if position error < threshold (radians)
        int controller_id = 13;                // Controller ID (13 = eExternalVelocity)
        int safety_type = 3;                   // Safety setting for robot
        bool check_error_every_step = true;    // If true, checks using getRobotData() every cycle
        int error_check_interval = 10;         // Unused if check_error_every_step is true
        
        double drift_correction_gain = 1.0;    // Gain for position feedback (Try 0.5 - 2.0). 0.0 = Open Loop.

        SegmentDurations durations;            // Durations for specific segments
    };

    /**
     * @brief Structure to store a sampled trajectory
     */
    struct SavedTrajectory
    {
        std::vector<std::vector<double>> dense_positions;
        std::vector<std::vector<double>> dense_velocities;
        std::vector<double> dense_times;
        bool is_valid = false;
    };

    // --- Helper Functions ---

    inline std::vector<double> getCurrentJointPosition(SystemManager& stk)
    {
        RobotData data;
        stk.getRobotData(data);
        std::vector<double> current_joint_position(N_DOF);
        for (int i = 0; i < N_DOF; i++)
        {
            current_joint_position[i] = data.jpos[i];
        }
        return current_joint_position;
    }

    inline double calculatePositionError(const std::vector<double>& current, const std::vector<double>& target)
    {
        if (current.size() != target.size()) return -1.0;
        
        double error_squared = 0.0;
        for (size_t i = 0; i < current.size(); i++)
        {
            double diff = current[i] - target[i];
            error_squared += diff * diff;
        }
        return std::sqrt(error_squared);
    }

    /**
     * @brief Create a cubic spline and sample it for positions and velocities
     */
    inline bool createAndSampleSplineWithVelocity(
        const std::vector<double>& seg_times,
        const std::vector<std::vector<double>>& seg_positions,
        double sample_rate,
        std::vector<std::vector<double>>& dense_positions,
        std::vector<std::vector<double>>& dense_velocities,
        std::vector<double>& dense_times)
    {
        if (seg_times.empty()) return false;
        double seg_duration = seg_times.back();
        
        CubicSpline spline;
        if (!spline.setWaypoints(seg_times, seg_positions))
        {
            std::cerr << "[ExecSpline] Failed to create spline" << std::endl;
            return false;
        }
        
        int num_samples = static_cast<int>(seg_duration / sample_rate) + 1;
        
        dense_positions.clear();
        dense_velocities.clear();
        dense_times.clear();
        
        dense_positions.reserve(num_samples);
        dense_velocities.reserve(num_samples);
        dense_times.reserve(num_samples);
        
        for (int j = 0; j < num_samples; j++)
        {
            double t = j * sample_rate;
            if (t > seg_duration) t = seg_duration;
            
            double pos[N_DOF];
            double vel[N_DOF];
            
            if (spline.interpolate(t, pos) && spline.getVelocity(t, vel))
            {
                std::vector<double> p_vec(pos, pos + N_DOF);
                std::vector<double> v_vec(vel, vel + N_DOF);
                
                dense_positions.push_back(p_vec);
                dense_velocities.push_back(v_vec);
                dense_times.push_back(t);
            }
        }
        return true;
    }

    /**
     * @brief Execute pre-calculated trajectory using External Velocity Control
     */
    inline bool executeExternalVelocityTrajectory(
        SystemManager& stk, 
        Config& data, 
        AlliedData& interrupt,
        Timer& tm, 
        const std::vector<std::vector<double>>& positions,
        const std::vector<std::vector<double>>& velocities,
        const ExecutionConfig& config)
    {
        // 1. Reset controller
        data.reset_controller = config.controller_id; 
        data.safety_type = config.safety_type;
        
        if (!stk.reset(data))
        {
            std::cerr << "[ExecSpline] Failed to reset controller" << std::endl;
            return false;
        }
        
        // 2. Control Loop
        tm.timeReset();
        auto start_time = std::chrono::steady_clock::now();
        size_t num_points = velocities.size();
        
        bool use_error_threshold = (config.position_error_threshold > 0.0) && !positions.empty();
        std::vector<double> final_target; 
        if (!positions.empty()) final_target = positions.back();
        
        for (size_t i = 0; i < num_points; i++)
        {
            // a. Prepare velocity command with Drift Correction
            if (config.drift_correction_gain > 0.0 && !positions.empty())
            {
                std::vector<double> current_q = getCurrentJointPosition(stk);
                const std::vector<double>& target_q = positions[i];
                
                // Initialize with feed-forward velocity
                interrupt.external_control_data = velocities[i];
                
                // Add proportional feedback correction
                for (int d = 0; d < N_DOF; d++)
                {
                    double pos_err = target_q[d] - current_q[d];
                    interrupt.external_control_data[d] += (config.drift_correction_gain * pos_err);
                }
            }
            else
            {
                interrupt.external_control_data = velocities[i];
            }
            
            // b. Send command
            if (!stk.doControl(interrupt))
            {
                std::cerr << "[ExecSpline] Control failed at step " << i << std::endl;
                return false;
            }
            
            // c. Check position error
            bool check_now = config.check_error_every_step || (i % config.error_check_interval == 0);
            if (use_error_threshold && check_now)
            {
                std::vector<double> current_pos = getCurrentJointPosition(stk);
                double error = calculatePositionError(current_pos, final_target);
                
                if (error < config.position_error_threshold)
                {
                    std::cout << "  [ExecSpline] Position error (" << error 
                              << " rad) < threshold (" << config.position_error_threshold 
                              << "). Stopping." << std::endl;
                    break;
                }
            }
            
            // d. Wait for next cycle
            auto target_time = start_time + std::chrono::nanoseconds(
                static_cast<int64_t>((i + 1) * config.sample_rate * 1e9));
            std::this_thread::sleep_until(target_time);
        }
        
        // Stop robot
        std::fill(interrupt.external_control_data.begin(), interrupt.external_control_data.end(), 0.0);
        stk.doControl(interrupt);
        
        return true;
    }

    /**
     * @brief Generate spline segment and execute it
     */
    inline bool executeSplineSegment(
        SystemManager& stk, 
        Config& data, 
        AlliedData& interrupt,
        Timer& tm, 
        const std::vector<double>& seg_times,
        const std::vector<std::vector<double>>& seg_positions,
        const ExecutionConfig& config,
        SavedTrajectory* save_to = nullptr)
    {
        std::vector<std::vector<double>> dense_positions;
        std::vector<std::vector<double>> dense_velocities;
        std::vector<double> dense_times;
        
        if (!createAndSampleSplineWithVelocity(seg_times, seg_positions, config.sample_rate, 
                                  dense_positions, dense_velocities, dense_times))
        {
            return false;
        }
        
        if (save_to)
        {
            save_to->dense_positions = dense_positions;
            save_to->dense_velocities = dense_velocities;
            save_to->dense_times = dense_times;
            save_to->is_valid = true;
        }
        
        return executeExternalVelocityTrajectory(stk, data, interrupt, tm, 
                                                dense_positions, dense_velocities, 
                                                config);
    }

    /**
     * @brief Execute a previously saved trajectory (forward or reverse)
     */
    inline bool executeSavedTrajectory(
        SystemManager& stk, 
        Config& data, 
        AlliedData& interrupt,
        Timer& tm, 
        const SavedTrajectory& saved_traj,
        bool reverse,
        const ExecutionConfig& config)
    {
        if (!saved_traj.is_valid) return false;
        
        if (reverse)
        {
            // Reverse positions
            std::vector<std::vector<double>> rev_positions(
                saved_traj.dense_positions.rbegin(), saved_traj.dense_positions.rend());
                
            // Reverse velocities AND negate
            std::vector<std::vector<double>> rev_velocities;
            rev_velocities.reserve(saved_traj.dense_velocities.size());
            
            for (auto it = saved_traj.dense_velocities.rbegin(); 
                 it != saved_traj.dense_velocities.rend(); ++it)
            {
                std::vector<double> neg_vel = *it;
                for (double& v : neg_vel) v = -v;
                rev_velocities.push_back(neg_vel);
            }
            
            return executeExternalVelocityTrajectory(stk, data, interrupt, tm,
                                                    rev_positions, rev_velocities,
                                                    config);
        }
        else
        {
            return executeExternalVelocityTrajectory(stk, data, interrupt, tm,
                                                    saved_traj.dense_positions, 
                                                    saved_traj.dense_velocities,
                                                    config);
        }
    }

} // namespace SplineExecution

#endif // EXECUTE_SPLINE_H
