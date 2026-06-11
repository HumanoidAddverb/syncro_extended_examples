/**
 * @file execute_pick_place_spline.cpp
 * @brief Execute pick-and-place trajectory using direct External Velocity Control
 * @version 2.0
 * @date 2026-05-20
 *
 * Variable-count waypoints. The CSV (produced by collect_pick_place_waypoints)
 * has waypoints in collection order with names "home", "pick", "place", or
 * "wp_*" (intermediates). Phases are derived from those role names; intermediate
 * waypoints between roles are interpolated by the cubic spline.
 *
 * Phase durations are read from SplineExecution::ExecutionConfig::durations
 * (see include/execute_spline.h).
 */

#include "system_manager.h"
#include "pick_place_config.h"
#include "execute_spline.h"
#include "ee_config.h"
#include <memory>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>

struct PickPlaceWaypoint
{
    std::string name;
    std::vector<double> joints;
};

bool loadPickPlaceWaypointsFromCSV(const std::string& filename,
                                    std::vector<PickPlaceWaypoint>& waypoints)
{
    std::ifstream file(filename);
    if (!file.is_open()) return false;

    waypoints.clear();
    std::string line;
    bool first_line = true;

    while (std::getline(file, line))
    {
        if (first_line) { first_line = false; continue; }
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string value;
        PickPlaceWaypoint wp;

        std::getline(ss, wp.name, ',');
        std::getline(ss, value, ',');           // skip duration column
        wp.joints.resize(N_DOF);
        for (int i = 0; i < N_DOF; i++)
        {
            std::getline(ss, value, ',');
            wp.joints[i] = std::stod(value);
        }
        waypoints.push_back(wp);
    }
    file.close();
    return true;
}

// Find the first waypoint whose name matches `role`. Returns -1 if missing.
int findRoleIndex(const std::vector<PickPlaceWaypoint>& wps, const std::string& role)
{
    for (size_t i = 0; i < wps.size(); i++)
        if (wps[i].name == role) return static_cast<int>(i);
    return -1;
}

// Build inclusive index range [from, to]
std::vector<int> rangeInclusive(int from, int to)
{
    std::vector<int> out;
    if (from <= to) for (int i = from; i <= to; i++) out.push_back(i);
    else for (int i = from; i >= to; i--) out.push_back(i);
    return out;
}

void buildTrajectoryFromSequence(
    const std::vector<PickPlaceWaypoint>& waypoints,
    const std::vector<int>& indices,
    double total_duration,
    const std::vector<double>& start_position,
    std::vector<double>& seg_times,
    std::vector<std::vector<double>>& seg_positions)
{
    seg_times.clear();
    seg_positions.clear();

    seg_times.push_back(0.0);
    if (!start_position.empty()) seg_positions.push_back(start_position);
    else seg_positions.push_back(waypoints[indices[0]].joints);

    int num_segments = start_position.empty() ? static_cast<int>(indices.size()) - 1
                                              : static_cast<int>(indices.size());
    if (num_segments < 1) num_segments = 1;

    double time_per_segment = total_duration / num_segments;
    double cumulative_time = 0.0;
    int start_idx = start_position.empty() ? 1 : 0;

    for (size_t i = start_idx; i < indices.size(); i++)
    {
        cumulative_time += time_per_segment;
        seg_times.push_back(cumulative_time);
        seg_positions.push_back(waypoints[indices[i]].joints);
    }
}

bool executeWaypointSequence(
    SystemManager& stk,
    Config& data,
    AlliedData& interrupt,
    Timer& tm,
    const std::vector<PickPlaceWaypoint>& waypoints,
    const std::vector<int>& indices,
    const std::string& segment_name,
    const SplineExecution::ExecutionConfig& exec_config,
    bool use_current_position = true,
    SplineExecution::SavedTrajectory* save_to = nullptr)
{
    if (indices.size() < 2) return true;  // nothing to traverse

    std::vector<double> seg_times;
    std::vector<std::vector<double>> seg_positions;

    std::vector<double> start_pos;
    if (use_current_position) start_pos = SplineExecution::getCurrentJointPosition(stk);

    double total_duration = 1.0;
    if (segment_name == "home_to_pick")       total_duration = exec_config.durations.home_to_pick;
    else if (segment_name == "pick_to_place") total_duration = exec_config.durations.pick_to_place;
    else if (segment_name == "place_to_home") total_duration = exec_config.durations.place_to_home;
    else if (segment_name == "pick_to_home")  total_duration = exec_config.durations.home_to_pick;

    buildTrajectoryFromSequence(waypoints, indices, total_duration, start_pos,
                                seg_times, seg_positions);

    return SplineExecution::executeSplineSegment(stk, data, interrupt, tm,
                                                seg_times, seg_positions, exec_config, save_to);
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <pick_place_waypoints_csv>" << std::endl;
        return 1;
    }
    std::string csv_filename = argv[1];

    PickPlaceConfig::GripperSettings gripper_settings;
    PickPlaceConfig::HomingSettings homing_settings;
    PickPlaceConfig::TrajectorySettings trajectory_settings;

    SplineExecution::ExecutionConfig exec_config;

    std::cout << "  Sample Rate: " << exec_config.sample_rate << "s" << std::endl;
    std::cout << "  Pos Error Threshold: " << exec_config.position_error_threshold << " rad" << std::endl;
    std::cout << "Trajectory Timings Configured:" << std::endl;
    std::cout << "  Home -> Pick: " << exec_config.durations.home_to_pick << "s" << std::endl;
    std::cout << "  Pick -> Place: " << exec_config.durations.pick_to_place << "s" << std::endl;
    std::cout << "  Place -> Home: " << exec_config.durations.place_to_home << "s" << std::endl;

    std::vector<PickPlaceWaypoint> waypoints;
    if (!loadPickPlaceWaypointsFromCSV(csv_filename, waypoints)) {
        std::cerr << "Failed to load CSV" << std::endl;
        return 1;
    }
    if (waypoints.size() < 3) {
        std::cerr << "Need at least 3 waypoints (home, pick, place)" << std::endl;
        return 1;
    }

    // Locate roles
    int home_idx  = findRoleIndex(waypoints, "home");
    int pick_idx  = findRoleIndex(waypoints, "pick");
    int place_idx = findRoleIndex(waypoints, "place");

    if (home_idx < 0 || pick_idx < 0 || place_idx < 0) {
        std::cerr << "CSV missing one of: home, pick, place" << std::endl;
        return 1;
    }
    if (!(home_idx < pick_idx && pick_idx < place_idx)) {
        std::cerr << "Waypoints must be ordered: home ... pick ... place ..." << std::endl;
        return 1;
    }

    // Phase index lists (inclusive ranges, all intermediates included)
    std::vector<int> home_to_pick_idx  = rangeInclusive(home_idx, pick_idx);
    std::vector<int> pick_to_place_idx = rangeInclusive(pick_idx, place_idx);
    std::vector<int> place_to_home_idx = rangeInclusive(place_idx, static_cast<int>(waypoints.size()) - 1);
    place_to_home_idx.push_back(home_idx);  // close the loop back to home

    // Reverse path for pick_to_home (used in the return half of the cycle)
    std::vector<int> pick_to_home_idx;
    for (int i = pick_idx; i >= home_idx; i--) pick_to_home_idx.push_back(i);

    std::cout << "Phase sizes — home_to_pick: " << home_to_pick_idx.size()
              << ", pick_to_place: " << pick_to_place_idx.size()
              << ", place_to_home: " << place_to_home_idx.size() << std::endl;

    SystemManager stk;
    Config data;
    AlliedData interrupt;
    Timer tm;

    EEInertia ee;
    ee.mass = EE_MASS;
    for (int i = 0; i < 3; i++) ee.com[i] = EE_COM[i];
    for (int i = 0; i < 6; i++) ee.inertia[i] = EE_MOI[i];

    data.reset_ee = true;
    data.iner = ee;
    data.gripper_type = static_cast<int>(GripperTypes::eFeetechGripper);

    data.reset_controller = static_cast<int>(ControllerEnum::eLinearVelocity);
    data.safety_type = 2;
    for (int i = 0; i < N_DOF; i++) data.target_position[i] = 0.0;
    data.delta_t = homing_settings.duration;

    if (!stk.setupRobot(data)) {
        std::cerr << "Failed to setup robot" << std::endl;
        return 1;
    }

    std::cout << "Moving to home..." << std::endl;
    tm.timeReset();
    double start_time = tm.getCurTime();
    bool moving_to_home = true;
    while (moving_to_home)
    {
        if (tm.getCurTime() - start_time > homing_settings.duration + homing_settings.buffer_time)
            moving_to_home = false;
        if (!stk.doControl(interrupt)) return 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    stk.openGripper();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    int cycle_count = 0;
    SplineExecution::SavedTrajectory traj_A_to_B;
    SplineExecution::SavedTrajectory traj_place_to_home;

    while (true)
    {
        cycle_count++;
        std::cout << "\nCycle " << cycle_count << " - Part 1: A -> B" << std::endl;

        if (!executeWaypointSequence(stk, data, interrupt, tm, waypoints,
                home_to_pick_idx, "home_to_pick", exec_config)) return 1;

        stk.closeGripper(gripper_settings.close_force);
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(gripper_settings.pick_dwell_time*1000)));

        if (cycle_count == 1) {
            if (!executeWaypointSequence(stk, data, interrupt, tm, waypoints,
                    pick_to_place_idx, "pick_to_place", exec_config, true, &traj_A_to_B)) return 1;
        } else {
            if (!SplineExecution::executeSavedTrajectory(stk, data, interrupt, tm,
                    traj_A_to_B, false, exec_config)) return 1;
        }

        stk.openGripper();
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(gripper_settings.place_dwell_time*1000)));

        if (cycle_count == 1) {
            if (!executeWaypointSequence(stk, data, interrupt, tm, waypoints,
                    place_to_home_idx, "place_to_home", exec_config, true, &traj_place_to_home)) return 1;
        } else {
            if (!SplineExecution::executeSavedTrajectory(stk, data, interrupt, tm,
                    traj_place_to_home, false, exec_config)) return 1;
        }

        std::cout << "\nCycle " << cycle_count << " - Part 2: B -> A" << std::endl;

        if (!SplineExecution::executeSavedTrajectory(stk, data, interrupt, tm,
                traj_place_to_home, true, exec_config)) return 1;

        stk.closeGripper(gripper_settings.close_force);
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(gripper_settings.pick_dwell_time*1000)));

        if (!SplineExecution::executeSavedTrajectory(stk, data, interrupt, tm,
                traj_A_to_B, true, exec_config)) return 1;

        stk.openGripper();
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(gripper_settings.place_dwell_time*1000)));

        if (!executeWaypointSequence(stk, data, interrupt, tm, waypoints,
                pick_to_home_idx, "pick_to_home", exec_config)) return 1;
    }

    return 0;
}
