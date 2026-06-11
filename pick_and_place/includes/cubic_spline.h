/**
 * @file cubic_spline.h
 * @brief Natural cubic spline interpolation for smooth trajectory generation
 * @version 0.1
 * @date 2026-02-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef CUBIC_SPLINE_H_
#define CUBIC_SPLINE_H_

#include <vector>
#include <array>
#include <stdexcept>
#include "robot_config.h"

/**
 * @brief Natural cubic spline interpolator for multi-joint trajectories
 * 
 * Implements natural cubic splines with C² continuity for smooth robot motion.
 * Uses natural boundary conditions (second derivative = 0 at endpoints).
 */
class CubicSpline
{
public:
    /**
     * @brief Construct a new Cubic Spline object
     */
    CubicSpline();

    /**
     * @brief Destroy the Cubic Spline object
     */
    ~CubicSpline();

    /**
     * @brief Set waypoints for spline interpolation
     * 
     * @param times Vector of timestamps for each waypoint (must be strictly increasing)
     * @param positions Vector of joint positions at each waypoint [waypoint_idx][joint_idx]
     * @return true if waypoints set successfully
     * @return false if input validation fails
     */
    bool setWaypoints(const std::vector<double>& times, 
                      const std::vector<std::vector<double>>& positions);

    /**
     * @brief Get interpolated joint positions at given time
     * 
     * @param time Time at which to interpolate
     * @param positions Output array of joint positions (size N_DOF)
     * @return true if interpolation successful
     * @return false if time is out of range or spline not initialized
     */
    bool interpolate(double time, double* positions) const;

    /**
     * @brief Get interpolated joint velocities at given time
     * 
     * @param time Time at which to compute velocity
     * @param velocities Output array of joint velocities (size N_DOF)
     * @return true if computation successful
     * @return false if time is out of range or spline not initialized
     */
    bool getVelocity(double time, double* velocities) const;

    /**
     * @brief Validate that all waypoints are within joint limits
     * 
     * @param lower_limits Array of lower joint limits (size N_DOF)
     * @param upper_limits Array of upper joint limits (size N_DOF)
     * @return true if all waypoints within limits
     * @return false if any waypoint violates limits
     */
    bool validateLimits(const double* lower_limits, const double* upper_limits) const;

    /**
     * @brief Check if spline is initialized and ready for interpolation
     * 
     * @return true if initialized
     * @return false otherwise
     */
    bool isInitialized() const { return initialized_; }

    /**
     * @brief Get the time range of the spline
     * 
     * @param start_time Output start time
     * @param end_time Output end time
     * @return true if spline is initialized
     * @return false otherwise
     */
    bool getTimeRange(double& start_time, double& end_time) const;

private:
    /**
     * @brief Spline coefficients for one segment
     * 
     * Represents: s(t) = a + b*(t-ti) + c*(t-ti)^2 + d*(t-ti)^3
     */
    struct SplineSegment
    {
        double a, b, c, d;  // Coefficients
        double t_start;     // Start time of this segment
    };

    /**
     * @brief Compute spline coefficients for a single joint
     * 
     * @param times Vector of waypoint times
     * @param values Vector of joint values at waypoints
     * @param segments Output vector of spline segments
     */
    void computeSplineCoefficients(const std::vector<double>& times,
                                   const std::vector<double>& values,
                                   std::vector<SplineSegment>& segments);

    void solveTridiagonal(const std::vector<double>& a,
                         const std::vector<double>& b,
                         const std::vector<double>& c,
                         const std::vector<double>& d,
                         std::vector<double>& x);

    /**
     * @brief Find the segment index for a given time
     * 
     * @param time Query time
     * @return int Segment index, or -1 if out of range
     */
    int findSegment(double time) const;

    // Member variables
    std::vector<double> times_;                              ///< Waypoint times
    std::vector<std::vector<double>> waypoints_;             ///< Waypoints [waypoint_idx][joint_idx]
    std::vector<std::vector<SplineSegment>> spline_joints_;  ///< Spline segments [joint_idx][segment_idx]
    bool initialized_;                                        ///< Initialization flag
};

#endif // CUBIC_SPLINE_H_
