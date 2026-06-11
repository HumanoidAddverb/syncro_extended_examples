/**
 * @file pick_place_config.h
 * @brief Configuration for pick-and-place trajectory durations
 * @version 1.0
 * @date 2026-02-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef PICK_PLACE_CONFIG_H
#define PICK_PLACE_CONFIG_H

#include <string>
#include <map>

namespace PickPlaceConfig
{
    // Waypoint name constants
    const std::string HOME = "home";
    const std::string PRE_PICK_1 = "pre_pick_1";
    const std::string PRE_PICK_2 = "pre_pick_2";
    const std::string PRE_PICK_3 = "pre_pick_3";
    const std::string PICK = "pick";
    const std::string POST_PICK_1 = "post_pick_1";
    const std::string POST_PICK_2 = "post_pick_2";
    const std::string POST_PICK_3 = "post_pick_3";
    const std::string PRE_PLACE_1 = "pre_place_1";
    const std::string PRE_PLACE_2 = "pre_place_2";
    const std::string PRE_PLACE_3 = "pre_place_3";
    const std::string PLACE = "place";
    const std::string POST_PLACE_1 = "post_place_1";
    const std::string POST_PLACE_2 = "post_place_2";
    const std::string POST_PLACE_3 = "post_place_3";
    const std::string HOME_RETURN = "home";

    /**
     * @brief Trajectory segment durations (in seconds)
     * These define the TOTAL time for each major segment
     * The time will be divided equally among intermediate waypoints
     */
    struct TrajectoryDurations
    {
        // Major segment durations (total time for entire segment)
        double home_to_pick = 0.5;          // Total time: home → pre_pick_1/2/3 → pick
        double pick_to_place = 1.25;         // Total time: pick → post_pick_1/2/3 → pre_place_1/2/3 → place
        double place_to_home = 0.5;         // Total time: place → post_place_1/2/3 → home
        double home_to_place = 0.5;         // Total time: home → pre_place_1/2/3 → place
        double place_to_pick = 1.25;         // Total time: place → (reverse) → pick
        double pick_to_home = 0.5;          // Total time: pick → (reverse pre_pick) → home
    };

    /**
     * @brief Gripper control settings
     */
    struct GripperSettings
    {
        int close_force = 30;           // Gripper closing force (0-100)
        double pick_dwell_time = 4.0;   // Time to wait after closing gripper (seconds)
        double place_dwell_time = 4.0;  // Time to wait after opening gripper (seconds)
    };

    /**
     * @brief Homing sequence settings
     */
    struct HomingSettings
    {
        double duration = 3.0;      // Time to move to home position (seconds)
        double buffer_time = 2.0;   // Extra time to ensure completion (seconds)
    };

    /**
     * @brief Trajectory execution settings
     */
    struct TrajectorySettings
    {
        double position_error_threshold = 0.0;  // Position error threshold (radians)
    };

    /**
     * @brief Get duration for a trajectory segment
     * This function calculates the time per waypoint based on total segment duration
     * 
     * @param segment_name Name of the major segment (e.g., "home_to_pick")
     * @param num_waypoints Number of waypoints in the segment (including start and end)
     * @param durations TrajectoryDurations struct containing segment durations
     * @return Duration per sub-segment
     */
    double getSegmentDuration(const std::string& segment_name,
                             const TrajectoryDurations& durations);

} // namespace PickPlaceConfig

#endif // PICK_PLACE_CONFIG_H
