/**
 * @file pick_place_config.cpp
 * @brief Implementation of pick-and-place configuration
 * @version 1.0
 * @date 2026-02-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "pick_place_config.h"
#include <fstream>
#include <sstream>
#include <iostream>

namespace PickPlaceConfig
{
    double getSegmentDuration(const std::string& segment_name,
                             const TrajectoryDurations& durations)
    {
        if (segment_name == "home_to_pick")
            return durations.home_to_pick;
        else if (segment_name == "pick_to_place")
            return durations.pick_to_place;
        else if (segment_name == "place_to_home")
            return durations.place_to_home;
        else if (segment_name == "home_to_place")
            return durations.home_to_place;
        else if (segment_name == "place_to_pick")
            return durations.place_to_pick;
        else if (segment_name == "pick_to_home")
            return durations.pick_to_home;
        else
        {
            std::cerr << "Warning: Unknown segment '" << segment_name 
                      << "', using default 2.0s" << std::endl;
            return 2.0;
        }
    }
}
