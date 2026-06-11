/**
 * @file collect_pick_place_waypoints.cpp
 * @brief Variable-count waypoint collection for pick-and-place
 * @version 2.0
 * @date 2026-05-20
 *
 * Keyboard:
 *   h = mark current pose as HOME (typically first)
 *   w = mark current pose as an intermediate waypoint
 *   p = mark current pose as PICK
 *   l = mark current pose as PLACE
 *   q = finish and save
 *
 * Expected collection order (loose):
 *   home, [wp_*]..., pick, [wp_*]..., place, [wp_*]...
 *
 * Phase durations are configured in include/execute_spline.h, not here.
 */

#include "system_manager.h"
#include "timer.h"
#include "ee_config.h"
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>

struct CollectedWaypoint
{
    std::string name;
    std::vector<double> joints;
};

std::mutex input_mutex;
std::vector<char> pending_marks;
bool collection_complete = false;

void keyboardInputThread()
{
    std::cout << "\nKeys: h=home  w=waypoint  p=pick  l=place  q=quit\n" << std::endl;
    while (true)
    {
        std::string input;
        std::cin >> input;
        if (input.empty()) continue;
        char c = std::tolower(input[0]);

        std::lock_guard<std::mutex> lock(input_mutex);
        if (c == 'q')
        {
            collection_complete = true;
            break;
        }
        if (c == 'h' || c == 'w' || c == 'p' || c == 'l')
        {
            pending_marks.push_back(c);
        }
        else
        {
            std::cout << "Invalid key. Use h/w/p/l/q.\n";
        }
    }
}

bool saveWaypointsToCSV(const std::string& filename,
                        const std::vector<CollectedWaypoint>& waypoints)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error: Could not open file " << filename << " for writing" << std::endl;
        return false;
    }

    file << "waypoint_name,duration";
    for (int i = 0; i < N_DOF; i++) file << ",joint" << i;
    file << "\n";

    file << std::fixed << std::setprecision(6);
    for (const auto& wp : waypoints)
    {
        file << wp.name << ",0.0";
        for (int j = 0; j < N_DOF; j++) file << "," << wp.joints[j];
        file << "\n";
    }
    file.close();
    return true;
}

int main(int argc, char** argv)
{
    std::string output_filename = "pick_place_waypoints.csv";
    if (argc > 1) output_filename = argv[1];
    std::cout << "Output file: " << output_filename << std::endl;

    SystemManager stk;
    Config data;
    AlliedData interrupt;
    RobotData rob_data;
    Timer tm;

    EEInertia ee;
    ee.mass = EE_MASS;
    for (int i = 0; i < 3; i++) ee.com[i] = EE_COM[i];
    for (int i = 0; i < 6; i++) ee.inertia[i] = EE_MOI[i];
    data.reset_ee = true;
    data.iner = ee;

    data.reset_controller = static_cast<int>(ControllerEnum::eLinearVelocity);
    data.safety_type = 2;
    for (int i = 0; i < N_DOF; i++) data.target_position[i] = 0.0;
    data.delta_t = 10.0;

    if (!stk.setupRobot(data))
    {
        std::cerr << "Failed to setup robot" << std::endl;
        return 1;
    }
    std::cout << "Robot setup successful. Moving to home." << std::endl;

    tm.timeReset();
    double start_time = tm.getCurTime();
    double buffer_time = 3.0;
    bool moving_to_home = true;
    while (moving_to_home)
    {
        if (tm.getCurTime() - start_time > data.delta_t + buffer_time)
            moving_to_home = false;
        if (!stk.doControl(interrupt))
        {
            std::cerr << "Control failed during homing" << std::endl;
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::cout << "Reached home position" << std::endl;

    data.reset_controller = static_cast<int>(ControllerEnum::eFreeDrive);
    data.safety_type = 1;
    if (!stk.reset(data))
    {
        std::cerr << "Failed to reset to freedrive mode" << std::endl;
        return 1;
    }
    std::cout << "Switched to freedrive mode" << std::endl;

    std::thread input_thread(keyboardInputThread);

    std::vector<CollectedWaypoint> waypoints;
    int wp_counter = 0;
    bool has_pick = false;
    bool has_place = false;

    bool running = true;
    while (running)
    {
        if (!stk.doControl(interrupt))
        {
            std::cerr << "Control failed" << std::endl;
            break;
        }

        {
            std::lock_guard<std::mutex> lock(input_mutex);
            while (!pending_marks.empty())
            {
                char c = pending_marks.front();
                pending_marks.erase(pending_marks.begin());

                stk.getRobotData(rob_data);
                CollectedWaypoint wp;
                wp.joints.assign(rob_data.jpos, rob_data.jpos + N_DOF);

                if (c == 'h') wp.name = "home";
                else if (c == 'p') {
                    if (has_pick) { std::cout << "Pick already marked — skipping.\n"; continue; }
                    wp.name = "pick"; has_pick = true;
                }
                else if (c == 'l') {
                    if (has_place) { std::cout << "Place already marked — skipping.\n"; continue; }
                    wp.name = "place"; has_place = true;
                }
                else /* w */ wp.name = "wp_" + std::to_string(++wp_counter);

                waypoints.push_back(wp);

                std::cout << ">>> Marked [" << wp.name << "] (#" << waypoints.size() << "): ";
                for (int i = 0; i < N_DOF; i++)
                    std::cout << std::fixed << std::setprecision(3) << wp.joints[i] << " ";
                std::cout << std::endl;
            }

            if (collection_complete) running = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    input_thread.join();

    if (waypoints.size() < 2)
    {
        std::cerr << "\nNeed at least 2 waypoints. Collected: " << waypoints.size() << std::endl;
        return 1;
    }
    if (!has_pick || !has_place)
    {
        std::cerr << "\nMust mark at least one 'pick' (p) and one 'place' (l)." << std::endl;
        return 1;
    }

    std::cout << "\nCollected " << waypoints.size() << " waypoints" << std::endl;
    if (saveWaypointsToCSV(output_filename, waypoints))
        std::cout << "Saved to: " << output_filename << std::endl;
    else
        return 1;

    return 0;
}
