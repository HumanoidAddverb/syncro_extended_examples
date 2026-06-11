cmake_minimum_required(VERSION 3.0)

# #########---------------------#############
SET(CMAKE_INSTALL_PREFIX /usr)
set(INSTALL_FILES_DESTINATION /usr/include)

cmake_minimum_required(VERSION 3.0)

# ########## Project Name ###################
project(Heal_Cobot)

# ###########################################

# #########-----SET VARIABLES -----#############
set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(/usr/local/include/soem)
include_directories(/usr/include/soem)
include_directories(/usr/include/include)
include_directories(/usr/include/struct_mapping)

########################################################
include_directories(/usr/include/robotous_ft) 
include_directories(/usr/include/grippers/feetech_gripper)##
include_directories(/usr/include/gripper_drivers/feetech_drivers)
include_directories(/usr/include/grippers)            
include_directories(/usr/include/gripper_drivers) 
include_directories(/usr/include) 
include_directories(/usr/include/grippers/dynamixel_gripper)
include_directories(/usr/include/gripper_drivers/dynamixel_sdk)
include_directories(/usr/include/grippers/suction_gripper)
include_directories(/usr/include/gripper_drivers/suction_gripper_drivers)##
include_directories(/usr/include/grippers/dh_gripper)##
include_directories(/usr/include/gripper_drivers/dh_gripper_drivers)##

###################################################################################
add_subdirectory(pick_and_place)


file(COPY /usr/robot_data DESTINATION ${CMAKE_BINARY_DIR})
