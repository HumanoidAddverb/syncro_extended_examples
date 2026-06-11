
# --- cubic_spline: compiled as an object library shared by executables ---
add_library(cubic_spline_lib STATIC
  src/cubic_spline.cpp
)
target_include_directories(cubic_spline_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/includes)

# --- pick_place_config: compiled as an object library shared by executables ---
add_library(pick_place_config_lib STATIC
  src/pick_place_config.cpp)
target_include_directories(pick_place_config_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/includes)
target_link_libraries(pick_place_config_lib PUBLIC cubic_spline_lib)

add_executable(collect_pick_place_waypoints src/collect_pick_place_waypoints.cpp)
target_link_libraries(collect_pick_place_waypoints PUBLIC system_manager timer)

add_executable(execute_pick_place_spline src/execute_pick_place_spline.cpp)
target_link_libraries(execute_pick_place_spline PUBLIC system_manager timer cubic_spline_lib pick_place_config_lib)
