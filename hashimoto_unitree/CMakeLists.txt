cmake_minimum_required(VERSION 3.5)
project(hashimoto_unitree)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  include/common
  include/nlohmann)
link_directories(src)

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_api
  rclcpp
  std_msgs
  rosbag2_cpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_geometry_msgs
  turtlesim
  visualization_msgs
  obstacle_detector
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(obstacle_detector REQUIRED)

add_executable(sport_prog src/sport_prog.cpp src/common/ros2_sport_client.cpp)
add_executable(mujoco_prog src/mujoco_prog.cpp src/common/motor_crc.cpp src/common/ros2_sport_client.cpp)
add_executable(imu_publisher src/imu_publisher.cpp)
add_executable(cmd2sport src/cmd2sport.cpp src/common/ros2_sport_client.cpp)
add_executable(waypoint_pot_goal src/waypoint_pot_goal.cpp)
add_executable(waypoint_pot_goal_turtlesim src/waypoint_pot_goal_turtlesim.cpp)
add_executable(rotate_matrix src/rotate_matrix.cpp)
add_executable(goal_pose_editor src/goal_pose_editor.cpp)
add_executable(cmd2climb src/cmd2climb.cpp src/common/ros2_sport_client.cpp)
add_executable(height_map src/height_map.cpp)
add_executable(obstacle_hit src/obstacle_hit.cpp)
add_executable(cmd2sport_stop src/cmd2sport_stop.cpp src/common/ros2_sport_client.cpp)

target_include_directories(rotate_matrix
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(sport_prog ${DEPENDENCY_LIST})
ament_target_dependencies(mujoco_prog ${DEPENDENCY_LIST})
ament_target_dependencies(imu_publisher ${DEPENDENCY_LIST})
ament_target_dependencies(cmd2sport ${DEPENDENCY_LIST})
ament_target_dependencies(waypoint_pot_goal ${DEPENDENCY_LIST})
ament_target_dependencies(waypoint_pot_goal_turtlesim ${DEPENDENCY_LIST})
ament_target_dependencies(goal_pose_editor ${DEPENDENCY_LIST})
ament_target_dependencies(cmd2climb ${DEPENDENCY_LIST})
ament_target_dependencies(height_map ${DEPENDENCY_LIST})
ament_target_dependencies(obstacle_hit ${DEPENDENCY_LIST})
ament_target_dependencies(cmd2sport_stop ${DEPENDENCY_LIST})

ament_target_dependencies(rotate_matrix ${DEPENDENCY_LIST})

install(TARGETS 
        sport_prog
        mujoco_prog
        imu_publisher
        cmd2sport
        waypoint_pot_goal
        waypoint_pot_goal_turtlesim
        rotate_matrix
        goal_pose_editor
        cmd2climb
        height_map
        obstacle_hit
        cmd2sport_stop
        DESTINATION lib/${PROJECT_NAME})
        
install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
