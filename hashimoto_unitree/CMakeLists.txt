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

add_executable(sport_prog src/sport_prog.cpp src/common/ros2_sport_client.cpp)
add_executable(mujoco_prog src/mujoco_prog.cpp src/common/motor_crc.cpp src/common/ros2_sport_client.cpp)
#add_executable(imu_publisher src/imu_publisher.cpp)
add_executable(cmd2sport src/cmd2sport.cpp src/common/ros2_sport_client.cpp)
add_executable(waypoint_pot_goal src/waypoint_pot_goal.cpp)
add_executable(waypoint_pot_goal_turtlesim src/waypoint_pot_goal_turtlesim.cpp)

ament_target_dependencies(sport_prog ${DEPENDENCY_LIST})
ament_target_dependencies(mujoco_prog ${DEPENDENCY_LIST})
#ament_target_dependencies(imu_publisher ${DEPENDENCY_LIST})
ament_target_dependencies(cmd2sport ${DEPENDENCY_LIST})
ament_target_dependencies(waypoint_pot_goal ${DEPENDENCY_LIST})
ament_target_dependencies(waypoint_pot_goal_turtlesim ${DEPENDENCY_LIST})

install(TARGETS 
        sport_prog
        mujoco_prog
        #imu_publisher
        cmd2sport
        waypoint_pot_goal
        waypoint_pot_goal_turtlesim
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
