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
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(sport_prog src/sport_prog.cpp src/common/ros2_sport_client.cpp)
add_executable(mujoco_prog src/mujoco_prog.cpp src/common/motor_crc.cpp src/common/ros2_sport_client.cpp)

ament_target_dependencies(sport_prog ${DEPENDENCY_LIST})
ament_target_dependencies(mujoco_prog ${DEPENDENCY_LIST})

install(TARGETS 
        sport_prog
        mujoco_prog
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
