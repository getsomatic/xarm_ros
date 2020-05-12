cmake_minimum_required(VERSION 3.5)
project(xarm_controller)

add_compile_options(-std=c++14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(combined_robot_hw REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_state_controller REQUIRED)
find_package(control_msgs REQUIRED)
# find_package(control_toolbox REQUIRED)
find_package(xarm_api REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(std_msgs REQUIRED)

#catkin_package(
##  INCLUDE_DIRS include
#  LIBRARIES xarm_hw
#  CATKIN_DEPENDS combined_robot_hw controller_manager joint_state_controller robot_state_publisher xarm_api
##  DEPENDS system_lib
#)

include_directories(
  include
)

## Declare a C++ library
add_library(xarm_hw
  src/xarm_hw.cpp
)
add_dependencies(xarm_hw xarm_ros_client )
target_link_libraries(xarm_hw xarm_ros_client)


add_library(xarm_combined_hw
  src/xarm_combined_hw.cpp
)
add_dependencies(xarm_combined_hw xarm_hw )
target_link_libraries(xarm_combined_hw xarm_hw)

add_executable(sample_motion src/sample_motion.cpp)
target_link_libraries(sample_motion controller_manager joint_state_controller robot_state_publisher xarm_api)

add_executable(xarm_traj_controller src/xarm_control_node.cpp)
target_link_libraries(xarm_traj_controller xarm_hw controller_manager joint_state_controller robot_state_publisher xarm_api)

add_executable(xarm_combined_traj_controller src/xarm_combined_control_node.cpp)
target_link_libraries(xarm_combined_traj_controller xarm_combined_hw controller_manager joint_state_controller robot_state_publisher xarm_api)

ament_package()