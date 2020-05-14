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
find_package(control_toolbox REQUIRED)
find_package(xarm_api REQUIRED COMPONENTS xarm_ros_client)
find_package(xarm_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

## Declare a C++ library
add_library(xarm_hw
  src/xarm_hw.cpp
)
# add_dependencies(xarm_hw xarm_ros_client )
target_link_libraries(xarm_hw xarm_ros_client xarm_msgs)

add_library(xarm_combined_hw
  src/xarm_combined_hw.cpp
)

ament_target_dependencies(xarm_hw
        rclcpp
        std_msgs
        xarm_msgs
        sensor_msgs
        controller_manager
        joint_state_controller
        control_msgs
        xarm_api
        xarm_msgs
        hardware_interface
        control_toolbox
        )

ament_target_dependencies(xarm_combined_hw
        rclcpp
        std_msgs
        xarm_msgs
        sensor_msgs
        controller_manager
        joint_state_controller
        control_msgs
        xarm_api
        xarm_msgs
        hardware_interface
        control_toolbox
        )

add_dependencies(xarm_combined_hw xarm_hw )
target_link_libraries(xarm_combined_hw xarm_hw)

add_executable(sample_motion src/sample_motion.cpp)
ament_target_dependencies(sample_motion
        std_msgs
        rclcpp
        )

add_executable(xarm_traj_controller src/xarm_control_node.cpp)
target_link_libraries(xarm_traj_controller xarm_hw controller_manager joint_state_controller robot_state_publisher xarm_api)

add_executable(xarm_combined_traj_controller src/xarm_combined_control_node.cpp)
target_link_libraries(xarm_combined_traj_controller xarm_combined_hw controller_manager joint_state_controller robot_state_publisher xarm_api)

ament_package()