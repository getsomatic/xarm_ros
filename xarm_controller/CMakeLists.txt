cmake_minimum_required(VERSION 3.5)
project(xarm_controller)
add_compile_options(-std=c++14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_state_controller REQUIRED)
find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(xarm_api REQUIRED COMPONENTS xarm_ros_client)
#find_package(xarm_api REQUIRED)
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

#add_dependencies(xarm_hw xarm_ros_client )
# target_link_libraries(xarm_hw xarm_ros_client xarm_msgs)

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

add_executable(sample_motion src/sample_motion.cpp)
ament_target_dependencies(sample_motion
        std_msgs
        rclcpp
        )

add_executable(xarm_traj_controller src/xarm_control_node.cpp)
ament_target_dependencies(xarm_traj_controller
        control_toolbox
        xarm_msgs
        xarm_api
        controller_manager
        std_msgs
        rclcpp
        )
target_link_libraries(xarm_traj_controller xarm_hw)
ament_package()