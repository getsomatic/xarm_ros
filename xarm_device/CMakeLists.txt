cmake_minimum_required(VERSION 3.5)
project(xarm_device)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

install(
        DIRECTORY
        scripts/camera
        DESTINATION share/${PROJECT_NAME}
)

ament_package()