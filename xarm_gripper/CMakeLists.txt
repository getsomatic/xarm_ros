cmake_minimum_required(VERSION 3.5)
project(xarm_gripper)

find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(gazebo REQUIRED)
# find_package(joint_state_publisher REQUIRED)

install(
		DIRECTORY
		config
		urdf
		launch
		meshes
		DESTINATION share/${PROJECT_NAME}
)

ament_package()