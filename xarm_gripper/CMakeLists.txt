cmake_minimum_required(VERSION 2.8.3)
project(xarm_gripper)

find_package(ament_cmake REQUIRED)

find_package(roslaunch)

install(
		DIRECTORY
		config
		urdf
		launch
		meshes
		DESTINATION share/${PROJECT_NAME}
)

ament_package()