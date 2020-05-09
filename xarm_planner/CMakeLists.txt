cmake_minimum_required(VERSION 3.5)
project(xarm_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
COMPONENTS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    message_generation
    geometry_msgs
    std_msgs
)

add_service_files(
  FILES
  pose_plan.srv
  joint_plan.srv
  exec_plan.srv
  single_straight_plan.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xarm_planner
 CATKIN_DEPENDS message_runtime geometry_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_executable(xarm_simple_planner src/xarm_simple_planner.cpp)
add_dependencies(xarm_simple_planner ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(xarm_simple_planner ${catkin_LIBRARIES})

add_executable(xarm_simple_planner_test src/xarm_simple_planner_test.cpp)
add_dependencies(xarm_simple_planner_test ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(xarm_simple_planner_test ${catkin_LIBRARIES})
