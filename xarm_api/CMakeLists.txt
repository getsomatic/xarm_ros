cmake_minimum_required(VERSION 3.5)
project(xarm_api)
add_compile_options(-std=c++14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(XARM_API_DIR src/xarm)

include_directories(
 include
)

add_library(${PROJECT_NAME} SHARED
   ${XARM_API_DIR}/connect.cc
   ${XARM_API_DIR}/report_data.cc
   ${XARM_API_DIR}/common/crc16.cc
   ${XARM_API_DIR}/common/queue_memcpy.cc
   ${XARM_API_DIR}/debug/debug_print.cc
   ${XARM_API_DIR}/instruction/uxbus_cmd.cc
   ${XARM_API_DIR}/instruction/uxbus_cmd_ser.cc
   ${XARM_API_DIR}/instruction/uxbus_cmd_tcp.cc
   ${XARM_API_DIR}/linux/network.cc
   ${XARM_API_DIR}/linux/thread.cc
   ${XARM_API_DIR}/port/serial.cc
   ${XARM_API_DIR}/port/socket.cc
 )

ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        std_msgs
        xarm_msgs
        sensor_msgs
        geometry_msgs
        )

ament_export_libraries(${PROJECT_NAME})

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
######################################################################

add_library(xarm_ros_client SHARED
  src/xarm_ros_client.cpp
)

ament_target_dependencies(xarm_ros_client
        rclcpp
        std_msgs
        xarm_msgs
        geometry_msgs
        sensor_msgs
        )

ament_export_libraries(xarm_ros_client)
#ament_export_dependencies(xarm_ros_client)

install(
        TARGETS xarm_ros_client
        EXPORT export_xarm_ros_client
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

#########################################################################

add_executable(example1_report_norm_node test/example1_report_norm.cc )
target_link_libraries(example1_report_norm_node ${PROJECT_NAME})

add_executable(xarm_driver_node src/xarm_driver_node.cpp src/xarm_driver.cpp )
target_link_libraries(xarm_driver_node ${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
        xarm_driver_node
        DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()