cmake_minimum_required(VERSION 3.5)
project(xarm_api)
add_compile_options(-std=c++14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(XARM_API_DIR src/xarm)
set(XARM_CORE_API_DIR src/xarm/core)
set(SERIAL_API_DIR src/serial)

add_compile_options(-fPIC)

include_directories(
 include
)

add_library(${PROJECT_NAME} SHARED
        ${XARM_CORE_API_DIR}/connect.cc
        ${XARM_CORE_API_DIR}/report_data.cc

        ${XARM_CORE_API_DIR}/common/crc16.cc
        ${XARM_CORE_API_DIR}/common/queue_memcpy.cc

        ${XARM_CORE_API_DIR}/debug/debug_print.cc

        ${XARM_CORE_API_DIR}/instruction/uxbus_cmd.cc
        ${XARM_CORE_API_DIR}/instruction/uxbus_cmd_ser.cc
        ${XARM_CORE_API_DIR}/instruction/uxbus_cmd_tcp.cc

        ${XARM_CORE_API_DIR}/os/network.cc

        ${XARM_CORE_API_DIR}/port/ser.cc
        ${XARM_CORE_API_DIR}/port/socket.cc

        ${SERIAL_API_DIR}/serial.cc
        ${SERIAL_API_DIR}/impl/unix.cc
        ${SERIAL_API_DIR}/impl/win.cc
        ${SERIAL_API_DIR}/impl/list_ports/list_ports_linux.cc
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

install(DIRECTORY include/
        DESTINATION include)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()