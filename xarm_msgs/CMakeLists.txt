cmake_minimum_required(VERSION 3.5)
project(xarm_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        #Message
        "msg/IOState.msg"
        "msg/RobotMsg.msg"

        # Service
        "srv/ClearErr.srv"
        "srv/GetAnalogIO.srv"
        "srv/GetDigitalIO.srv"
        "srv/GripperConfig.srv"
        "srv/GripperMove.srv"
        "srv/GripperState.srv"
        "srv/Move.srv"
        "srv/SetAxis.srv"
        "srv/SetDigitalIO.srv"
        "srv/SetInt16.srv"
        "srv/SetLoad.srv"
        "srv/TCPOffset.srv"

        # Srv for Planner
        "srv/ExecPlan.srv"
        "srv/JointPlan.srv"
        "srv/PosePlan.srv"
        "srv/SingleStraightPlan.srv"

        DEPENDENCIES
        builtin_interfaces
        std_msgs
        geometry_msgs
        )

ament_export_dependencies(rosidl_default_runtime)
ament_package()