import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('xarm_api'),
        'config',
        'xarm.yaml'
    )


    node = Node(
        package='xarm_api',
        executable='xarm_driver_node',
        name='XarmRTConnection',
        parameters=[config]
    )

    ld.add_action(node)
    return ld