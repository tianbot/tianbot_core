import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='460800'),
        DeclareLaunchArgument('type', default_value='omni'),
        DeclareLaunchArgument('type_verify', default_value='true'),
        DeclareLaunchArgument('publish_tf', default_value='true'),

        Node(
            package='tianbot_core',
            executable='tianbot_core',
            name='tianbot_core',
            output='screen',
            parameters=[
                {'serial_port': LaunchConfiguration('serial_port')},
                {'serial_baudrate': LaunchConfiguration('serial_baudrate')},
                {'type': LaunchConfiguration('type')},
                {'type_verify': LaunchConfiguration('type_verify')},
                {'publish_tf': LaunchConfiguration('publish_tf')},
            ]
        )
    ])
