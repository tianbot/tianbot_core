from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

        
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=LaunchConfiguration('serial_port',  default='/dev/ttyUSB0')),
        DeclareLaunchArgument('serial_baudrate', default_value=LaunchConfiguration('serial_baudrate',  default='115200')),
      
        Node(
            package='tianbot_core',
            executable='tianbot_core',
            output='screen',
            parameters=[
                {'serial_port': LaunchConfiguration('serial_port')},
                {'serial_baudrate': LaunchConfiguration('serial_baudrate')},
                {'type': 'ackermann'},
                {'type_verify': False},
                {'publish_tf': False}
            ]
        ),

        # Node(
        #     package='robot_localization',
        #     executable='ekf_localization_node',
        #     name='ekf_se',
        #     clear_parameters=True,
        #     parameters=[{'file': os.path.join(os.path.dirname(__file__), '..', 'param', 'tianbot_ekf_params.yaml')}]
        # )
    ])
