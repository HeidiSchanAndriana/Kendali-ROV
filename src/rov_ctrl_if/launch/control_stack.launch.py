from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ns = LaunchConfiguration('ns')
    mux_yaml = os.path.join(get_package_share_directory('rov_ctrl_if'), 'config', 'control_mux.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('ns', default_value='/mavros/mavros_uas1'),
        Node(
            package='rov_ctrl_if',
            executable='control_mux',
            parameters=[mux_yaml, {'ns': ns}],
            output='screen'
        )
    ])
