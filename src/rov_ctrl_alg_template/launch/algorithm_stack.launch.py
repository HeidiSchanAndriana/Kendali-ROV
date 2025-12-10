from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_alg = get_package_share_directory('rov_ctrl_alg_template')
    pkg_if  = get_package_share_directory('rov_ctrl_if')

    default_yaml = os.path.join(pkg_alg, 'config', 'example_forward_right.yaml')
    mux_yaml     = os.path.join(pkg_if,  'config', 'control_mux.yaml')

    yaml_file = LaunchConfiguration('yaml_file')

    return LaunchDescription([
        DeclareLaunchArgument('yaml_file', default_value=default_yaml),

        Node(
            package='rov_ctrl_alg_template',
            executable='alg_runner',
            parameters=[yaml_file],
            output='screen'
        ),
        Node(
            package='rov_ctrl_if',
            executable='control_mux',
            parameters=[mux_yaml],
            output='screen'
        )
    ])
