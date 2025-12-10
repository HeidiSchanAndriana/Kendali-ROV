from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():
    udp=LaunchConfiguration('udp_mavros'); ns=LaunchConfiguration('ns')
    return LaunchDescription([
        DeclareLaunchArgument('udp_mavros', default_value='14560'),
        DeclareLaunchArgument('ns', default_value='/mavros/mavros_uas1'),
        Node(package='mavros', executable='mavros_node', namespace=ns,
             parameters=['config/mavros_params.yaml', {'fcu_url': ['udp://:', udp, '@']}])
    ])
