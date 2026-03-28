import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the config file
    pkg_dir = get_package_share_directory('localization_engine')
    ekf_config_path = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    return LaunchDescription([
        # 1. The Map Generator & EKF Bridge (Your C++ Node)
        # This MUST run to generate the .gml and .pgm files!
        Node(
            package='localization_engine',
            executable='particle_filter_node',
            name='localization_node',
            output='screen'
        ),

        # 2. The Robot Localization EKF Node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),

        # 3. THE BRIDGE: Locks the Map to the Odom frame for Nav2
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
    ])