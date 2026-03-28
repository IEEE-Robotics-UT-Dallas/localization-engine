import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the directory for mcl_node (now called localization_engine)
    mcl_pkg_dir = get_package_share_directory('localization_engine')

    # NEW: Get the directory for the separate efk_node package
    try:
        ekf_pkg_dir = get_package_share_directory('efk_node')
        ekf_config_path = os.path.join(ekf_pkg_dir, 'config', 'ekf.yaml')
    except:
        # Fallback if the package name is different in your package.xml
        ekf_config_path = "/home/ieee/ros2_ws/src/localizationAlgorithm/efk_node/config/ekf.yaml"

    return LaunchDescription([
        # 1. Map Generator (mcl_node)
        Node(
            package='localization_engine',
            executable='particle_filter_node',
            name='localization_node',
            output='screen'
        ),

        # 2. EKF Node (using the config from efk_node package)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),

        # 3. Static TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
    ])