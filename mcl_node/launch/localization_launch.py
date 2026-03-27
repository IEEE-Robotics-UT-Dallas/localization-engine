import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # 1. The C++ Particle Filter Node
        Node(
            package='localization_engine',
            executable='particle_filter_node',
            name='mcl_node',
            output='screen'
        ),

        # 2. The Python Serial Bridge
        # Note: Ensure the path to your script is correct
        Node(
            package='localization_engine',
            executable='tof_bridge.py',
            name='serial_bridge',
            output='screen'
        ),

        # 2b. The Python Odom Serial Bridge
        Node(
            package='localization_engine',
            executable='odom_bridge.py',
            name='odom_bridge',
            output='screen'
        ),

        # 3. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
            # (Optional) Add arguments here later to auto-load your specific RViz config file
        )
    ])