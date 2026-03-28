import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the config file
    pkg_dir = get_package_share_directory('localization_engine')
    ekf_config_path = os.path.join(pkg_dir, 'config', 'ekf.yaml') # 1. The C++ MCL Particle Filter Node

        Node(
            package='localization_engine',
            executable='particle_filter_node',
            name='mcl_node',
            output='screen'
        ),


        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),

        # 2. The Python Serial Bridge
        # Note: Ensure the path to your script is correct
#        Node(
#            package='localization_engine',
#            executable='tof_bridge.py',
#            name='serial_bridge',
#            output='screen'
#        ),


        # 3. RViz2
#        Node(
 #           package='rviz2',
  #          executable='rviz2',
   #         name='rviz2',
    #        output='screen'
            # (Optional) Add arguments here later to auto-load your specific RViz config file
     #   )
    ])