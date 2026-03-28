#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, FollowPath
from opennav_coverage_msgs.action import ComputeCoveragePath
import math

class MasterSweepClient(Node):
    def __init__(self):
        super().__init__('master_sweep_client')

        # We need 3 separate action clients to talk to Nav2's different "brains"
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.compute_path_client = ActionClient(self, ComputeCoveragePath, 'compute_coverage_path')
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')

    # =========================================================================
    # PHASE 1: Drive to the Starting Waypoint
    # =========================================================================
    def start_mission(self, start_x, start_y, start_yaw_deg, gml_path):
        self.gml_path = gml_path
        self.get_logger().info(f'PHASE 1: Driving to starting waypoint ({start_x}, {start_y})...')

        self.nav_to_pose_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(start_x)
        goal_msg.pose.pose.position.y = float(start_y)

        # Convert degrees to quaternion
        yaw_rad = math.radians(start_yaw_deg)
        goal_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        # Send goal and wait for it to finish driving
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_nav_accepted)

    def _on_nav_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Transit rejected by Nav2!')
            return
        future = goal_handle.get_result_async()
        future.add_done_callback(self._on_nav_completed)

    def _on_nav_completed(self, future):
        self.get_logger().info('Arrived at start! Transitioning to Phase 2...')
        self.calculate_sweep()

    # =========================================================================
    # PHASE 2: Calculate the Sweeping Path
    # =========================================================================
    def calculate_sweep(self):
        self.get_logger().info(f'PHASE 2: Calculating sweep from {self.gml_path}...')
        self.compute_path_client.wait_for_server()

        goal_msg = ComputeCoveragePath.Goal()
        goal_msg.use_gml_file = True
        goal_msg.gml_field_path = self.gml_path
        goal_msg.robot_width = 0.35 # 35cm sweeping rows

        future = self.compute_path_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_compute_accepted)

    def _on_compute_accepted(self, future):
        goal_handle = future.result()
        future = goal_handle.get_result_async()
        future.add_done_callback(self._on_compute_completed)

    def _on_compute_completed(self, future):
        result = future.result().result
        self.get_logger().info('Sweep calculated! Transitioning to Phase 3...')

        # We grab the physical path that the server just calculated
        generated_path = result.coverage_path
        self.execute_sweep(generated_path)

    # =========================================================================
    # PHASE 3: Drive the Sweeping Path
    # =========================================================================
    def execute_sweep(self, path):
        self.get_logger().info('PHASE 3: Executing sweep! Motors engaged.')
        self.follow_path_client.wait_for_server()

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        # Optional: You can specify a custom controller here, or leave blank for default
        goal_msg.controller_id = ''

        future = self.follow_path_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_follow_accepted)

    def _on_follow_accepted(self, future):
        goal_handle = future.result()
        future = goal_handle.get_result_async()
        future.add_done_callback(self._on_mission_completed)

    def _on_mission_completed(self, future):
        self.get_logger().info('MISSION COMPLETE! The arena has been swept.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    master = MasterSweepClient()

    # 1. Define your starting coordinate here (e.g., X=1.0m, Y=0.5m, facing 90 degrees)
    START_X = 1.0
    START_Y = 0.5
    START_YAW = 90.0

    # 2. Point it to your generated GML file
    GML_PATH = '/ros2_ws/src/localization_engine/ieee_arena_boundary.gml'

    # Fire it off!
    master.start_mission(START_X, START_Y, START_YAW, GML_PATH)
    rclpy.spin(master)

if __name__ == '__main__':
    main()