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

        # Action clients for the 3 phases of the mission
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.compute_path_client = ActionClient(self, ComputeCoveragePath, 'compute_coverage_path')
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')

    def start_mission(self, start_x, start_y, start_yaw_deg, gml_path):
        self.gml_path = gml_path
        self.get_logger().info(f'PHASE 1: Driving to starting waypoint ({start_x}, {start_y})...')

        if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 Action Server not available! Is Terminal 2 running?')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = float(start_x)
        goal_msg.pose.pose.position.y = float(start_y)

        # Orientation (Degrees to Quaternion)
        yaw_rad = math.radians(start_yaw_deg)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        self.get_logger().info('Sending transit goal...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._on_nav_accepted)

    def _on_nav_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Transit rejected by Nav2!')
            return
        self.get_logger().info('Transit accepted, moving...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_completed)

    def _on_nav_completed(self, future):
        self.get_logger().info('Arrived at start! Transitioning to Phase 2...')
        self.calculate_sweep()

    def calculate_sweep(self):
        self.get_logger().info(f'PHASE 2: Requesting sweep path for: {self.gml_path}')
        self.compute_path_client.wait_for_server()

        goal_msg = ComputeCoveragePath.Goal()
        goal_msg.use_gml_file = True
        goal_msg.gml_field_path = self.gml_path
        goal_msg.robot_width = 0.35 # 35cm lanes

        send_goal_future = self.compute_path_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._on_compute_accepted)

    def _on_compute_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Sweep calculation rejected!')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_compute_completed)

    def _on_compute_completed(self, future):
        result = future.result().result
        self.get_logger().info('Sweep calculated! Transitioning to Phase 3...')
        self.execute_sweep(result.coverage_path)

    def execute_sweep(self, path):
        self.get_logger().info('PHASE 3: Motors engaged. Following coverage path.')
        self.follow_path_client.wait_for_server()

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = 'FollowPath' # Using the DWB controller we fixed in yaml

        send_goal_future = self.follow_path_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._on_follow_accepted)

    def _on_follow_accepted(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Path following rejected!')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_mission_completed)

    def _on_mission_completed(self, future):
        self.get_logger().info('MISSION COMPLETE! The arena has been swept.')
        # Give it a second to breathe before shutting down
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    master = MasterSweepClient()

    # Settings
    START_X = 1.0
    START_Y = 0.5
    START_YAW = 90.0

    # FIXED: Variable name now matches the call below
    GML_FILE_PATH = "/home/ieee/ros2_ws/src/localizationAlgorithm/ieee_arena_boundary.gml"

    # FIXED: Passing GML_FILE_PATH instead of GML_PATH
    master.start_mission(START_X, START_Y, START_YAW, GML_FILE_PATH)

    try:
        rclpy.spin(master)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()