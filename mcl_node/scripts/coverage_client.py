#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# This is the standard action message for opennav_coverage
from opennav_coverage_msgs.action import ComputeCoveragePath

class CoverageClient(Node):
    def __init__(self):
        super().__init__('coverage_path_client')

        # Create the action client connecting to the coverage server
        self._action_client = ActionClient(self, ComputeCoveragePath, 'compute_coverage_path')

    def send_goal(self, gml_path):
        self.get_logger().info('Waiting for opennav_coverage action server...')
        self._action_client.wait_for_server()

        # Build the goal request
        goal_msg = ComputeCoveragePath.Goal()

        # Tell the server to use our generated file
        goal_msg.use_gml_file = True

        # WARNING: This must be the ABSOLUTE path to the file on the Jetson
        goal_msg.gml_field_path = gml_path

        # Optional: You can tweak the sweeping distance here (e.g., width of the robot)
        goal_msg.robot_width = 0.35 # 35cm sweeping rows

        self.get_logger().info(f'Sending coverage goal using file: {gml_path}')

        # Send the goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Coverage Goal Rejected! (Check your GML file formatting)')
            return

        self.get_logger().info('Coverage Goal Accepted! Calculating path...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Path generated successfully!')
        # The result contains the standard nav_msgs/Path that you can pass to your local planner!

        # Clean shutdown after path is received
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = CoverageClient()

    # Update this path to exactly where your Jetson saves the GML file!
    gml_absolute_path = '/ros2_ws/src/localization_engine/ieee_arena_boundary.gml'

    action_client.send_goal(gml_absolute_path)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()