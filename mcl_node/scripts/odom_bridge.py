#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import serial
import math

class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_serial_bridge')

        # The publisher that feeds your C++ Particle Filter
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)

        # OPEN THE USB PORT (Ensure this is the correct port for the motor board)
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1.0)
            self.get_logger().info("Successfully connected to Motor STM32!")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        self.timer = self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        if self.serial_port.in_waiting > 500:
            self.serial_port.reset_input_buffer()
            return

        if self.serial_port.in_waiting > 0:
            try:
                raw_line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()

                # Assuming the STM32 sends: "X, Y, HEADING, V_X, V_Y"
                parts = raw_line.split(',')
                if len(parts) == 5:
                    x = float(parts[0])
                    y = float(parts[1])
                    heading = float(parts[2])
                    v_x = float(parts[3])
                    v_y = float(parts[4])

                    self.publish_odometry(x, y, heading, v_x, v_y)

            except Exception as e:
                pass # Ignore half-written lines from the serial buffer

    def publish_odometry(self, x, y, heading, v_x, v_y):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # 1. Pose (Position)
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        # 2. Pose (Orientation / Quaternion Conversion)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(heading / 2.0)
        msg.pose.pose.orientation.w = math.cos(heading / 2.0)

        # 3. Twist (Velocity)
        msg.twist.twist.linear.x = v_x
        msg.twist.twist.linear.y = v_y
        msg.twist.twist.linear.z = 0.0

        # (Optional: If the STM32 calculates rotational velocity, add it here)
        # msg.twist.twist.angular.z = v_theta

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    bridge = OdomBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()