#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import re

class SerialBridge(Node):
    def __init__(self):
        super().__init__('tof_serial_bridge')

        # The publisher that talks to your C++ node
        self.publisher_ = self.create_publisher(Float32MultiArray, '/tof_array', 10)

        # OPEN THE USB PORT (You may need to change ttyUSB0 to ttyACM0 depending on the STM32)
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1.0)
            self.get_logger().info("Successfully connected to STM32!")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return

        # Check the serial port extremely fast (100 times a second)
        self.timer = self.create_timer(0.01, self.read_serial)

    def read_serial(self):
        if self.serial_port.in_waiting > 0:
            try:
                # 1. Read the raw line
                raw_line = self.serial_port.readline().decode('utf-8').strip()

                # DEBUG: See exactly what the STM32 sent
                self.get_logger().info(f"RAW: {raw_line}")

                # 2. Match your printf format
                match = re.search(r'TOF1:(\d+)\s+TOF2:(\d+)\s+TOF3:(\d+)\s+TOF4:(\d+)\s+TOF5:(\d+)', raw_line)

                if match:
                    # 3. Convert mm to meters
                    distances = [float(match.group(i)) / 1000.0 for i in range(1, 6)]

                # DEBUG: See the processed meters
                    self.get_logger().info(f"METERS: {distances}")

                    msg = Float32MultiArray()
                    msg.data = distances
                    self.publisher_.publish(msg)
                else:
                    self.get_logger().warn("Regex failed to match the RAW string.")

            except Exception as e:
                self.get_logger().warn(f"Serial Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    bridge = SerialBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()