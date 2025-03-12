#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import FourImuData

class ConsolePrinterNode(Node):
    def __init__(self):
        super().__init__('console_printer')
        self.subscription = self.create_subscription(
            FourImuData,
            '/four_imus_data',
            self.callback,
            10)
    
    def callback(self, msg):
        self.get_logger().info("----------------------------------------")
        for i, imu in enumerate([msg.imu1, msg.imu2, msg.imu3, msg.imu4], start=1):
            text = (f"Sensor {i} Acceleration (m/s^2): "
                    f"X={imu.linear_acceleration.x:.6f}, "
                    f"Y={imu.linear_acceleration.y:.6f}, "
                    f"Z={imu.linear_acceleration.z:.6f}\n"
                    f"Sensor {i} Gyro (rad/s): "
                    f"X={imu.angular_velocity.x:.6f}, "
                    f"Y={imu.angular_velocity.y:.6f}, "
                    f"Z={imu.angular_velocity.z:.6f}")
            self.get_logger().info(text)

def main(args=None):
    rclpy.init(args=args)
    node = ConsolePrinterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
