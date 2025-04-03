#!/usr/bin/env python3
# This publisher is used for the hardware configruation consisting in an I2C buse
# In the bus there are two IMUs connected with different i2c address 0x4A and 0x4B
# This publisher uses the custom msg (TwoIMU) for the IMUs

import rclpy
from rclpy.node import Node

from imu_custom_msgs.msg import ImuData

import time
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE

class Bus2Pub_Imu4(Node):
    def __init__(self):
        super().__init__('bus2_imu4_pub')

        # Initialize the I2C bus
        self.i2c_bus = I2C(4) # /dev/i2c-4 for sensors 4

        # Initialize sensors with different addresses on the same I2C bus 
        try:
            self.sensor = BNO08X_I2C(self.i2c_bus, address=0x4B)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize sensors: {e}")
            return

        time.sleep(1)  # Wait a second before enabling features
        try:
            self.sensor.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.sensor.enable_feature(BNO_REPORT_GYROSCOPE)
        except RuntimeError as e:
            self.get_logger().error(f"Sensor init error: {e}")

        # Create publisher of type TwoImuData
        self.pub_imu4_bus2 = self.create_publisher(ImuData, '/imu4_bus2', 10)

        # Create a timer at 200Hz (0.005s)
        self.timer_period = 0.004  
        self.timer = self.create_timer(self.timer_period, self.publish_imu)

        self.get_logger().info("Bus2Imu4Publisher node started.")

    def publish_imu(self):
        imu_msg = ImuData()
        try:
            accel_x, accel_y, accel_z = self.sensor.acceleration
            gyro_x, gyro_y, gyro_z = self.sensor.gyro

            imu_msg = ImuData()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = f"bno_frame"

            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z

            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z

        except Exception as e:
            self.get_logger().warn(f"Error reading sensor on bus#2: {e}")

        self.pub_imu4_bus2.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Bus2Pub_Imu4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
