#!/usr/bin/env python3
# This file publishes a ros2 message topic containing the data of an IMU connected via I2C to the raspberry Pi 5
# In the bus there is an IMU connected with address 0x4A 
# This publisher uses the custom msg (ImuData) for the IMU

import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import ImuData

import time
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE

class Bus0Pub_Imu1(Node):
    def __init__(self):
        super().__init__('bus0_imu1_pub')

        # Initialize the I2C bus
        self.i2c_bus = I2C(0) # /dev/i2c-0 for sensor 1 

        # Initialize sensor
        try:
            self.sensor = BNO08X_I2C(self.i2c_bus, address=0x4A)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize sensor: {e}")
            return

        time.sleep(1)  # Wait a second before enabling features
        try:
            self.sensor.enable_feature(BNO_REPORT_ACCELEROMETER)
            self.sensor.enable_feature(BNO_REPORT_GYROSCOPE)
        except RuntimeError as e:
            self.get_logger().error(f"Sensor init error: {e}")

        # Create publisher of type ImuData
        self.pub_imu1_bus0 = self.create_publisher(ImuData, '/imu1_bus0', 10)

        # Create a timer at 200Hz (0.005s)
        self.timer_period = 0.0045  
        self.timer = self.create_timer(self.timer_period, self.publish_imu)

        self.get_logger().info("Bus0Imu1Publisher node started.")

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
            self.get_logger().warn(f"Error reading sensor on bus#0: {e}")

        self.pub_imu1_bus0.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Bus0Pub_Imu1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
