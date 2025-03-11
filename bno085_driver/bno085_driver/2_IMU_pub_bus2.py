#!/usr/bin/env python3
# This publisher is used for the hardware configruation consisting in an I2C buse
# In the bus there are two IMUs connected with different i2c address 0x4A and 0x4B
# This publisher uses the custom msg (TwoIMU) for the IMUs

import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import TwoIMU, ImuData

import time
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE

class Bus2Pub(Node):
    def __init__(self):
        super().__init__('bus1_imus_pub')

        # Initialize the I2C bus
        self.i2c_bus = I2C(4) # /dev/i2c-4 for sensors 3 & 4

        # Initialize sensors with different addresses on the same I2C bus
        self.sensors = []
        try:
            sensor1 = BNO08X_I2C(self.i2c_bus, address=0x4A)
            sensor2 = BNO08X_I2C(self.i2c_bus, address=0x4B)
            self.sensors = [sensor1, sensor2]
        except Exception as e:
            self.get_logger().error(f"Failed to initialize sensors: {e}")
            return

        time.sleep(1)  # Wait a second before enabling features

        for i, sensor in enumerate(self.sensors):
            try:
                sensor.enable_feature(BNO_REPORT_ACCELEROMETER)
                sensor.enable_feature(BNO_REPORT_GYROSCOPE)
            except RuntimeError as e:
                self.get_logger().error(f"Sensor {i+1} init error: {e}")

        # Create publisher of type TwoImuData
        self.pub_2imus_bus2 = self.create_publisher(TwoIMU, '/two_imus_bus2', 10)

        # Create a timer at 200Hz (0.005s)
        self.timer_period = 0.005  
        self.timer = self.create_timer(self.timer_period, self.publish_imus)

        self.get_logger().info("Bus2ImusPublisher node started.")

    def publish_imus(self):
        msg = TwoIMU()

        for i, sensor in enumerate(self.sensors):
            try:
                accel_x, accel_y, accel_z = sensor.acceleration
                gyro_x, gyro_y, gyro_z = sensor.gyro

                imu_msg = ImuData()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = f"bno{i+1}_frame"

                imu_msg.linear_acceleration.x = accel_x
                imu_msg.linear_acceleration.y = accel_y
                imu_msg.linear_acceleration.z = accel_z

                imu_msg.angular_velocity.x = gyro_x
                imu_msg.angular_velocity.y = gyro_y
                imu_msg.angular_velocity.z = gyro_z

                if i == 0:
                    msg.imu1 = imu_msg
                elif i == 1:
                    msg.imu2 = imu_msg

            except Exception as e:
                self.get_logger().warn(f"Error reading sensor {i+1} on bus#2: {e}")

        self.pub_2imus_bus2.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Bus2Pub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
