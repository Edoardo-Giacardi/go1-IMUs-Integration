#!/usr/bin/env python3
# This publisher is used for the hardware configruation consisting in 2 I2C buses
# For each bus there two IMUs connected with different i2c address 0x4A and 0x4B
# This publisher uses the custom msg for the IMUs
import rclpy
from rclpy.node import Node

# from sensor_msgs.msg import Imu
from imu_custom_msgs.msg import FourIMU, ImuData

import time
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE

class FourImusPublisher(Node):
    def __init__(self):
        super().__init__('four_imus_publisher')

        # Initialize the two I2C buses
        self.i2c_buses = {
            1: I2C(1),  # /dev/i2c-1 for sensors 1 & 2
            3: I2C(3),  # /dev/i2c-3 for sensors 3 & 4
        }

        # Initialize sensors with different addresses on the same I2C buses
        self.sensors = []
        try:
            sensor1 = BNO08X_I2C(self.i2c_buses[1], address=0x4A)
            sensor2 = BNO08X_I2C(self.i2c_buses[1], address=0x4B)
            sensor3 = BNO08X_I2C(self.i2c_buses[3], address=0x4A)
            sensor4 = BNO08X_I2C(self.i2c_buses[3], address=0x4B)
            self.sensors = [sensor1, sensor2, sensor3, sensor4]
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

        # Create publisher of type FourImuData
        self.pub_4imus = self.create_publisher(FourIMU, '/four_imus_data', 10)

        # Create a timer at 100Hz (0.01s)
        self.timer_period = 0.01  # 100 Hz
        self.timer = self.create_timer(self.timer_period, self.publish_imus)

        self.get_logger().info("FourImusPublisher node started.")

    def publish_imus(self):
        msg = FourIMU()



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
                elif i == 2:
                    msg.imu3 = imu_msg
                elif i == 3:
                    msg.imu4 = imu_msg

            except Exception as e:
                self.get_logger().warn(f"Error reading sensor {i+1}: {e}")

        self.pub_4imus.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FourImusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
