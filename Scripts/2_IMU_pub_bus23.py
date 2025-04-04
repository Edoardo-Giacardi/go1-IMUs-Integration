#!/usr/bin/env python3
# This file publishes a ros2 message topic containing the data of two IMUs connected via I2C to the raspberry Pi 5
# Each imu is connected to an hardware I2C bus (in this case bus2 and bus3
# In each bus there IMU i2c address is 0x4A
# This publisher uses the custom msg (TwoIMU) for the IMUs

import rclpy
from rclpy.node import Node

from imu_custom_msgs.msg import TwoIMU, ImuData

import time
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE

class Bus23Pub(Node):
    def __init__(self):
        super().__init__('bus23_imus_pub')

        # Initialize the two I2C buses
        self.i2c_buses = {
            2: I2C(2),  # /dev/i2c-2 for sensor 3 (Rear Left)
            3: I2C(3),  # /dev/i2c-3 for sensor 4 (Rear Right)
        }

        # Initialize sensors 
        self.sensors = []
        try:
            sensor1 = BNO08X_I2C(self.i2c_buses[2], address=0x4A)
            sensor2 = BNO08X_I2C(self.i2c_buses[3], address=0x4A)
            self.sensors = [sensor1, sensor2]
        except Exception as e:
            self.get_logger().error(f"Failed to initialize sensor {i+3}: {e}")
            return

        time.sleep(1)  # Wait a second before enabling features

        for i, sensor in enumerate(self.sensors):
            try:
                sensor.enable_feature(BNO_REPORT_ACCELEROMETER)
                sensor.enable_feature(BNO_REPORT_GYROSCOPE)
            except RuntimeError as e:
                self.get_logger().error(f"Sensor {i+3} init error: {e}")

        # Create publisher of type TwoImuData
        self.pub_2imus_bus23 = self.create_publisher(TwoIMU, '/two_imus_bus23', 10)

        # Create a timer at ~250Hz 
        self.timer_period = 0.004  
        self.timer = self.create_timer(self.timer_period, self.publish_imus)

        self.get_logger().info("Bus23ImusPublisher node started.")

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
                self.get_logger().warn(f"Error reading sensor {i+3}: {e}")

        self.pub_2imus_bus23.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Bus23Pub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
