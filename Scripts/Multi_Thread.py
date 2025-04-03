#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import time
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE

# Qui assumiamo di avere un messaggio "TwoIMU" con:
#   ImuData imu1
#   ImuData imu2
# e un msg "ImuData" con campi standard per accelerazione/vel. angolare.
from imu_custom_msgs.msg import TwoIMU, ImuData


class Bus1ImuPublisher(Node):
    def __init__(self):
        super().__init__('bus1_imus_publisher')

        # Bus I2C #1
        try:
            self.i2c_bus = I2C(1)  # /dev/i2c-1
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C bus #1: {e}")
            return

        # Due sensori sul bus #1
        self.sensors = []
        try:
            sensor1 = BNO08X_I2C(self.i2c_bus, address=0x4A)
            sensor2 = BNO08X_I2C(self.i2c_bus, address=0x4B)
            self.sensors = [sensor1, sensor2]
        except Exception as e:
            self.get_logger().error(f"Failed to initialize sensors on bus #1: {e}")
            return

        time.sleep(2)  # piccola pausa prima di abilitare i sensori

        for i, sensor in enumerate(self.sensors):
            try:
                sensor.enable_feature(BNO_REPORT_ACCELEROMETER)
                sensor.enable_feature(BNO_REPORT_GYROSCOPE)
            except RuntimeError as err:
                self.get_logger().error(f"Sensor {i+1} on bus#1 init error: {err}")

        # Publisher per i dati di 2 IMU (bus #1)
        self.pub_2imus_bus1 = self.create_publisher(TwoIMU, '/two_imus_bus1', 10)

        # Timer a 200Hz
        self.timer_period = 0.005  # 200 Hz
        self.timer = self.create_timer(self.timer_period, self.publish_imus)

        self.get_logger().info("Bus1ImuPublisher node started.")

    def publish_imus(self):
        msg = TwoIMU()

        for i, sensor in enumerate(self.sensors):
            try:
                accel_x, accel_y, accel_z = sensor.acceleration
                gyro_x, gyro_y, gyro_z = sensor.gyro

                imu_msg = ImuData()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = f"bno_bus1_{i+1}_frame"

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
                self.get_logger().warn(f"Error reading sensor {i+1} on bus#1: {e}")

        self.pub_2imus_bus1.publish(msg)


class Bus3ImuPublisher(Node):
    def __init__(self):
        super().__init__('bus3_imus_publisher')

        # Bus I2C #3
        try:
            self.i2c_bus = I2C(3)  # /dev/i2c-3
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C bus #3: {e}")
            return

        # Due sensori sul bus #3
        self.sensors = []
        try:
            sensor3 = BNO08X_I2C(self.i2c_bus, address=0x4A)
            sensor4 = BNO08X_I2C(self.i2c_bus, address=0x4B)
            self.sensors = [sensor3, sensor4]
        except Exception as e:
            self.get_logger().error(f"Failed to initialize sensors on bus #3: {e}")
            return

        time.sleep(2)

        for i, sensor in enumerate(self.sensors):
            try:
                sensor.enable_feature(BNO_REPORT_ACCELEROMETER)
                sensor.enable_feature(BNO_REPORT_GYROSCOPE)
            except RuntimeError as err:
                self.get_logger().error(f"Sensor {i+1} on bus#3 init error: {err}")

        # Publisher per i dati di 2 IMU (bus #3)
        self.pub_2imus_bus3 = self.create_publisher(TwoIMU, '/two_imus_bus3', 10)

        # Timer a 200Hz
        self.timer_period = 0.005
        self.timer = self.create_timer(self.timer_period, self.publish_imus)

        self.get_logger().info("Bus3ImuPublisher node started.")

    def publish_imus(self):
        msg = TwoIMU()

        for i, sensor in enumerate(self.sensors):
            try:
                accel_x, accel_y, accel_z = sensor.acceleration
                gyro_x, gyro_y, gyro_z = sensor.gyro

                imu_msg = ImuData()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = f"bno_bus3_{i+1}_frame"

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
                self.get_logger().warn(f"Error reading sensor {i+1} on bus#3: {e}")

        self.pub_2imus_bus3.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    bus1_node = Bus1ImuPublisher()
    bus3_node = Bus3ImuPublisher()

    # Usiamo un MultiThreadedExecutor per poter eseguire in parallelo
    # i due nodi, se lo desideriamo.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(bus1_node)
    executor.add_node(bus3_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bus1_node.destroy_node()
        bus3_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()



