#!/usr/bin/env python3
# This publisher is used for the hardware configruation consisting in 4 I2C buses. 
# For each bus there is only one IMU connected with same i2c address 0x4A
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from imu_custom_msgs.msg import FourImuData

import time
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE

class FourImusPublisher(Node):
    def __init__(self):
        super().__init__('four_imus_publisher')

        # Inizializza i 4 bus I2C
        # (Attenzione: su Raspberry Pi 5, /dev/i2c-0 potrebbe corrispondere a bus #0 o #4.
        #  Devi verificare con "ls /dev/i2c*" quali bus ci sono.)
        self.i2c_buses = {
            1: I2C(1),  # /dev/i2c-1
            2: I2C(2),  # /dev/i2c-2
            3: I2C(3),  # /dev/i2c-3
            0: I2C(0)   # /dev/i2c-0 (o /dev/i2c-4 se mappato diversamente)
        }

        # Inizializza i 4 sensori, tutti con address=0x4A ma su bus diversi
        # Attento all'ordine, se preferisci sensor1 = bus1, sensor2 = bus2, ecc...
        # per coerenza, usiamo l'ordine i2c0, i2c1, i2c2, i2c3
        self.sensors = []
        try:
            sensor1 = BNO08X_I2C(self.i2c_buses[1], address=0x4A)
            sensor2 = BNO08X_I2C(self.i2c_buses[2], address=0x4A)
            sensor3 = BNO08X_I2C(self.i2c_buses[3], address=0x4A)
            sensor4 = BNO08X_I2C(self.i2c_buses[0], address=0x4A)
            self.sensors = [sensor1, sensor2, sensor3, sensor4]
        except Exception as e:
            self.get_logger().error(f"Failed to initialize sensors: {e}")
            return

        # Attendi un secondo prima di abilitare le feature
        time.sleep(1)

        # Abilita accelerometro e giroscopio per ciascun sensore
        for i, s in enumerate(self.sensors):
            try:
                s.enable_feature(BNO_REPORT_ACCELEROMETER)
                s.enable_feature(BNO_REPORT_GYROSCOPE)
            except RuntimeError as e:
                self.get_logger().error(f"Sensor {i+1} init error: {e}")

        # Crea publisher di tipo FourImuData
        self.pub_4imus = self.create_publisher(FourImuData, '/four_imus_data', 10)

        # Crea un timer a 100Hz (0.01s)
        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.publish_imus)

        self.get_logger().info("FourImusPublisher node started.")

    def publish_imus(self):
        msg = FourImuData()

        # Leggi i dati dai 4 sensori
        for i, sensor in enumerate(self.sensors):
            try:
                accel_x, accel_y, accel_z = sensor.acceleration
                gyro_x, gyro_y, gyro_z   = sensor.gyro

                # Convertiamo in un Imu msg
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = f"bno{i+1}_frame"

                imu_msg.linear_acceleration.x = accel_x
                imu_msg.linear_acceleration.y = accel_y
                imu_msg.linear_acceleration.z = accel_z

                imu_msg.angular_velocity.x = gyro_x
                imu_msg.angular_velocity.y = gyro_y
                imu_msg.angular_velocity.z = gyro_z

                # Assegna al campo corrispondente di msg
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

        # Pubblica il messaggio
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
