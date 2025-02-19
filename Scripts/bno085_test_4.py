import time
import board
import busio

# The main BNO08x driver class
import adafruit_bno08x
# The specific class for I2C usage
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_GYROSCOPE
from adafruit_bno08x.i2c import BNO08X_I2C

# Create I2C buses on different sets of pins
i2c1 = I2C(1)  # /dev/i2c-1
i2c2 = I2C(2)  # /dev/i2c-2
i2c3 = I2C(3)  # /dev/i2c-3
i2c0 = I2C(0)  # /dev/i2c-4
# Initialize sensors on different buses (both can use the same I2C address in this case)
sensor1 = BNO08X_I2C(i2c1, address=0x4A)
sensor2 = BNO08X_I2C(i2c2, address=0x4A)
sensor3 = BNO08X_I2C(i2c3, address=0x4A)
sensor4 = BNO08X_I2C(i2c0, address=0x4A)
time.sleep(1) # Wait for 1 second before enabling features
# Enable features for both sensors
for sensor in [sensor1, sensor2, sensor3, sensor4]:
    try:
        sensor.enable_feature(BNO_REPORT_ACCELEROMETER)
        sensor.enable_feature(BNO_REPORT_GYROSCOPE)
    except RuntimeError as e:
        print(f"Failed to initialize sensor features: {e}")

while True:
    for i, sensor in enumerate([sensor1, sensor2, sensor3, sensor4], 1):
        try:
            accel_x, accel_y, accel_z = sensor.acceleration  # pylint:disable=no-member
            gyro_x, gyro_y, gyro_z = sensor.gyro  # pylint:disable=no-member

            print(f"Sensor {i} Acceleration (m/s^2): X={accel_x:.6f}, Y={accel_y:.6f}, Z={accel_z:.6f}")
            print(f"Sensor {i} Gyro (rad/s): X={gyro_x:.6f}, Y={gyro_y:.6f}, Z={gyro_z:.6f}")
            print("-" * 20)
        except Exception as e:
            print(f"Error reading from Sensor {i}: {e}")

    print("-" * 40)
    time.sleep(0.1)
