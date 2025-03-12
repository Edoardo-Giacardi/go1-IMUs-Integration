from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bno085_driver',
            executable='2_IMU_pub_bus01.py',  
            name='bus01_imus_pub',
            output='screen'
        ),
        Node(
            package='bno085_driver',
            executable='2_IMU_pub_bus23.py',  
            name='bus23_imus_pub',
            output='screen'
        )
    ])