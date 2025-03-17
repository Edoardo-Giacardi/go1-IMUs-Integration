# launch file for the four imus. Select the actual configuration (2 node or 4 node)
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

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='bno085_driver',
#             executable='imu1.py',  
#             name='bus0_imu1_pub',
#             output='screen'
#         ),
#         Node(
#             package='bno085_driver',
#             executable='imu2.py',  
#             name='bus1_imu2_pub',
#             output='screen'
#         ),
#         Node(
#             package='bno085_driver',
#             executable='imu3.py',  
#             name='bus2_imu3_pub',
#             output='screen'
#         ),
#         Node(
#             package='bno085_driver',
#             executable='imu4.py',  
#             name='bus3_imu4_pub',
#             output='screen'
#         )
#     ])

