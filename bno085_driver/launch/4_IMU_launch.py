import sys
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

def prompt_and_launch(context, *args, **kwargs):
    """Prompt the user in the terminal and return the selected Node."""
    print("Which set of IMUs do you want to launch? (front/rear)")
    sys.stdout.flush()  # Ensure the prompt is printed immediately

    choice = sys.stdin.readline().strip()  # Read user input from stdin
    if choice.lower() == 'front':
        return [Node(
            package='bno085_driver',
            executable='Front_Imus.py',
            name='front_imus_pub',
            output='screen'
        )]
    elif choice.lower() == 'rear':
        return [Node(
            package='bno085_driver',
            executable='Rear_Imus.py',
            name='rear_imus_pub',
            output='screen'
        )]
    else:
        print("Invalid choice. Please re-run and choose either 'front' or 'rear'.")
        return []  # Return an empty list so no nodes are launched

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=prompt_and_launch),
    ])


# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='bno085_driver',
#             executable='2_IMU_pub_bus01.py',  
#             name='bus01_imus_pub',
#             output='screen'
#         ),
#         Node(
#             package='bno085_driver',
#             executable='2_IMU_pub_bus23.py',  
#             name='bus23_imus_pub',
#             output='screen'
#         )
#     ])

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

