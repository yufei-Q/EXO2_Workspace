from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dm_motor_usb_bridge',
            executable='dm_motor_usb_node',
            name='dm_motor_usb',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',
                'rate': 500.0,
                'kp': [0.0] * 7,
                'kd': [0.0] * 7,
            }],
        ),
    ])
