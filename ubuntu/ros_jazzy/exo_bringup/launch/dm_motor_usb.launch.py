from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='USB CDC port connected to the STM32 controller',
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='500.0',
            description='USB command transmission frequency in Hz',
        ),
        Node(
            package='exo_bringup',
            executable='dm_motor_usb_node',
            name='dm_motor_usb',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'rate': ParameterValue(
                    LaunchConfiguration('rate'), value_type=float),
                'kp': [0.0] * 7,
                'kd': [0.0] * 7,
            }],
        ),
    ])