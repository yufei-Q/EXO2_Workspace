from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        DeclareLaunchArgument(
            'kp',
            default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
            description='Seven MIT position gains, one per CAN ID',
        ),
        DeclareLaunchArgument(
            'kd',
            default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
            description='Seven MIT damping gains, one per CAN ID',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='false',
            choices=['true', 'false'],
            description='Start the graphical motor debugging controller',
        ),
        DeclareLaunchArgument(
            'gui_d4340_velocity_limit',
            default_value='2.0',
            description='Absolute D4340P GUI velocity limit in rad/s',
        ),
        DeclareLaunchArgument(
            'gui_d4340_torque_limit',
            default_value='2.0',
            description='Absolute D4340P GUI torque limit in N*m',
        ),
        DeclareLaunchArgument(
            'gui_d4310_velocity_limit',
            default_value='2.0',
            description='Absolute D4310P GUI velocity limit in rad/s',
        ),
        DeclareLaunchArgument(
            'gui_d4310_torque_limit',
            default_value='2.0',
            description='Absolute D4310P GUI torque limit in N*m',
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
                'kp': ParameterValue(
                    LaunchConfiguration('kp'), value_type=List[float]),
                'kd': ParameterValue(
                    LaunchConfiguration('kd'), value_type=List[float]),
            }],
        ),
        Node(
            package='exo_bringup',
            executable='dm_motor_control_gui',
            name='dm_motor_control_gui',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_gui')),
            parameters=[{
                'd4340_velocity_limit': ParameterValue(
                    LaunchConfiguration('gui_d4340_velocity_limit'),
                    value_type=float),
                'd4340_torque_limit': ParameterValue(
                    LaunchConfiguration('gui_d4340_torque_limit'),
                    value_type=float),
                'd4310_velocity_limit': ParameterValue(
                    LaunchConfiguration('gui_d4310_velocity_limit'),
                    value_type=float),
                'd4310_torque_limit': ParameterValue(
                    LaunchConfiguration('gui_d4310_torque_limit'),
                    value_type=float),
            }],
        ),
    ])
