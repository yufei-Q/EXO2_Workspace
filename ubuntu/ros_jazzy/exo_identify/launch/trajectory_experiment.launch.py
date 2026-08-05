from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('exo_bringup')
    identify_share = get_package_share_directory('exo_identify')

    return LaunchDescription([
        DeclareLaunchArgument(
            'trajectory_file',
            description='Absolute path to excitation_id.csv',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=f'{identify_share}/config/experiment.yaml',
            description='Experiment mapping and safety parameter YAML',
        ),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('bridge_rate', default_value='500.0'),
        DeclareLaunchArgument(
            'kp', default_value='[10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0]'),
        DeclareLaunchArgument(
            'kd', default_value='[5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0]'),
        DeclareLaunchArgument('auto_enable', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{bringup_share}/launch/dm_motor_usb.launch.py'),
            launch_arguments={
                'port': LaunchConfiguration('port'),
                'rate': LaunchConfiguration('bridge_rate'),
                'kp': LaunchConfiguration('kp'),
                'kd': LaunchConfiguration('kd'),
                'use_gui': 'false',
            }.items(),
        ),
        Node(
            package='exo_identify',
            executable='trajectory_experiment.py',
            name='exo_trajectory_experiment',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'trajectory_file': LaunchConfiguration('trajectory_file'),
                    'auto_enable_on_prepare': ParameterValue(
                        LaunchConfiguration('auto_enable'), value_type=bool),
                },
            ],
        ),
    ])
