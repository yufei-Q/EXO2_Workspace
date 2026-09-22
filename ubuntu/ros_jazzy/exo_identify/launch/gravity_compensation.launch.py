from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('exo_bringup')
    identify_share = get_package_share_directory('exo_identify')
    zero_gains = '[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'

    return LaunchDescription([
        DeclareLaunchArgument(
            'formula_file',
            default_value=(
                f'{identify_share}/results/seven_dof/'
                'dynamics_identification/identified_parameters_sim.npz'),
            description='Absolute path to gravity model .npz or formula .json',
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=(
                f'{identify_share}/config/gravity_compensation.yaml'),
        ),
        DeclareLaunchArgument(
            'hardware_mapping_file',
            default_value=f'{identify_share}/config/hardware_mapping.yaml',
            description='Shared motor-to-joint calibration YAML',
        ),
        DeclareLaunchArgument(
            'identification_config_file',
            default_value=(
                f'{identify_share}/config/identification.json'),
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=(
                f'{identify_share}/urdf/exo.SLDASM/urdf/'
                '装配体.SLDASM.urdf'),
        ),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('bridge_rate', default_value='500.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f'{bringup_share}/launch/dm_motor_usb.launch.py'),
            launch_arguments={
                'port': LaunchConfiguration('port'),
                'rate': LaunchConfiguration('bridge_rate'),
                'kp': zero_gains,
                'kd': zero_gains,
                'use_gui': 'false',
            }.items(),
        ),
        Node(
            package='exo_identify',
            executable='gravity_compensation.py',
            name='exo_gravity_compensation',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'formula_file': LaunchConfiguration('formula_file'),
                    'hardware_mapping_file': LaunchConfiguration(
                        'hardware_mapping_file'),
                    'identification_config_file': LaunchConfiguration(
                        'identification_config_file'),
                    'urdf_file': LaunchConfiguration('urdf_file'),
                },
            ],
        ),
    ])
