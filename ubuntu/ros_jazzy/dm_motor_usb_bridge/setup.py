from setuptools import find_packages, setup


package_name = 'dm_motor_usb_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch',
         ['launch/dm_motor_usb.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='EXO developer',
    maintainer_email='user@example.com',
    description='USB CDC bridge between ROS 2 Jazzy and seven DM motors.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dm_motor_usb_node = dm_motor_usb_bridge.node:main',
        ],
    },
)
