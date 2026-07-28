cmake_minimum_required(VERSION 3.8)
project(exo_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

ament_python_install_package(scripts)

install(
  PROGRAMS scripts/node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dm_motor_usb_node
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md LICENSE
  DESTINATION share/${PROJECT_NAME}
)

ament_package()