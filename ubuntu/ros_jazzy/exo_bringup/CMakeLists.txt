cmake_minimum_required(VERSION 3.8)
project(exo_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcl_interfaces REQUIRED)
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
  PROGRAMS scripts/control_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dm_motor_control_gui
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  FILES README.md LICENSE
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_trajectory_interlock
    test/test_trajectory_interlock.py
    TIMEOUT 30
  )
endif()

ament_package()
