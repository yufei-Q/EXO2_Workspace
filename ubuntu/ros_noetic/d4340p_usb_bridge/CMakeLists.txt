cmake_minimum_required(VERSION 3.0.2)
project(d4340p_usb_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package()

catkin_install_python(
  PROGRAMS scripts/d4340p_usb_node.py
           scripts/usb_loopback_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
