cmake_minimum_required(VERSION 3.0.2)
project(exo_bringup)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package()

catkin_install_python(
  PROGRAMS scripts/node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES scripts/protocol.py scripts/__init__.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES README.md LICENSE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)