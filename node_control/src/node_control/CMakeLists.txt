cmake_minimum_required(VERSION 3.0.2)
project(node_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/node_control.py
  scripts/plot_orphaned_logs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
