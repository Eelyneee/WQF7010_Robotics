cmake_minimum_required(VERSION 3.0.2)
project(interaction_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/interaction_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)