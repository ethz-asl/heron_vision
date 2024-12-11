cmake_minimum_required(VERSION 3.0.2)

# EDIT: name of package
project(heron_vision)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  heron_msgs
  cv_bridge
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
   scripts/*
   nodes/*
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
