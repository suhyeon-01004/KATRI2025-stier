cmake_minimum_required(VERSION 3.0.2)
project(gps_bringup)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## Declare catkin package
catkin_package()

## Mark other files for installation (e.g. launch files)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

