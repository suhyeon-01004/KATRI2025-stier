cmake_minimum_required(VERSION 3.0.2)
project(rddf_recorder)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geometry_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp roslib geometry_msgs
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(rddf_recorder src/rddf_recorder.cpp)
target_link_libraries(rddf_recorder
  ${catkin_LIBRARIES}
)


## Mark other files for installation (e.g. launch files)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

