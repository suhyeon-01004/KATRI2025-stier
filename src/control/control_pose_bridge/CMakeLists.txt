cmake_minimum_required(VERSION 3.0.2)
project(control_pose_bridge)

add_compile_options(-std=c++14 -O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy std_msgs geometry_msgs nav_msgs
  tf2 tf2_ros tf2_geometry_msgs ublox_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs ublox_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_arbiter
  src/pose_arbiter.cpp
  src/pose_arbiter_node.cpp
)
add_executable(gps_drop_test src/gps_drop_test.cpp)
target_link_libraries(pose_arbiter ${catkin_LIBRARIES})
target_link_libraries(gps_drop_test ${catkin_LIBRARIES})
add_dependencies(gps_drop_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS pose_arbiter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS
  scripts/gps_quality_monitor.py
  src/wheel_speed_to_twist.py
  src/odom_to_pose.py
  scripts/pose_set_frame.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
