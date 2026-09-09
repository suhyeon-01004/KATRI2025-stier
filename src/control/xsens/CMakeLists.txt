cmake_minimum_required(VERSION 3.0.2)
project(xsens)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf
  ublox_msgs
)

find_package(Eigen3 REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs tf ublox_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/control/ROS-UTM-LLA/include
)

add_executable(gps_imu_fusion
  src/gps_imu_fusion.cpp
  ${CMAKE_SOURCE_DIR}/control/ROS-UTM-LLA/src/utm_lla_converter.cpp
)
target_link_libraries(gps_imu_fusion
  ${catkin_LIBRARIES}
  Geographic
)
add_executable(test_fake_sensors src/test_fake_sensors.cpp)
target_link_libraries(test_fake_sensors ${catkin_LIBRARIES})

