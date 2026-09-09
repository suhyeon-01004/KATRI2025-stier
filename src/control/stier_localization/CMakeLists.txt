cmake_minimum_required(VERSION 3.0.2)

# 패키지의 이름으로, package.xml에서 입력한 패키지 이름을 그대로 사용
project(stier_localization)

# C++17을 사용하도록 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 캐킨 빌드할 시 요구되는 구성 요소 패키지. 사용자가 만든 패키지가 의존하는 다른 패키지를 먼저 설치하는 옵션
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  message_runtime
  roslib
  sbg_driver
  ublox_msgs
)

# ROS 이외의 패키지를 사용하는 예: Boost를 사용할 때 system 패키지를 설치하도록 함
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# 의존하는 메시지를 설정
# DEPENDENCIES: 아래에 해당하는 메시지 패키지를 사용한다는 의미
# std_msgs, sensor_msgs가 그 예시
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  ublox_msgs
)

# 캐킨 빌드 옵션
## INCLUDE: 뒤에 설정한 패키지 내부 폴더인 include의 헤더 파일을 사용함
## LIBRARIES: 뒤에 설정한 패키지의 라이브러리를 사용함
## CATKIN_DEPENDS: 의존하는 패키지 지정
## DEPENDS: 시스템 의존 패키지
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stier
 CATKIN_DEPENDS roscpp rospy std_msgs message_runtime ublox_msgs
#  DEPENDS system_lib
)

# add_executable(${PROJECT_NAME}_node src/stier_node.cpp)

#add_executable(rddf src/rddf.cpp)
#target_link_libraries(rddf ${catkin_LIBRARIES})
#add_dependencies(rddf ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(utm_navpvt_to_odom src/utm_navpvt_to_odom.cpp)
target_link_libraries(utm_navpvt_to_odom ${catkin_LIBRARIES})
add_dependencies(utm_navpvt_to_odom ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  #src/node.py
  scripts/wheel_odom.py
  scripts/imu_covariance_republisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


