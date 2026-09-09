#!/usr/bin/env bash

# 1) RPLIDAR 뷰어
xterm -hold -geometry 80x24+0+0    -e bash -i -c "roslaunch rplidar_ros rplidar_s2.launch; exec bash" &

# 2) Point-cloud converter
xterm -hold -geometry 80x24+500+0  -e bash -i -c "rosrun object_detector cloud_converter.py; exec bash" &

# 3) Obstacle detector
xterm -hold -geometry 80x24+1000+0 -e bash -i -c "rosrun object_detector obstacle_detector.py; exec bash" &

# 4) GPS bringup
xterm -hold -geometry 80x24+1500+0 -e bash -i -c "roslaunch gps_bringup gps.launch; exec bash" &

# 5) UBlox 상태 확인
xterm -hold -geometry 80x24+0+380   -e bash -i -c "rosrun rostopic rostopic echo /ublox_position_receiver/navstatus; exec bash" &

# 6) Arduino serial
xterm -hold -geometry 80x24+500+380 -e bash -i -c "rosrun rosserial_python serial_node.py /dev/dev_arduino; exec bash" &

# 7) Pure pursuit LiDAR
xterm -hold -geometry 80x24+1000+380 -e bash -i -c "rosrun lidar pure_pursuit_lidar.py; exec bash" &

# 8) Pure pursuit tunnel
xterm -hold -geometry 80x24+1500+380 -e bash -i -c "rosrun lidar pure_pursuit_tunnel.py; exec bash" &

# 9) 전체 Pure Pursuit 런치
xterm -hold -geometry 80x24+0+800   -e bash -i -c "roslaunch stier pp.launch; exec bash" &

# 10) GPS-RRT mission
xterm -hold -geometry 80x24+500+800 -e bash -i -c "rosrun mission gps_rrt.py; exec bash" &

# 11) Tunnel vision processing
xterm -hold -geometry 80x24+1000+800 -e bash -i -c "rosrun vision_pkg tunnel.py; exec bash" &

# 12) Vision 전체 런치
xterm -hold -geometry 80x24+1500+800 -e bash -i -c "roslaunch vision_pkg vision.launch; exec bash" &

# 백그라운드 프로세스 대기
wait

