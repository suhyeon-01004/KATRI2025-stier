#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Tunnel Section 9

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32, Float32
from mission.msg import PathInfo
from object_detector.msg import ObjectInfo

from scipy.interpolate import CubicSpline
from ultralytics import YOLO
from pathlib import Path
import numpy as np
import argparse
import time
import json
import copy
import cv2

import sys
sys.path.append('{}/catkin_ws/src/mission/src'.format(Path.home()))
import mission_utils as util
from mission_utils import CUT_ROAD_RATIO, IMAGE_HEIGHT, WARP_SIZE


def callback_section(msg):
    global section
    section = msg.data

def callback_obstacle(msg):
    global obstacles

    obstacle_list = []

    cx = msg.centerX
    cy = msg.centerY

    for i in range(msg.objectCounts):
        center_x = cx[i]
        center_y = cy[i]

        obstacle_list.append((center_x, center_y))

    obstacles = obstacle_list

def callback_deg(data):
    global steering_angle, section

    if section == 10:
        steering_angle = data.data
        print('steering_angle :', steering_angle)


if __name__ == '__main__':
    rospy.init_node('tollgate', anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('--section', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])

    # ROS Publisher
    path_pub = rospy.Publisher("/local_path", PathInfo, queue_size=1)    
    vel_pub = rospy.Publisher('/missionKPH', Float32, queue_size=1)
    deg_pub = rospy.Publisher("/missionDeg", Float32, queue_size=1)
    ld_pub = rospy.Publisher('/lidar_pp_ld', Int32, queue_size=1) 
    waypoint_vis_pub = rospy.Publisher("/waypoint_vis_pub", MarkerArray, queue_size=1)
    mission_finish_pub = rospy.Publisher('/avoid_finish', Int32, queue_size=1)

    # ROS Subscriber
    rospy.Subscriber('/object_info', ObjectInfo, callback_obstacle)
    rospy.Subscriber('/lidarDeg', Float32, callback_deg)
    rospy.Subscriber("/section", Int32, callback_section)

    obstacles = list()
    started = True
    section = None
    src, dst, M = util.get_camera_bev_parameters()
    ploty = np.linspace(0, 5, 20)
    steering_angle = 0

    if args.section:
        while True:
            if section == 10:
                break
            else:
                print('not tollgate section')
                time.sleep(0.1)

    print('started!!')
    while not rospy.is_shutdown():

        lane_data = rospy.wait_for_message('lane_data_publisher', String)
        lanes_xys = json.loads(lane_data.data)
        lanes_xys = [[(x, y - int(CUT_ROAD_RATIO * IMAGE_HEIGHT)) for x, y in lane] for lane in lanes_xys]

        slope_list = list()
        for lane_points in lanes_xys:
            points = np.array(lane_points, dtype=np.float32)
            points = points.reshape(-1, 1, 2)
            transformed_points = cv2.perspectiveTransform(points, M).reshape(-1, 2)
            transformed_points[:, 1] = -(transformed_points[:, 1] - WARP_SIZE)

            x_list = transformed_points[:, 0]
            y_list = transformed_points[:, 1]
            coefficients = np.polyfit(y_list, x_list, 1)

            plotx = np.polyval(coefficients, ploty)
            lidar_points = util.convert_point_bevcam2lidar(plotx, ploty)
            lidar_fit = util.fit_polynomial(lidar_points, 1)
            slope_list.append(lidar_fit[0])

        if len(slope_list) > 0:  # slope 정보가 있을 때만 평균 계산, 없는 경우 이전에 계산했던 값 사용
            slope_avg = sum(slope_list) / len(slope_list)          
        if slope_avg is None:
            continue

        linear_fit = [slope_avg, 0]

        obstacles_temp = copy.deepcopy(obstacles)
        left_obstacles = list()
        right_obstacles = list()

        for ob in obstacles_temp:
            if ob[1] >= np.polyval(linear_fit, ob[0]):
                left_obstacles.append(ob)
            else:
                right_obstacles.append(ob)

        if len(left_obstacles)==0 or len(right_obstacles)==0:
            print('finished!!')
            mission_finish_pub.publish(1)
            break

        left_fit = util.calculate_line_parameters(slope_avg, left_obstacles[0])
        right_fit = util.calculate_line_parameters(slope_avg, right_obstacles[0])
        linear_fit = [(left_fit[0]+right_fit[0])/2, (left_fit[1]+right_fit[1])/2]

        x_points = np.linspace(0, 8, 20)
        y_points = np.polyval(linear_fit, x_points)

        waypoint = list(zip(x_points, y_points))

        ob = util.visualization_marker_array(waypoint, (0,255,0), 0.1, 0.1, 0.1)
        waypoint_vis_pub.publish(ob)

        path_msg = PathInfo()
        path_msg.cnt = 20
        path_msg.x = x_points
        path_msg.y = y_points

        path_pub.publish(path_msg)
        ld_pub.publish(8)
        vel_pub.publish(10)
        deg_pub.publish(steering_angle)

        
        