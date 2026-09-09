#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Tunnel Section 9

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from object_detector.msg import ObjectInfo

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

AVOID_START_DISTANCE = 15

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

# 카메라로 라바콘 + PE드럼통 인식한 뒤 BEV 변환하여 위치 추정 후, 현재 차선 내에 있는 좌표만 반환
def estimate_cone_coord(left_lane_lidar_points, right_lane_lidar_points, frame):

    result = cone_model.predict(frame, verbose=False)[0]
    boxes = result.boxes.xyxy
    x_list = list()
    y_list = list()

    for box in boxes:
        box = [int(pt) for pt in box]
        start = (box[0], box[1])
        end = (box[2], box[3])

        pixel_coord = [[int((start[0]+end[0])/2), int(end[1]-CUT_ROAD_RATIO*IMAGE_HEIGHT)]]
        points = np.array(pixel_coord, dtype=np.float32)
        points = points.reshape(-1, 1, 2)
        transformed_points = cv2.perspectiveTransform(points, M).reshape(-1, 2)[0]
        transformed_points[1] = -(transformed_points[1] - WARP_SIZE)
        x_list.append(transformed_points[0])
        y_list.append(transformed_points[1])

    cone_lidar_coord_list = util.convert_point_bevcam2lidar(x_list, y_list)
    current_obstacles = util.get_current_obstacles(left_lane_lidar_points, right_lane_lidar_points, cone_lidar_coord_list)

    return current_obstacles

def calculate_dummy_point(left_lane_lidar_points, right_lane_lidar_points, estimated_coords):
    global obstacles
    obstacles_temp = copy.deepcopy(obstacles)
    current_obstacles = util.get_current_obstacles(left_lane_lidar_points, right_lane_lidar_points, obstacles_temp)

    dummy_point = None
    if len(estimated_coords) > 0:
        for obstacle in current_obstacles:
            distance_list = [util.calculate_distance(obstacle, pt) for pt in estimated_coords]
            if min(distance_list) > 3:
                dummy_point = obstacle
                break

    else:
        if len(current_obstacles) > 0:
            distance_list = [util.calculate_distance((0,0), pt) for pt in current_obstacles]
            min_idx = np.argmin(distance_list)
            dummy_point = current_obstacles[min_idx]
        

    return dummy_point



if __name__ == '__main__':
    rospy.init_node('tunnel', anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('--section', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])

    cone_model = YOLO('{}/catkin_ws/ModelFiles/RubberCone_YOLOv10m_1280.pt'.format(Path.home()))
    roadmark_model = YOLO('{}/catkin_ws/ModelFiles/StopLine.pt'.format(Path.home()))
    cone_model.predict(np.zeros((720, 1280,3)), verbose=False)
    roadmark_model.predict(np.zeros((720, 1280,3)), verbose=False)

    # ROS Publisher
    vel_pub = rospy.Publisher('/missionKPH', Float32, queue_size=1)
    deg_pub = rospy.Publisher("/missionDeg", Float32, queue_size=1)
    mission_lane_control_pub = rospy.Publisher('/mission_lane_control', Int32, queue_size=1)
    tunnel_avoid_start_pub = rospy.Publisher('/tunnel_avoid_start', Int32, queue_size=1)

    # ROS Subscriber
    rospy.Subscriber('/object_info', ObjectInfo, callback_obstacle)
    rospy.Subscriber("/section", Int32, callback_section)

    src, dst, M = util.get_camera_bev_parameters()
    left_lane_lidar_points = None
    right_lane_lidar_points = None
    avoid_small_finished = False
    rotary_closed = False
    obstacles = list()
    section = None
    dummy_passed = False
    bridge = CvBridge()

    if args.section:
        while True:
            if section == 9:
                break
            else:
                print('not tunnel section')
                time.sleep(0.1)

    mission_lane_control_pub.publish(1)
    while not rospy.is_shutdown():
        image_msg = rospy.wait_for_message('/usb_cam1/image_raw', Image)
        frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        lane_data = rospy.wait_for_message('lane_data_publisher', String)
        lanes_xys = json.loads(lane_data.data)
        lanes_xys = [[(x, y - int(CUT_ROAD_RATIO * IMAGE_HEIGHT)) for x, y in lane] for lane in lanes_xys]

        ret_val = util.get_current_lane_lidar_points(lanes_xys, M)
        if ret_val is None:
            if left_lane_lidar_points is None or right_lane_lidar_points is None:
                continue
        else:
            left_lane_lidar_points, right_lane_lidar_points = ret_val
        left_lane_lidar_fit = util.fit_polynomial(left_lane_lidar_points)
        right_lane_lidar_Fit = util.fit_polynomial(right_lane_lidar_points)

        if not avoid_small_finished or not dummy_passed:
            estimated_coords = estimate_cone_coord(left_lane_lidar_points, right_lane_lidar_points, frame)

        if not dummy_passed:
            dummy_point = calculate_dummy_point(left_lane_lidar_points, right_lane_lidar_points, estimated_coords)
            if dummy_point is not None and dummy_point[0] <= 12:
                mission_lane_control_pub.publish(0)
                print('dummy stop!!')
                while not rospy.is_shutdown():
                    dummy_point = calculate_dummy_point(left_lane_lidar_points, right_lane_lidar_points, estimated_coords)
                    if dummy_point is None:
                        dummy_passed = True
                        mission_lane_control_pub.publish(1)
                        break

                    if dummy_point[0] >= 6:
                        vel_pub.publish(4)
                        deg_pub.publish(0)
                        print('slow down')
                        time.sleep(0.1)
                    else:
                        vel_pub.publish(0)
                        deg_pub.publish(0)
                        print('stop')
                        time.sleep(0.1)
                    

        if not avoid_small_finished:
            if len(estimated_coords) > 0:
                distance_list = [util.calculate_distance((0,0), pt) for pt in estimated_coords]
                distance_min = min(distance_list)
                if distance_min <= AVOID_START_DISTANCE:
                    print('avoid small started')
                    tunnel_avoid_start_pub.publish(1)
                    mission_lane_control_pub.publish(0)
                    rospy.wait_for_message('/tunnel_avoid_finish', Int32)
                    print('avoid small finished')
                    mission_lane_control_pub.publish(1)
                    avoid_small_finished = True
                    continue

        # if avoid_small_finished:
        #     result = roadmark_model.predict(frame, verbose=False)[0]
        #     classes = [int(pt) for pt in result.boxes.cls]
        #     confs = [int(conf * 100) for conf in result.boxes.conf]

        #     for i in range(len(classes)):
        #         if classes[i] == 3 and confs[i] >= 70:
        #             rotary_closed = True
                    
        # print('rotary_closed :', rotary_closed)
        # if rotary_closed:
        #     mission_lane_control_pub.publish(2)