#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Int32, Float32, Bool
from mission.msg import PathInfo
from object_detector.msg import ObjectInfo

from paddleocr import PaddleOCR
from pathlib import Path
import numpy as np
import argparse
import copy
import time
import json
import cv2

import sys
sys.path.append('{}/catkin_ws/src/mission/src'.format(Path.home()))
import mission_utils as util
from mission_utils import CUT_ROAD_RATIO, IMAGE_HEIGHT, WARP_SIZE, XM_PER_PIXEL

import logging
logging.disable(logging.DEBUG)
logging.disable(logging.WARNING)

ROAD_WIDTH = 3.5
STOP_DISTANCE = 2.8
STOP_TIME = 5
BASE_SPEED = 5
APPROACH_DISTANCE = 3.4

def callback_obstacle(msg):
    global obstacles

    obstacle_list = list()

    cx = msg.centerX
    cy = msg.centerY
    cz = msg.centerZ

    for i in range(msg.objectCounts):
        center_x = cx[i]
        center_y = cy[i]
        center_z = cz[i]

        obstacle_list.append([center_x, center_y, center_z])

    obstacles = obstacle_list

def callback_section(msg):
    global section
    section = msg.data

def callback_deg(data):
    global steering_angle
    steering_angle = data.data


# 왼쪽 차선을 기준 차선으로
def calculate_criteria_lane_first():
    lane_data = rospy.wait_for_message('lane_data_publisher', String)
    lane_points = json.loads(lane_data.data)
    lanes_xys = [[(x, y - int(CUT_ROAD_RATIO * IMAGE_HEIGHT)) for x, y in sublist] for sublist in lane_points]

    lane_coefficients = list()
    x_intercept_list = list()
    for lane_points in lanes_xys:
        points = np.array(lane_points, dtype=np.float32)
        points = points.reshape(-1, 1, 2)
        transformed_points = cv2.perspectiveTransform(points, M).reshape(-1, 2)
        transformed_points[:, 1] = -(transformed_points[:, 1] - WARP_SIZE)

        x_list = transformed_points[:, 0]
        y_list = transformed_points[:, 1]
        coefficients = np.polyfit(y_list, x_list, 1)  # 왼쪽 차선이라 잘 안보이는거 감안해서 3차식이 아닌 1차식에 적합
        
        lane_coefficients.append(coefficients)
        x_intercept_list.append(coefficients[-1])

    left_lane_idx = util.get_left_lane_index(x_intercept_list)
    criteria_lane_x_intercept = x_intercept_list[left_lane_idx]

    return criteria_lane_x_intercept

def calculate_criteria_lane(before_criteria_x_intercept, x_intercept_list):
    if not x_intercept_list:  # 차선이 한개도 인식되지 않은 경우
        return None

    criteria_lane_idx = np.argmin([(before_criteria_x_intercept - x)**2 for x in x_intercept_list])
    criteria_lane_x_intercept = x_intercept_list[criteria_lane_idx]

    if abs(before_criteria_x_intercept - criteria_lane_x_intercept)*XM_PER_PIXEL > 1.0:  # 기준 차선이 순간적으로 인식되지 않은 경우
        return None
    else:
        return criteria_lane_idx, criteria_lane_x_intercept

def publish_path(waypoint, path_pub):
    x_points = [pt[0] for pt in waypoint]
    y_points = [pt[1] for pt in waypoint]

    path_msg = PathInfo()
    path_msg.cnt = 20
    path_msg.x = x_points
    path_msg.y = y_points

    path_pub.publish(path_msg)

def calculate_sign_point(frame, previous_sign_point, target_sign):
    global obstacles
    obstacles_temp = np.array(copy.deepcopy(obstacles))

    sign_point = None
    if previous_sign_point is None:

        image_point = None
        result = ocr.ocr(frame, cls=True)[0]
        if result is not None:
            for line in result:
                box = line[0]  # 텍스트의 바운딩 박스 좌표
                text = line[1][0].strip()  # 인식된 텍스트
                score = line[1][1]  # 인식 신뢰도
                
                start = box[0]
                end = box[2]
                print(text)
                if score > 0.98 and (text==target_sign):
                    image_point = ((start[0]+end[0])/2, (start[1]+end[1])/2)

        if image_point is not None and obstacles_temp.size > 0:
            img_points, _ = cv2.projectPoints(
                obstacles_temp, rvec, tvec, cameraMatrix,
                np.array([0, 0, 0, 0], dtype=float))

            distance_list = [util.calculate_distance(image_point, pt[0]) for pt in img_points]
            distance_min_idx = np.argmin(distance_list)
            sign_point = obstacles_temp[distance_min_idx][:2]

            if sign_point[0] < 3:
                sign_point = None

    else:
        obstacles_temp = obstacles_temp[:, :2]
        distance_list = [util.calculate_distance(previous_sign_point, pt) for pt in obstacles_temp]
        min_idx = np.argmin(distance_list)
        if min(distance_list) < 1:
            sign_point = obstacles_temp[min_idx]
        else:
            sign_point = previous_sign_point

    return sign_point


if __name__ == '__main__':
    rospy.init_node('DeliveryB', anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('--section', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])

    ocr = PaddleOCR(use_angle_cls=True, lang='en')

    # ROS Publisher
    mission_lane_control_pub = rospy.Publisher('/mission_lane_control', Int32, queue_size=1)
    vel_pub = rospy.Publisher('/missionKPH', Float32, queue_size=1)
    deg_pub = rospy.Publisher("/missionDeg", Float32, queue_size=1)
    ld_pub = rospy.Publisher('/lidar_pp_ld', Int32, queue_size=1)
    path_pub = rospy.Publisher("/local_path", PathInfo, queue_size=1)
    obstacle_vis_pub = rospy.Publisher('/obstacle_vis_pub', MarkerArray, queue_size=1)
    deliveryB_stop_pub = rospy.Publisher('/deliveryB_stop', Bool, queue_size=1)

    # ROS Subscriber
    rospy.Subscriber('/object_info', ObjectInfo, callback_obstacle)
    rospy.Subscriber("/section", Int32, callback_section)
    rospy.Subscriber('/lidarDeg', Float32, callback_deg)

    # Camera and LIDAR parameters
    rvec = np.array([
        [-0.05578533, -0.99827818, -0.01812908],
        [0.14790644, 0.00969451, -0.98895384],
        [0.9874268, -0.05785053, 0.14711096]
    ])

    tvec = np.array([
        [0.01902152],
        [0.04891112],
        [-0.01215745]
    ])

    cameraMatrix = np.array([
        [759.63868072, 0., 633.25037],
        [0., 763.35336903, 367.11064456],
        [0., 0., 1.]
    ])

    src, dst, M = util.get_camera_bev_parameters()
    obstacles = list()
    sign_point = None
    delivery_finished = False
    approach_activate = False
    back_car_finish = False
    delivery_finish_time = None
    section = None
    steering_angle = 0
    bridge = CvBridge()

    if args.section:
        while True:
            if section == 21:
                break
            else:
                print('not delivery B section')
                time.sleep(0.1)


    # Get Target Sign
    f = open('{}/catkin_ws/src/mission/msg/delivery_target_sign.txt'.format(Path.home()), 'r')
    target_sign = f.read().strip()
    print('target_sign :', target_sign)


    print('started!!')
    while not rospy.is_shutdown():
        image_msg = rospy.wait_for_message('/usb_cam1/image_raw', Image)
        frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        cv2.imshow('OCR Test Detection', frame.copy())
        cv2.waitKey(1)

        #print('started!!')
        if delivery_finished:
            print("delivery finished!")
            continue

        if target_sign is not None:
            sign_point = calculate_sign_point(frame, sign_point, target_sign)
            print(sign_point)
            if sign_point is not None and sign_point[0] <= STOP_DISTANCE:
                print('stop!!')
                delivery_finished = True
                stop_start_time = time.time()
                while True:
                    if time.time() - stop_start_time <= STOP_TIME:
                        deliveryB_stop_pub.publish(True)
                    else:
                        deliveryB_stop_pub.publish(False)
                        break

        # Sign Visualization
        if sign_point is not None:
            ob = util.visualization_marker_array([sign_point], (255,0,0), 0.2, 0.2, 0.2)
            obstacle_vis_pub.publish(ob)
