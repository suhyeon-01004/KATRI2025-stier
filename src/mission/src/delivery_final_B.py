#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32, Float32
from mission.msg import PathInfo
from cv_bridge import CvBridge
from object_detector.msg import ObjectInfo
from erp42_msgs.msg import SerialFeedBack

from sensor_msgs.msg import CompressedImage # 시뮬용 09.08 추가

from paddleocr import PaddleOCR
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
from mission_utils import CUT_ROAD_RATIO, IMAGE_HEIGHT, WARP_SIZE, XM_PER_PIXEL

import logging
logging.disable(logging.DEBUG)
logging.disable(logging.WARNING)

LANE_TIME = 5
STOP_TIME = 5
STOP_DISTANCE = 1.2
ROAD_WIDTH = 4.0  # kcity
# ROAD_WIDTH = 3.5  # fmtc

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



def calculate_sign_point(frame, previous_sign_point):
    global obstacles, target_detected
    obstacles_temp = np.array(copy.deepcopy(obstacles), dtype=np.float32)

    sign_point = None
    display_frame = frame.copy()
    projected_points = None

    if obstacles_temp.size > 0:
        img_points, _ = cv2.projectPoints(
            obstacles_temp, rvec, tvec, cameraMatrix,
            np.array([0, 0, 0, 0], dtype=float))
        projected_points = img_points.reshape(-1, 2)

        for idx, pt in enumerate(projected_points):
            x, y = int(pt[0]), int(pt[1])
            if 0 <= x < display_frame.shape[1] and 0 <= y < display_frame.shape[0]:
                cv2.circle(display_frame, (x, y), 4, (0, 0, 255), -1)
                cv2.putText(display_frame, str(idx), (x + 5, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

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

                if score > 0.98 and (text=='A1'):
                    pts = np.array(box, dtype=np.int32)
                    cv2.polylines(display_frame, [pts], True, (0, 255, 0), 2)
                    cv2.putText(display_frame, text, (int(start[0]), int(start[1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
                    image_point = ((start[0]+end[0])/2, (start[1]+end[1])/2)
                    print("text : ", text)

                    # if not target_detected:
                    #     target_detected = True
                    #     target_sign = 'B' + text[1]
                    #     f = open('{}/catkin_ws/src/mission/msg/delivery_target_sign.txt'.format(Path.home()), 'w')
                    #     f.write(target_sign)
                    #     f.close()


        if image_point is not None and projected_points is not None:
            distance_list = [util.calculate_distance(image_point, pt) for pt in projected_points]
            distance_min_idx = np.argmin(distance_list)
            print("min dist : ", distance_list[distance_min_idx]) 
            if distance_list[distance_min_idx] < 300:
                sign_point = obstacles_temp[distance_min_idx][:2]
            
            if sign_point is not None:
                if sign_point[0] < 3:
                    print("less than 3")
                    sign_point = None

    else:
        if obstacles_temp.size > 0:
            obstacles_xy = obstacles_temp[:, :2]
            distance_list = [util.calculate_distance(previous_sign_point, pt) for pt in obstacles_xy]
            min_idx = np.argmin(distance_list)
            if min(distance_list) < 1:
                sign_point = obstacles_xy[min_idx]
            else:
                sign_point = previous_sign_point
        else:
            sign_point = previous_sign_point

    cv2.imshow('OCR Test Detection', display_frame)
    cv2.waitKey(1)

    return sign_point
    


if __name__ == '__main__':
    rospy.init_node('OcrTest', anonymous=True)

    ocr = PaddleOCR(use_angle_cls=True, lang='en')

    # ROS Publisher
    obstacle_vis_pub = rospy.Publisher('/obstacle_vis_pub', Marker, queue_size=1)

    # ROS Subscriber
    rospy.Subscriber('/object_info', ObjectInfo, callback_obstacle)

    # # Camera and LIDAR parameters
    rvec = np.array([
        [-0.00901827, -0.99986437,  0.0137809 ],
        [ 0.03088284, -0.01405338, -0.99942421],
        [ 0.99948233, -0.00858748,  0.03100539]
    ])

    tvec = np.array([
        [-0.02671437],
        [ 0.19031678],
        [ 0.6092148 ]
    ])

    cameraMatrix = np.array([
    [916.509137,     0.,         660.08973047],
    [  0.,         917.33875912, 335.60822492],
    [  0.,           0.        ,   1.        ]
    ])




    src, dst, M = util.get_camera_bev_parameters()
    obstacles = list()
    target_detected = False
    delivery_finished = False
    sign_point = None
    delivery_finish_time = None
    section = None
    target_sign = None
    prev_time = None
    steering_angle = 0
    ld_value = 9
    bridge = CvBridge()


    start_time = time.time()
    while not rospy.is_shutdown():
        image_msg = rospy.wait_for_message('/usb_cam1/image_raw', Image)
        frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        # image_msg = rospy.wait_for_message('/image_jpeg/compressed_6', CompressedImage) # 시뮬용 09.08 추가
        # np_arr = np.frombuffer(image_msg.data, np.uint8) # 시뮬용 09.08 추가
        # frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # 시뮬용 09.08 추가

        sign_point = calculate_sign_point(frame, sign_point)
        if sign_point is not None and sign_point[0] <= STOP_DISTANCE:
            print('stop!!')
            delivery_finished = True
            stop_start_time = time.time()
            while True:
                if time.time() - stop_start_time > STOP_TIME:
                    break
                else:
                    print("stopping")
                    time.sleep(0.1)
                    continue
            delivery_finish_time = time.time()
            continue

        else:
            # if sign_point is None:
            #     vel = 5
            # else:
            #     vel = 3
            vel = 5
            print('sign_point :', sign_point)
            ld_value = 7



        # Sign Visualization
        if sign_point is not None:
            ob = util.visualization_marker(sign_point, (255,0,0), 0.2, 0.2, 0.2)
            obstacle_vis_pub.publish(ob)
