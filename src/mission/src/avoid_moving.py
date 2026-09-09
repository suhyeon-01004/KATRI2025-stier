#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from object_detector.msg import ObjectInfo
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String, Int32

import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge

from scipy.interpolate import CubicSpline
from ultralytics import YOLO
from pathlib import Path
import numpy as np
import argparse
import json
import copy
import cv2
import time

from mission.msg import PathInfo

import warnings
warnings.simplefilter('ignore', np.RankWarning)

import sys
sys.path.append('{}/catkin_ws/src/mission/src'.format(Path.home()))
from mission_utils import *

STATE_CLEAR = 0
STATE_SLOW_FAR = 1
STATE_SLOW_NEAR = 2
STATE_STOP = 3
STATE_RESUME = 4

RESUME_DELAY_SEC = 5.0
section = None

# 장애물 정보 콜백 함수
def callback_obstacle(msg):
    global obstacles

    obstacle_list = list()

    cx = msg.centerX
    cy = msg.centerY

    for i in range(msg.objectCounts):
        if cx[i] >= 0.3:
            center_x = cx[i]
            center_y = cy[i]

            obstacle_list.append([center_x, center_y])

    obstacles = obstacle_list

def callback_section(msg):
    global section
    section = msg.data

# 왼쪽 상단이 원점인 BEV 이미지를 라이다 좌표계로 변환
def convert_point_bevcam2lidar(pixel_coord):
    YM_PER_PIXEL = 12 / 720 
    x = pixel_coord[0]
    y = pixel_coord[1]

    y = -(y - 720)
    x = -(x - 360)
    x = x * XM_PER_PIXEL
    y = y * YM_PER_PIXEL
    x, y = y, x
    x += CAR_OFFSET

    return [x, y]




if __name__ == '__main__':
    rospy.init_node('avoid_moving', anonymous=True)

    human_model = YOLO('{}/catkin_ws/ModelFiles/CarHumanYOLOv9c_1280.pt'.format(Path.home()))
    src, dst, M = get_camera_bev_parameters()

    parser = argparse.ArgumentParser()
    parser.add_argument('--section', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])

    left_lidar_points = None
    right_lidar_points = None
    obstacles = list()
    state_tracker = {
        "current": STATE_CLEAR,
        "stop_stamp": None,
        "resume_sent": False
    }

    def publish_state(new_state):
        if new_state == state_tracker["current"]:
            # 상태가 STOP이면 최초 진입 시각이 없을 수도 있으니 보정
            if new_state == STATE_STOP and state_tracker["stop_stamp"] is None:
                state_tracker["stop_stamp"] = rospy.Time.now()
            return
        stop_sign_pub.publish(new_state)
        state_tracker["current"] = new_state
        if new_state == STATE_STOP:
            state_tracker["stop_stamp"] = rospy.Time.now()
            state_tracker["resume_sent"] = False
        elif new_state == STATE_RESUME:
            state_tracker["stop_stamp"] = None
            state_tracker["resume_sent"] = True
        else:
            state_tracker["stop_stamp"] = None
            state_tracker["resume_sent"] = False

    # ROS Publisher
    # obstacle_vis_pub = rospy.Publisher('/obstacle_vis_pub', Marker, queue_size=1) # 1018 시각화를 위한 수정
    obstacle_vis_pub = rospy.Publisher('/obstacle_vis_pub', MarkerArray, queue_size=1)
    lane_vis_pub = rospy.Publisher("/lane_vis_pub", MarkerArray, queue_size=1)
    stop_sign_pub = rospy.Publisher('/stop_sign', Int32, queue_size=1)  # 상태 퍼블리셔
    human_flag_pub = rospy.Publisher('/dynamic_human_detected', Int32, queue_size=1)

    publish_state(STATE_CLEAR)

    # ROS Subscriber
    rospy.Subscriber('/object_info', ObjectInfo, callback_obstacle)
    rospy.Subscriber('/section', Int32, callback_section)

    bridge = CvBridge()

    if args.section:
        while True:
            if section == 9:
                break
            else:
                time.sleep(0.1)

    while not rospy.is_shutdown():
        #stop_sign_pub.publish(0)
        image_msg = rospy.wait_for_message("/usb_cam2/image_raw", Image) # 시뮬용 09.08 주석
        frame = bridge.imgmsg_to_cv2(image_msg, "bgr8") # 시뮬용 09.08 주석

        lane_data = rospy.wait_for_message('lane_data_publisher', String)
        lanes_xys = json.loads(lane_data.data)
        lanes_xys = [[(x, y - int(CUT_ROAD_RATIO * IMAGE_HEIGHT)) for x, y in lane] for lane in lanes_xys]

        lane_point_pair = get_current_lane_lidar_points(lanes_xys, M)
        if lane_point_pair is None:
            rospy.logwarn_throttle(5.0, "[avoid_moving] Lane detection failed; skipping frame.")
            continue

        left_lidar_points, right_lidar_points = lane_point_pair
        left_lidar_fit = fit_polynomial(left_lidar_points)
        right_lidar_fit = fit_polynomial(right_lidar_points)

        lane_vis_points = [pt for i, pt in enumerate(left_lidar_points+right_lidar_points) if i%10==0]
        ob = visualization_marker_array(lane_vis_points, (255,255,255), 0.1, 0.1, 0.1)
        lane_vis_pub.publish(ob)

        result = human_model.predict(frame, verbose=False)[0]

        boxes = [[int(pt) for pt in box] for box in result.boxes.xyxy]
        classes = [int(pt) for pt in result.boxes.cls]
        confs = [int(conf*100) for conf in result.boxes.conf]

        est_coord_list = list()       
        for i, box in enumerate(boxes):
            if classes[i] == 6 and confs[i]>50:
                box = [int(pt) for pt in box]
                start = (box[0], box[1])
                end = (box[2], box[3])

                color = (255, 0, 0)
                cv2.rectangle(frame, start, end, color=color, thickness=2)
                cv2.putText(frame, 'Human', (box[0], box[3] + 15), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)
                cv2.putText(frame, '({}%)'.format(confs[i]), (box[0], box[3] + 40), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)

                pixel_coord = [[int((start[0]+end[0])/2), int(end[1]-0.4*IMAGE_HEIGHT)]]
                points = np.array(pixel_coord, dtype=np.float32)
                points = points.reshape(-1, 1, 2)
                transformed_points = cv2.perspectiveTransform(points, M).reshape(-1, 2)[0]
                lidar_coord = convert_point_bevcam2lidar(transformed_points)
                est_coord_list.append(lidar_coord)
                cv2.putText(frame, '({:.2f}, {:.2f})'.format(lidar_coord[0], lidar_coord[1]), (box[0], box[1] - 40), cv2.FONT_HERSHEY_DUPLEX, 0.6, color)
                
        ob = visualization_marker_array(est_coord_list, (0,0,255), 0.2, 0.2, 0.2, 'cube', 0.1) # 1018 시각화를 위한 수정
        obstacle_vis_pub.publish(ob) # 1018 시각화를 위한 수정

        target_ob = None
        target_distance = None
        human_detected = 0
        new_state = STATE_CLEAR

        current_obstacles = copy.deepcopy(obstacles)
        if current_obstacles:
            current_obstacles = np.asarray(current_obstacles)
            for est_coord in est_coord_list:
                est_coord = np.array(est_coord)
                if current_obstacles.ndim == 1:
                    current_obstacles = current_obstacles.reshape(1, -1)
                distance_list = np.linalg.norm(current_obstacles - est_coord, axis=1)
                distance_min_idx = np.argmin(distance_list)
                if distance_list[distance_min_idx] <= 6:
                    candidate = current_obstacles[distance_min_idx]
                    candidate_dist = calculate_distance((0,0), candidate)
                    if target_distance is None or candidate_dist < target_distance:
                        target_ob = candidate
                        target_distance = candidate_dist

        if target_ob is not None and target_distance is not None:
            ob_x, ob_y = target_ob
            left_lane_ob_y = np.polyval(left_lidar_fit, ob_x)
            right_lane_ob_y = np.polyval(right_lidar_fit, ob_x)

            if right_lane_ob_y <= ob_y <= left_lane_ob_y:
                obstacle_vis_pub.publish(
                    visualization_marker_array([tuple(target_ob)], (0,0,255), 0.2, 0.2, 0.2, 'cube', 0.1)
                )

                if target_distance < 5:
                    new_state = STATE_STOP
                elif target_distance < 10:
                    new_state = STATE_SLOW_NEAR
                elif target_distance < 15:
                    new_state = STATE_SLOW_FAR
                else:
                    new_state = STATE_CLEAR

                human_detected = 1
            else:
                target_ob = None
                target_distance = None

        if target_ob is None:
            obstacle_vis_pub.publish(visualization_marker_array([], (0,0,255), 0.2, 0.2, 0.2, 'cube', 0.1))

        previous_state = state_tracker["current"]
        publish_state(new_state)

        if previous_state == STATE_STOP and new_state == STATE_CLEAR:
            publish_state(STATE_RESUME)

        if state_tracker["current"] == STATE_STOP:
            if state_tracker["stop_stamp"] is None:
                state_tracker["stop_stamp"] = rospy.Time.now()
            if not human_detected and not state_tracker["resume_sent"]:
                publish_state(STATE_RESUME)
            elif (not state_tracker["resume_sent"]) and state_tracker["stop_stamp"] is not None:
                elapsed = (rospy.Time.now() - state_tracker["stop_stamp"]).to_sec()
                if elapsed >= RESUME_DELAY_SEC:
                    publish_state(STATE_RESUME)

        frame = cv2.resize(frame, (640, 360))
        cv2.imshow('frame', frame)
        if cv2.waitKey(25) == ord('q'):
            break

        human_flag_pub.publish(human_detected)
