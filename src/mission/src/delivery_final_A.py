
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32, Float32
from mission.msg import PathInfo
from cv_bridge import CvBridge
from object_detector.msg import ObjectInfo
from erp42_msgs.msg import SerialFeedBack

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
STOP_DISTANCE = 1.0
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

def callback_section(msg):
    global section
    section = msg.data

def callback_deg(data):
    global steering_angle
    steering_angle = data.data

# 특정 시간동안 차선보고 주행
def lane_control():
    start_time = time.time()
    while not rospy.is_shutdown():
        time_elapsed = time.time() - start_time
        if time_elapsed > LANE_TIME:
            break
        else:
            time.sleep(0.1)
            continue


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

# 기능 : 기준 차선의 이전 x절편 값을 받아 현재 x절편 리스트와 비교해서 기준 차선 추적
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

def calculate_sign_point(frame, previous_sign_point):
    global obstacles, target_detected
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

                if score > 0.98 and (text=='A1' or text=='A2' or text=='A3'):
                    image_point = ((start[0]+end[0])/2, (start[1]+end[1])/2)

                    if not target_detected:
                        target_detected = True
                        target_sign = 'B' + text[1]
                        f = open('{}/catkin_ws/src/mission/msg/delivery_target_sign.txt'.format(Path.home()), 'w')
                        f.write(target_sign)
                        f.close()


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
    rospy.init_node('DeliveryA', anonymous=True)

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
    avoid_finish_pub = rospy.Publisher('/avoid_finish', Int32, queue_size=1)
    waypoint_vis_pub = rospy.Publisher("/waypoint_vis_pub", MarkerArray, queue_size=1)
    criteria_lane_vis_pub = rospy.Publisher("/criteria_lane_vis_pub", MarkerArray, queue_size=1)
    obstacle_vis_pub = rospy.Publisher('/obstacle_vis_pub', Marker, queue_size=1)

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

    if args.section:
        while True:
            if section == 20:
                break
            else:
                print('not delivery A section')
                time.sleep(0.1)

    while True:  # auto 키면 시작
        feedback = rospy.wait_for_message('/erp42_serial/feedback', SerialFeedBack)
        if feedback.MorA == 1:
            break
        else:
            time.sleep(0.1)

    print('started!!')
    mission_lane_control_pub.publish(1)
    lane_control()
    mission_lane_control_pub.publish(0)
    print('lane finish!')

    start_time = time.time()
    criteria_lane_x_intercept = calculate_criteria_lane_first()
    while not rospy.is_shutdown():
        image_msg = rospy.wait_for_message('/usb_cam1/image_raw', Image)
        frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        lane_data = rospy.wait_for_message('lane_data_publisher', String)
        lanes_xys = json.loads(lane_data.data)
        lanes_xys = [[(x, y - int(CUT_ROAD_RATIO * IMAGE_HEIGHT)) for x, y in lane] for lane in lanes_xys]

        # Calculate Lane
        lane_coefficients = list()
        x_intercept_list = list()
        for lane_points in lanes_xys:
            points = np.array(lane_points, dtype=np.float32)
            points = points.reshape(-1, 1, 2)
            transformed_points = cv2.perspectiveTransform(points, M).reshape(-1, 2)
            transformed_points[:, 1] = -(transformed_points[:, 1] - WARP_SIZE)

            x_list = transformed_points[:, 0]
            y_list = transformed_points[:, 1]
            coefficients = np.polyfit(y_list, x_list, 1)
            
            lane_coefficients.append(coefficients)
            x_intercept_list.append(coefficients[-1])

        ret_val = calculate_criteria_lane(criteria_lane_x_intercept, x_intercept_list)
        if ret_val is not None:
            criteria_lane_idx, criteria_lane_x_intercept = ret_val
            ploty = np.linspace(0, WARP_SIZE-1, WARP_SIZE)
            criteria_lane_fit = lane_coefficients[criteria_lane_idx]
            criteria_plotx = np.polyval(criteria_lane_fit, ploty)
            criteria_lane_lidar_points = util.convert_point_bevcam2lidar(criteria_plotx, ploty)


        if delivery_finished:
            if time.time() - delivery_finish_time > 5:
                avoid_finish_pub.publish(1)
                break
            else:
                waypoint_temp = util.make_virtual_lane('right', criteria_lane_lidar_points, ROAD_WIDTH/2)
                waypoint_fit = util.fit_polynomial(waypoint_temp, 3)

                x_list = np.linspace(0, 6, 20)
                y_list = np.polyval(waypoint_fit, x_list)
                waypoint = list(zip(x_list, y_list))

                # Criteria Lane Visualiation
                lane_vis_points = [pt for i, pt in enumerate(criteria_lane_lidar_points) if i%10 == 0]
                ob = util.visualization_marker_array(lane_vis_points, (255,255,255), 0.1, 0.1, 0.1)
                criteria_lane_vis_pub.publish(ob)

                # Waypoint Visualization
                ob = util.visualization_marker_array(waypoint, (0,255,0), 0.1, 0.1, 0.1)
                waypoint_vis_pub.publish(ob)

                publish_path(waypoint, path_pub)
                vel_pub.publish(25)
                deg_pub.publish(steering_angle)
                ld_pub.publish(7)

                continue

        sign_point = calculate_sign_point(frame, sign_point)
        if sign_point is not None and sign_point[0] <= STOP_DISTANCE:
            print('stop!!')
            delivery_finished = True
            stop_start_time = time.time()
            while True:
                if time.time() - stop_start_time > STOP_TIME:
                    break
                else:
                    vel_pub.publish(0)
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

        waypoint_temp = util.make_virtual_lane('right', criteria_lane_lidar_points, ROAD_WIDTH-0.1)
        waypoint_fit = util.fit_polynomial(waypoint_temp, 3)
        x_list = np.linspace(0, 8, 20)
        y_list = np.polyval(waypoint_fit, x_list)
        waypoint = list(zip(x_list, y_list))

        if time.time() - start_time <= 2:
            waypoint_temp = util.make_virtual_lane('right', criteria_lane_lidar_points, ROAD_WIDTH/2)
            waypoint_fit = util.fit_polynomial(waypoint_temp, 3)

            x_list = np.linspace(0, 6, 20)
            y_list = np.polyval(waypoint_fit, x_list)
            waypoint = list(zip(x_list, y_list))

            vel = 5

        ### Visualization
        # Criteria Lane Visualiation
        lane_vis_points = [pt for i, pt in enumerate(criteria_lane_lidar_points) if i%10 == 0]
        ob = util.visualization_marker_array(lane_vis_points, (255,255,255), 0.1, 0.1, 0.1)
        criteria_lane_vis_pub.publish(ob)

        # Waypoint Visualization
        ob = util.visualization_marker_array(waypoint, (0,255,0), 0.1, 0.1, 0.1)
        waypoint_vis_pub.publish(ob)

        # Sign Visualization
        if sign_point is not None:
            ob = util.visualization_marker(sign_point, (255,0,0), 0.2, 0.2, 0.2)
            obstacle_vis_pub.publish(ob)

        ### Publish
        publish_path(waypoint, path_pub)
        vel_pub.publish(vel)
        deg_pub.publish(steering_angle)
        ld_pub.publish(ld_value)
        #print('vel : {}, ld : {}, steering_angle : {}'.format(vel, ld_value, steering_angle))

        #frame = cv2.resize(frame, (640,360))
        # cv2.imshow('frame', frame)
        # if cv2.waitKey(25) & 0xFF == ord('q'):
        #     cv2.destroyAllWindows()
        #     break
