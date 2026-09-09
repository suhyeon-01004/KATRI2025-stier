#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Int32, Float32
from mission.msg import PathInfo
from object_detector.msg import ObjectInfo

from ultralytics import YOLO
from pathlib import Path
import numpy as np
import argparse
import copy
import time
import json
import math
import cv2

import sys
sys.path.append('{}/catkin_ws/src/mission/src'.format(Path.home()))
import mission_utils as util
from mission_utils import CUT_ROAD_RATIO, IMAGE_HEIGHT, WARP_SIZE, XM_PER_PIXEL

SIGN_IDX = {7:'A1', 8:'A2', 9:'A3', 10:'B1', 11:'B2', 12:'B3', 4:'UTURN', 6:'PARKING'}
#START_METER = 25
ROAD_WIDTH = 3.5
STOP_DISTANCE = 3.2
STOP_TIME = 5
BASE_SPEED = 5
APPROACH_DISTANCE = 3.5

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

# 속도 받아 거리 추정, 이동 거리가 START_METER보다 커지면 return
# def lane_control():
#     global current_speed

#     last_time = time.time()
#     moved_distance = 0
#     while not rospy.is_shutdown():
#         moved_distance += current_speed * (time.time() - last_time)
#         print('moved_distance :', moved_distance)
#         last_time = time.time()
#         time.sleep(0.1)
#         if moved_distance > START_METER:
#             break

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

        result = delivery_model.predict(frame, verbose=False)[0]

        boxes = [[int(pt) for pt in box] for box in result.boxes.xyxy]
        classes = [int(pt) for pt in result.boxes.cls]
        confs = [int(conf*100) for conf in result.boxes.conf]

        image_point = None
        for i, box in enumerate(boxes):
            if 10 <= classes[i] <= 12 and confs[i] >= 90:
                box = [int(pt) for pt in box]
                start = (box[0], box[1])
                end = (box[2], box[3])

                sign = SIGN_IDX[classes[i]]

                color = (255,0,0)
                cv2.rectangle(frame, start, end, color=color, thickness=2)
                cv2.putText(frame, sign, (box[0], box[3] + 15), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)
                cv2.putText(frame, '({}%)'.format(confs[i]), (box[0], box[3] + 40), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)
                
                if sign == target_sign:
                    image_point = ((start[0]+end[0])/2, (start[1]+end[1])/2)

        if image_point is not None and obstacles_temp.size > 0:
            img_points, _ = cv2.projectPoints(
                obstacles_temp, rvec, tvec, cameraMatrix,
                np.array([0, 0, 0, 0], dtype=float))
            
            ### YOLO 박스 내에 있는 빨간 점 중에서 center와 가장 가까운 좌표를 가져오는 코드 추가해야됨

            # for i in range(len(img_points)):
            #     try:
            #         x, y = int(img_points[i][0][0]), int(img_points[i][0][1])
            #         if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]:  # 좌표가 이미지 범위 내에 있는지 확인
            #             cv2.circle(frame, (x, y), 5, (0, 0, 255), 5)
            #     except Exception as e:
            #         rospy.logerr(f"Error drawing circle at img_points[{i}]: {img_points[i]}, Error: {e}")

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

    delivery_model = YOLO('{}/catkin_ws/ModelFiles/Delivery.pt'.format(Path.home()))  # 작년 데이터
    delivery_model.predict(np.zeros((720, 1280, 3)), verbose=False)

    # ROS Publisher
    mission_lane_control_pub = rospy.Publisher('/mission_lane_control', Int32, queue_size=1)
    vel_pub = rospy.Publisher('/missionKPH', Float32, queue_size=1)
    deg_pub = rospy.Publisher("/missionDeg", Float32, queue_size=1)
    ld_pub = rospy.Publisher('/lidar_pp_ld', Int32, queue_size=1)
    path_pub = rospy.Publisher("/local_path", PathInfo, queue_size=1)
    avoid_finish_pub = rospy.Publisher('/avoid_finish', Int32, queue_size=1)
    waypoint_vis_pub = rospy.Publisher("/waypoint_vis_pub", MarkerArray, queue_size=1)
    criteria_lane_vis_pub = rospy.Publisher("/criteria_lane_vis_pub", MarkerArray, queue_size=1)
    obstacle_vis_pub = rospy.Publisher('/obstacle_vis_pub', MarkerArray, queue_size=1)

    # ROS Subscriber
    rospy.Subscriber('/object_info', ObjectInfo, callback_obstacle)
    rospy.Subscriber("/section", Int32, callback_section)
    rospy.Subscriber('/lidarDeg', Float32, callback_deg)

    # Camera and LIDAR parameters
    rvec = np.array([[-4.30318659e-02, -9.98973266e-01,  1.41659198e-02],
                 [-3.89347726e-04, -1.41622846e-02, -9.99899634e-01],
                 [ 9.99073624e-01, -4.30330624e-02,  2.20481560e-04]])

    tvec = np.array([[-0.01219121],
                     [ 0.13399846],
                     [-0.05050436]])
    
    cameraMatrix = np.array([[739.18597047, 0., 642.15092243],
                            [0., 736.97088648, 372.35643679],
                            [0., 0., 1.]])

    src, dst, M = util.get_camera_bev_parameters()
    obstacles = list()
    sign_point = None
    delivery_finished = False
    approach_activate = False
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

    # mission_lane_control_pub.publish(1)
    # lane_control()
    # mission_lane_control_pub.publish(0)
    # print('lane finish!')

    # Get Target Sign
    f = open('{}/catkin_ws/src/mission/msg/delivery_target_sign.txt'.format(Path.home()), 'r')
    target_sign = f.read().strip()
    print('target_sign :', target_sign)

    criteria_lane_x_intercept = calculate_criteria_lane_first()
    
    print('started!!')
    brake_start_time = time.time()
    while not rospy.is_shutdown():
        image_msg = rospy.wait_for_message('/usb_cam3/image_raw', Image)
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
            coefficients = np.polyfit(y_list, x_list, 3)
            
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
                if time.time() - delivery_finish_time >= 2:
                    ld_pub.publish(7)
                else:
                    ld_pub.publish(2)

                continue


        if target_sign == 'B1':
            sign_point = calculate_sign_point(frame, sign_point, 'B1')
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
                    if time.time() - stop_start_time > 2:
                        deg_pub.publish(28)
                delivery_finish_time = time.time()
                continue

            else:
                print('sign_point :', sign_point)
                vel = BASE_SPEED
                ld_value = 5

            waypoint_temp = util.make_virtual_lane('right', criteria_lane_lidar_points, ROAD_WIDTH+0.2)
            waypoint_fit = util.fit_polynomial(waypoint_temp, 3)
            x_list = np.linspace(0, 8, 20)
            y_list = np.polyval(waypoint_fit, x_list)
            waypoint = list(zip(x_list, y_list))

            slope_avg = util.calculate_slope_avg_on_cubic_function(waypoint, waypoint_fit)
            angle_with_waypoint = math.degrees(math.atan(slope_avg))

            # if sign_point is not None and abs(sign_point[1]) <= 0.3 and angle_with_waypoint <= 5 or sign_point is None:
            #     ld_value = 7
            # else:
            #     ld_value = 3
        
        elif target_sign == 'B2':
            if approach_activate:
                sign_point = calculate_sign_point(frame, sign_point, 'B2')
                if sign_point is not None and sign_point[0] <= STOP_DISTANCE:
                    print('B2 sign_point2 :', sign_point)
                    print('stop!!')
                    delivery_finished = True
                    stop_start_time = time.time()
                    while True:
                        if time.time() - stop_start_time > STOP_TIME:
                            break
                        else:
                            vel_pub.publish(0)
                            time.sleep(0.1)
                        if time.time() - stop_start_time > 2:
                            deg_pub.publish(28)
                    delivery_finish_time = time.time()
                    continue

                else:
                    print('B2 sign_point :', sign_point)
                    vel = 5
                    ld_value = 5

                waypoint_temp = util.make_virtual_lane('right', criteria_lane_lidar_points, ROAD_WIDTH-0.2)
                waypoint_fit = util.fit_polynomial(waypoint_temp, 3)
                x_list = np.linspace(0, 8, 20)
                y_list = np.polyval(waypoint_fit, x_list)
                waypoint = list(zip(x_list, y_list))

            else:
                sign_point = calculate_sign_point(frame, sign_point, 'B1')
                if sign_point is not None and sign_point[0] <= APPROACH_DISTANCE:
                    approach_activate = True
                    sign_point = None
                    continue

                print('B1 sign_point :', sign_point)
                waypoint_temp = util.make_virtual_lane('right', criteria_lane_lidar_points, ROAD_WIDTH/2+0.2)
                waypoint_fit = util.fit_polynomial(waypoint_temp, 3)
                x_list = np.linspace(0, 8, 20)
                y_list = np.polyval(waypoint_fit, x_list)
                waypoint = list(zip(x_list, y_list))

                vel = BASE_SPEED
                ld_value = 5


        elif target_sign == 'B3':
            if approach_activate:
                sign_point = calculate_sign_point(frame, sign_point, 'B3')
                if sign_point is None:
                    sign_point = calculate_sign_point(frame, sign_point, 'B2')
                if sign_point is not None and sign_point[0] <= STOP_DISTANCE:
                    print('B3 sign_point2 :', sign_point)
                    print('stop!!')
                    delivery_finished = True
                    stop_start_time = time.time()
                    while True:
                        if time.time() - stop_start_time > STOP_TIME:
                            break
                        else:
                            vel_pub.publish(0)
                            time.sleep(0.1)
                        if time.time() - stop_start_time > 2:
                            deg_pub.publish(28)
                    delivery_finish_time = time.time()
                    continue

                else:
                    print('B3 sign_point :', sign_point)
                    vel = 5
                    ld_value = 5

                waypoint_temp = util.make_virtual_lane('right', criteria_lane_lidar_points, ROAD_WIDTH-0.2)
                waypoint_fit = util.fit_polynomial(waypoint_temp, 3)
                x_list = np.linspace(0, 8, 20)
                y_list = np.polyval(waypoint_fit, x_list)
                waypoint = list(zip(x_list, y_list))

            else:
                sign_point = calculate_sign_point(frame, sign_point, 'B2')
                if sign_point is not None and sign_point[0] <= APPROACH_DISTANCE:
                    approach_activate = True
                    sign_point = None
                    continue

                print('B2 sign_point :', sign_point)
                waypoint_temp = util.make_virtual_lane('right', criteria_lane_lidar_points, ROAD_WIDTH/2+0.2)
                waypoint_fit = util.fit_polynomial(waypoint_temp, 3)
                x_list = np.linspace(0, 8, 20)
                y_list = np.polyval(waypoint_fit, x_list)
                waypoint = list(zip(x_list, y_list))

                vel = BASE_SPEED
                ld_value = 5


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
            ob = util.visualization_marker_array([sign_point], (255,0,0), 0.2, 0.2, 0.2)
            obstacle_vis_pub.publish(ob)

        if time.time() - brake_start_time <= 1:
            vel = 2
            steering_angle = 0

        ## Publish
        publish_path(waypoint, path_pub)
        vel_pub.publish(vel)
        deg_pub.publish(steering_angle)
        ld_pub.publish(ld_value)


        # cv2.imshow('frame', frame)
        # if cv2.waitKey(25) == ord('q'):
        #     break