#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Tunnel Section 9

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, Int32, String
from object_detector.msg import ObjectInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO
from pathlib import Path
import numpy as np
import time
import json
import copy
import cv2

import sys
sys.path.append('{}/catkin_ws/src/mission/src'.format(Path.home()))
import mission_utils as util
from mission_utils import CUT_ROAD_RATIO, IMAGE_HEIGHT, WARP_SIZE, XM_PER_PIXEL

# Callback Functions

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


def calculate_criteria_lane_first():
    lane_data = rospy.wait_for_message('lane_data_publisher', String)
    lane_points = json.loads(lane_data.data)
    lanes_xys = [[(x, y - int(CUT_ROAD_RATIO * IMAGE_HEIGHT)) for x, y in sublist] for sublist in lane_points]

    ploty = np.linspace(0, WARP_SIZE-1, WARP_SIZE)

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

    right_lane_idx = util.get_right_lane_index(x_intercept_list)
    criteria_lane_x_intercept = x_intercept_list[right_lane_idx]

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

def distance_point_line(point, line):
    x0, y0 = point
    a, b = line
    A = -a
    B = 1
    C = -b
    return abs(A * x0 + B * y0 + C) / ((A**2 + B**2)**0.5)

def get_park_decide_obstacles(criteria_lane_lidar_fit):
    global obstacles
    obstacles_temp = copy.deepcopy(obstacles)

    result_obstacles = list()
    for obstacle in obstacles_temp:
        distance = distance_point_line(obstacle, criteria_lane_lidar_fit)
        if distance <= 1.0:
            result_obstacles.append(obstacle)

    result_obstacles.sort(key=lambda x: x[0])

    return result_obstacles

def calculate_cone_closed():
    bridge = CvBridge()
    while not rospy.is_shutdown():
        image_msg = rospy.wait_for_message("/usb_cam1/image_raw", Image)
        frame = bridge.imgmsg_to_cv2(image_msg, "bgr8")

        result = cone_model.predict(frame, verbose=False)[0]
        boxes = result.boxes.xyxy
        x_list = list()
        y_list = list()

        for i, box in enumerate(boxes):
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

        est_coord_list = np.array(util.convert_point_bevcam2lidar(x_list, y_list))
        if est_coord_list.size > 0:
            est_x_list = est_coord_list[:, 0]
            if min(est_x_list) < 3:
                break




if __name__ == '__main__':
    rospy.init_node('parking', anonymous=True)

    # ROS Publisher
    start_park_pub = rospy.Publisher('/start_park', Bool, queue_size=1)
    criteria_lane_vis_pub = rospy.Publisher("/criteria_lane_vis_pub", MarkerArray, queue_size=1)
    obstacle_vis_pub = rospy.Publisher('/obstacle_vis_pub', MarkerArray, queue_size=1)
    parking_obstacle_vis_pub = rospy.Publisher('/parking_obstacle_vis_pub', Marker, queue_size=1)

    # ROS Subscriber
    rospy.Subscriber('/object_info', ObjectInfo, callback_obstacle)
    rospy.Subscriber("/section", Int32, callback_section)

    obstacles = list()
    section = None
    src, dst, M = util.get_camera_bev_parameters()

    # cone_model = YOLO('{}/catkin_ws/ModelFiles/RubberCone_YOLOv10m_1280.pt'.format(Path.home()))
    # cone_model.predict(np.zeros((720, 1280, 3)), verbose=False)

    while True:
        if section == 15:
            break
        else:
            print('not parking section')
            time.sleep(0.1)

    criteria_lane_x_intercept = calculate_criteria_lane_first()

    print('Parking Started!!')

    #calculate_cone_closed()

    while not rospy.is_shutdown():

        lane_data = rospy.wait_for_message('lane_data_publisher', String)
        lanes_xys = json.loads(lane_data.data)
        lanes_xys = [[(x, y - int(CUT_ROAD_RATIO * IMAGE_HEIGHT)) for x, y in lane] for lane in lanes_xys]

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
        if ret_val is None:
            pass
        else:
            criteria_lane_idx, criteria_lane_x_intercept = ret_val
            ploty = np.linspace(0, WARP_SIZE-1, WARP_SIZE)
            criteria_lane_fit = lane_coefficients[criteria_lane_idx]
            criteria_plotx = np.polyval(criteria_lane_fit, ploty)
            criteria_lane_lidar_points = util.convert_point_bevcam2lidar(criteria_plotx, ploty)
            criteria_lane_lidar_fit = util.fit_polynomial(criteria_lane_lidar_points, 1)

        park_decide_obstacles = get_park_decide_obstacles(criteria_lane_lidar_fit)
        start_obstacle = None
        for i in range(1, len(park_decide_obstacles)):
            diff = park_decide_obstacles[i][0] - park_decide_obstacles[i-1][0]
            if diff > 2.5:
                start_obstacle = park_decide_obstacles[i-1]
                break

            if i == (len(park_decide_obstacles)-1) and park_decide_obstacles[i][0] <= 2.5:
                start_obstacle = park_decide_obstacles[i]
                break

        if start_obstacle is not None and start_obstacle[0] < 0.4:
            while True:
                start_park_pub.publish(True)
                time.sleep(0.1)
            #break
        else:
            start_park_pub.publish(False)

        ### Visualization

        # Criteria Lane Visualization
        lane_vis_points = [pt for i, pt in enumerate(criteria_lane_lidar_points) if i%10 == 0]
        ob = util.visualization_marker_array(lane_vis_points, (255,255,255), 0.1, 0.1, 0.1)
        criteria_lane_vis_pub.publish(ob)

        # Obstacle Visualization
        ob = util.visualization_marker_array(park_decide_obstacles, (255,0,0), 0.2, 0.2, 0.2, 'sphere', duration=0.1)
        obstacle_vis_pub.publish(ob)

        # Parking Obstacle Visualization
        if start_obstacle is not None:
            print('start_obstacle :', start_obstacle)
            ob = util.visualization_marker(start_obstacle, (255,255,0), 0.5, 0.5, 0.5, 'cube', duration=0.1)
            parking_obstacle_vis_pub.publish(ob)

        


        


        
        