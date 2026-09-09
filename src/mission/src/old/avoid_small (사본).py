#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 시작할 때 차선 내부에 장애물이 한개도 인식되지 않으면 바로 finished로 들어가서 코드가 동작하지 않을 수 있음

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from object_detector.msg import ObjectInfo
from std_msgs.msg import String, Int32, Float32
from geometry_msgs.msg import PoseStamped
#from ublox_msgs.msg import NavPVT
from operator import itemgetter
import numpy as np
import random
from rtree import index
from scipy.interpolate import CubicSpline
import time
from pathlib import Path
import json
import math
import copy
import cv2
import argparse

from mission.msg import PathInfo

import warnings
warnings.simplefilter('ignore', np.RankWarning)

import sys
sys.path.append('{}/catkin_ws/src/mission/src'.format(Path.home()))
from mission_utils import *



def callback_section(msg):
    global section
    section = msg.data

# 장애물 정보 콜백 함수
def callback_obstacle(msg):
    global obstacles

    obstacle_list = []

    cx = msg.centerX
    cy = msg.centerY
    lenx = msg.lengthX
    leny = msg.lengthY

    for i in range(msg.objectCounts):
        if cx[i] >= 0.3:
            center_x = cx[i]
            center_y = cy[i]

            obstacle_list.append((center_x, center_y))

    obstacles = obstacle_list

def callback_deg(data):
    global steering_angle
    steering_angle = data.data

def find_intersection(m1, c1, m2, c2):
    # 두 직선이 평행한 경우(기울기가 동일)
    if m1 == m2:
        return None  # 교점이 없거나 무수히 많음
    
    # x 좌표 계산
    x = (c2 - c1) / (m1 - m2)
    
    # y 좌표 계산 (첫 번째 직선에 대입)
    y = m1 * x + c1
    
    return x, y

def calculate_goal_point(current_obstacle_list, left_lidar_points, right_lidar_points):
    if current_obstacle_list:
        last_obstacle = current_obstacle_list[-1]

        lane_mid_points = [(left[0], (left[1]+right[1])/2) for left, right in zip(left_lidar_points, right_lidar_points)]
        lane_mid_fit = fit_polynomial(lane_mid_points)
        slope_avg = calculate_slope_avg_on_cubic_function(lane_mid_points, lane_mid_fit)
        linear_fit = calculate_line_parameters(calculate_perpendicular_slope(slope_avg), last_obstacle)
        point_temp = calculate_intersection_point_cubic_linear(lane_mid_fit, linear_fit, last_obstacle)

        offset_x, offset_y = compute_coordinate_offset(slope_avg, 2)
        result = (point_temp[0]+offset_x, point_temp[1]+offset_y)

        return result

def calculate_closest_obstacle_dir(left_lidar_points, right_lidar_points, obstacle):
    left_slope_avg = calculate_slope_avg_on_cubic_function(left_lidar_points)
    left_linear_fit = calculate_line_parameters(calculate_perpendicular_slope(left_slope_avg), obstacle)
    left_meet_point = calculate_intersection_point_cubic_linear(left_lidar_fit, left_linear_fit, obstacle)

    right_slope_avg = calculate_slope_avg_on_cubic_function(right_lidar_points)
    right_linear_fit = calculate_line_parameters(calculate_perpendicular_slope(right_slope_avg), obstacle)
    right_meet_point = calculate_intersection_point_cubic_linear(right_lidar_fit, right_linear_fit, obstacle)

    left_distance = calculate_distance(left_meet_point, obstacle)
    right_distance = calculate_distance(right_meet_point, obstacle)

    if left_distance < right_distance:
        obstacle_dir = 'left'
    else:
        obstacle_dir = 'right'

    return obstacle_dir

def translate_cubic_curve(coeffs, point):
    a, b, c, d = coeffs
    x0, y0 = point
    
    new_coeffs = [a, b, c, d - (a * x0**3 + b * x0**2 + c * x0 + d - y0)]  # shift by y value
    
    return new_coeffs

def calculate_current_obstacles(left_lidar_fit, right_lidar_fit, obstacles):
    if obstacles is None:
        return

    current_obstacles = list()
 
    #print('obstacles :', obstacles)
    for pt in obstacles:
        ob_x, ob_y = pt

        left_lane_ob_y = np.polyval(left_lidar_fit, ob_x)
        right_lane_ob_y = np.polyval(right_lidar_fit, ob_x)

        if right_lane_ob_y-0.3 <= ob_y <= left_lane_ob_y+0.3:  # 0.3m만큼 더 여유를 줌
            current_obstacles.append((ob_x, ob_y))

    return current_obstacles

def calculate_current_lane_lidar_points(lanes_xys, M):
    ploty = np.linspace(0, WARP_SIZE-1, WARP_SIZE)

    lane_coefficients = list()
    x_intercept_list = list()
    for lane_points in lanes_xys:
        points = np.array(lane_points, dtype=np.float32)
        points = points.reshape(-1, 1, 2)
        
        transformed_points = cv2.perspectiveTransform(points, M).reshape(-1, 2)
        transformed_points[:, 1] = -(transformed_points[:, 1] - WARP_SIZE)
        transformed_points = transformed_points[transformed_points[:, 1] * YM_PER_PIXEL + CAR_OFFSET <= 5]

        if transformed_points.size != 0:
            x_list = transformed_points[:, 0]
            y_list = transformed_points[:, 1]

            coefficients = np.polyfit(y_list, x_list, 1)
            
            lane_coefficients.append(coefficients)
            x_intercept_list.append(coefficients[-1])

    left_lane_idx = get_left_lane_index(x_intercept_list)
    right_lane_idx = get_right_lane_index(x_intercept_list)

    if left_lane_idx is not None and right_lane_idx is not None:  # 양쪽 차선이 모두 인식된 경우
        left_fit = lane_coefficients[left_lane_idx]
        right_fit = lane_coefficients[right_lane_idx]

        left_plotx = np.polyval(left_fit, ploty)
        right_plotx = np.polyval(right_fit, ploty)

        left_lidar_points = convert_point_bevcam2lidar(left_plotx, ploty)
        right_lidar_points = convert_point_bevcam2lidar(right_plotx, ploty)

    elif left_lane_idx is None and right_lane_idx is not None:  # 오른쪽 차선만 인식된 경우
        right_fit = lane_coefficients[right_lane_idx]
        right_plotx = np.polyval(right_fit, ploty)
        right_lidar_points = convert_point_bevcam2lidar(right_plotx, ploty)

        left_lidar_points = make_virtual_lane('left', right_lidar_points)

    elif left_lane_idx is not None and right_lane_idx is None:  # 왼쪽 차선만 인식된 경우
        left_fit = lane_coefficients[left_lane_idx]
        left_plotx = np.polyval(left_fit, ploty)
        left_lidar_points = convert_point_bevcam2lidar(left_plotx, ploty)

        right_lidar_points = make_virtual_lane('right', left_lidar_points)

    else:
        return None


    return left_lidar_points, right_lidar_points


if __name__ == '__main__':
    rospy.init_node('avoid_small', anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('--section', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])

    src, dst, M = get_camera_bev_parameters()

    left_lidar_points = None
    right_lidar_points = None
    obstacles = list()
    section = None
    finish_start_time = None
    first_obstacle_passed = False
    second_obstacle_passed = False
    first_obstacle_closed = False
    obstacle_close_distance = 8
    steering_angle = 0

    # ROS Publisher
    path_pub = rospy.Publisher("/local_path", PathInfo, queue_size=1)
    avoid_finish_pub = rospy.Publisher('/avoid_finish', Int32, queue_size=1)
    waypoint_vis_pub = rospy.Publisher("/waypoint_vis_pub", MarkerArray, queue_size=1)
    obstacle_vis_pub = rospy.Publisher("/obstacle_vis_pub", MarkerArray, queue_size=1)
    lane_vis_pub = rospy.Publisher("/lane_vis_pub", MarkerArray, queue_size=1)
    goal_vis_pub = rospy.Publisher('/goal_vis_pub', Marker, queue_size=1)
    vel_pub = rospy.Publisher("/missionKPH", Float32, queue_size=1)
    deg_pub = rospy.Publisher("/missionDeg", Float32, queue_size=1)
    ld_level_pub = rospy.Publisher('small_ld_level', Int32, queue_size=1)

    # ROS Subscriber
    rospy.Subscriber('/object_info', ObjectInfo, callback_obstacle)
    rospy.Subscriber("/section", Int32, callback_section)
    rospy.Subscriber('/lidarDeg', Float32, callback_deg)

    if args.section:
        while True:
            if section == 11:
                break
            else:
                print('not avoid section')
                time.sleep(0.1)

    finished = False
    start_time = None
    print('started!!\n')
    ld_level_pub.publish(0)
    while not rospy.is_shutdown():

        lane_data = rospy.wait_for_message('lane_data_publisher', String)
        lanes_xys = json.loads(lane_data.data)
        lanes_xys = [[(x, y - int(CUT_ROAD_RATIO * IMAGE_HEIGHT)) for x, y in lane] for lane in lanes_xys]

        left_lidar_points, right_lidar_points = calculate_current_lane_lidar_points(lanes_xys, M)
        left_lidar_fit = fit_polynomial(left_lidar_points)
        right_lidar_fit = fit_polynomial(right_lidar_points)

        # Lane Visualization
        lane_vis_points = [pt for i, pt in enumerate(left_lidar_points+right_lidar_points) if i%10==0]
        ob = visualization_marker_array(lane_vis_points, (255,255,255), 0.1, 0.1, 0.1)
        lane_vis_pub.publish(ob)

        obstacles_temp = copy.deepcopy(obstacles)
        current_obstacle_list = calculate_current_obstacles(left_lidar_fit, right_lidar_fit, obstacles_temp)
        
        # Obstacle Visualization
        ob = visualization_marker_array(current_obstacle_list, (255,0,0), 0.3, 0.3, 0.3)
        obstacle_vis_pub.publish(ob)

        # Calculate Goal Point
        goal_point = calculate_goal_point(current_obstacle_list, left_lidar_points, right_lidar_points)
        if goal_point is not None:
            ob = visualization_marker(goal_point)
            goal_vis_pub.publish(ob)
        waypoint = [[0, 0], goal_point]

        if not first_obstacle_passed and not second_obstacle_passed:
            if first_obstacle_closed:  #(2)
                print(22222222)

                closest_obstacle = current_obstacle_list[0]
                closest_obstacle_dir = calculate_closest_obstacle_dir(left_lidar_points, right_lidar_points, closest_obstacle)
                if closest_obstacle_dir == 'right':
                    first_obstacle_passed = True
                    continue

                for obstacle in current_obstacle_list:
                    left_slope_avg = calculate_slope_avg_on_cubic_function(left_lidar_points)
                    left_linear_fit = calculate_line_parameters(calculate_perpendicular_slope(left_slope_avg), obstacle)
                    left_meet_point = calculate_intersection_point_cubic_linear(left_lidar_fit, left_linear_fit, obstacle)

                    right_slope_avg = calculate_slope_avg_on_cubic_function(right_lidar_points)
                    right_linear_fit = calculate_line_parameters(calculate_perpendicular_slope(right_slope_avg), obstacle)
                    right_meet_point = calculate_intersection_point_cubic_linear(right_lidar_fit, right_linear_fit, obstacle)

                    left_distance = calculate_distance(left_meet_point, obstacle)
                    right_distance = calculate_distance(right_meet_point, obstacle)

                    obstacle_x, obstacle_y = obstacle

                    if left_distance < right_distance:  # Left Obstacle
                        slope = calculate_perpendicular_slope(left_slope_avg)
                        linear_fit = calculate_line_parameters(slope, obstacle)
                        inter_x, inter_y = calculate_intersection_point_cubic_linear(right_lidar_fit, linear_fit, obstacle)
                        x = obstacle_x + (inter_x - obstacle_x) * 0.6
                        y = obstacle_y + (inter_y - obstacle_y) * 0.6
                        waypoint.append([x, y])

                    else:  # Right Obstacle
                        slope = calculate_perpendicular_slope(right_slope_avg)
                        linear_fit = calculate_line_parameters(slope, obstacle)
                        inter_x, inter_y = calculate_intersection_point_cubic_linear(left_lidar_fit, linear_fit, obstacle)
                        x = inter_x + (obstacle_x - inter_x) * 0.4
                        y = inter_y + (obstacle_y - inter_y) * 0.4
                        waypoint.append([x, y])

                waypoint = sorted(waypoint, key=lambda coord: coord[0])

                temp = list()
                for i in range(1, len(waypoint)):
                    pt1 = waypoint[i-1]
                    pt2 = waypoint[i]

                    num_points = 4
                    dx = (pt2[0] - pt1[0]) / (num_points - 1)
                    dy = (pt2[1] - pt1[1]) / (num_points - 1)
                    
                    # 각 지점 계산
                    points = [[pt1[0] + i * dx, pt1[1] + i * dy] for i in range(num_points)]
                    temp.extend(points[1:-1])

                waypoint.extend(temp)
                waypoint = sorted(waypoint, key=lambda coord: coord[0])


            else:  # (1)
                print(11111111)
                if current_obstacle_list:
                    closest_obstacle = current_obstacle_list[0]
                    distance = calculate_distance((0,0), closest_obstacle)
                    print('distance :', distance)
                    if distance < obstacle_close_distance:
                        first_obstacle_closed = True
                        continue

                waypoint = generate_lane_center_path(left_lidar_points, right_lidar_points)


        elif first_obstacle_passed and not second_obstacle_passed:
            print(33333333)

            if not current_obstacle_list:
                second_obstacle_passed = True
                finish_start_time = time.time()
                continue

            for obstacle in current_obstacle_list:
                left_slope_avg = calculate_slope_avg_on_cubic_function(left_lidar_points)
                left_linear_fit = calculate_line_parameters(calculate_perpendicular_slope(left_slope_avg), obstacle)
                left_meet_point = calculate_intersection_point_cubic_linear(left_lidar_fit, left_linear_fit, obstacle)

                right_slope_avg = calculate_slope_avg_on_cubic_function(right_lidar_points)
                right_linear_fit = calculate_line_parameters(calculate_perpendicular_slope(right_slope_avg), obstacle)
                right_meet_point = calculate_intersection_point_cubic_linear(right_lidar_fit, right_linear_fit, obstacle)

                left_distance = calculate_distance(left_meet_point, obstacle)
                right_distance = calculate_distance(right_meet_point, obstacle)

                obstacle_x, obstacle_y = obstacle

                if left_distance < right_distance:  # Left Obstacle
                    slope = calculate_perpendicular_slope(left_slope_avg)
                    linear_fit = calculate_line_parameters(slope, obstacle)
                    inter_x, inter_y = calculate_intersection_point_cubic_linear(right_lidar_fit, linear_fit, obstacle)
                    x = obstacle_x + (inter_x - obstacle_x) * 0.5
                    y = obstacle_y + (inter_y - obstacle_y) * 0.5
                    waypoint.append([x, y])


                else:  # Right Obstacle
                    slope = calculate_perpendicular_slope(right_slope_avg)
                    linear_fit = calculate_line_parameters(slope, obstacle)
                    inter_x, inter_y = calculate_intersection_point_cubic_linear(left_lidar_fit, linear_fit, obstacle)
                    x = inter_x + (obstacle_x - inter_x) * 0.5
                    y = inter_y + (obstacle_y - inter_y) * 0.5
                    waypoint.append([x, y])

            waypoint = sorted(waypoint, key=lambda coord: coord[0])

            temp = list()
            for i in range(1, len(waypoint)):
                pt1 = waypoint[i-1]
                pt2 = waypoint[i]

                num_points = 4
                dx = (pt2[0] - pt1[0]) / (num_points - 1)
                dy = (pt2[1] - pt1[1]) / (num_points - 1)
                
                # 각 지점 계산
                points = [[pt1[0] + i * dx, pt1[1] + i * dy] for i in range(num_points)]
                temp.extend(points[1:-1])

            waypoint.extend(temp)
            waypoint = sorted(waypoint, key=lambda coord: coord[0])




        elif first_obstacle_passed and second_obstacle_passed:
            print(44444444)

            waypoint = generate_lane_center_path(left_lidar_points, right_lidar_points)

            # waypoint_temp = make_virtual_lane('right', left_lidar_points, 1.0)
            # waypoint_fit = fit_polynomial(waypoint_temp)
            # x_list = np.linspace(0, 7, 20)
            # y_list = np.polyval(waypoint_fit, x_list)
            # waypoint = list(zip(x_list, y_list))

            if time.time() - finish_start_time > 2:
                print('finish!!')
                avoid_finish_pub.publish(1)
                ld_level_pub.publish(1)
                break

        
        x_points = [pt[0] for pt in waypoint]
        y_points = [pt[1] for pt in waypoint]
        cs = CubicSpline(x_points, y_points)
        x_new = np.linspace(x_points[0], x_points[-1], 20)
        y_new = cs(x_new)
        waypoint = list(zip(x_new, y_new)) 

        # Path Visuallization
        ob = visualization_marker_array(waypoint, (0,255,0), 0.1, 0.1, 0.1, duration=0.1)
        waypoint_vis_pub.publish(ob)
        
        # Publish Waypoint
        x_points = [pt[0] for pt in waypoint]
        y_points = [pt[1] for pt in waypoint]
        path_msg = PathInfo()
        path_msg.cnt = 20
        path_msg.x = x_points
        path_msg.y = y_points
        path_pub.publish(path_msg)

        vel_pub.publish(8)
        deg_pub.publish(steering_angle)


        