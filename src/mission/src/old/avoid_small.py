#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from object_detector.msg import ObjectInfo
from std_msgs.msg import String, Int32, Float32
from mission.msg import PathInfo

from scipy.interpolate import CubicSpline
from pathlib import Path
import numpy as np
import time
import json
import cv2
import argparse


import warnings
warnings.simplefilter('ignore', np.RankWarning)

import sys
sys.path.append('{}/catkin_ws/src/mission/src'.format(Path.home()))
import mission_utils as util
from mission_utils import WARP_SIZE, CAR_OFFSET, CUT_ROAD_RATIO, YM_PER_PIXEL, IMAGE_HEIGHT

AVOID_FINISH_TIME = 3
BASE_SPEED = 6
OBSTACLE_CLOSE_DISTANCE = 9

### Callback Functions
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
    global steering_angle
    steering_angle = data.data


def calculate_goal_point(current_obstacle_list, left_lane_lidar_points, right_lane_lidar_points, first_obstacle_passed):
    if current_obstacle_list:
        distance_list = [util.calculate_distance((0,0), pt) for pt in current_obstacle_list]
        max_idx = np.argmax(distance_list)
        last_obstacle = current_obstacle_list[max_idx]

        if not first_obstacle_passed:
            lane_mid_points = [(left[0], (left[1]+right[1])/2) for left, right in zip(left_lane_lidar_points, right_lane_lidar_points)]
            lane_mid_fit = util.fit_polynomial(lane_mid_points)
            slope_avg = util.calculate_slope_avg_on_cubic_function(lane_mid_points, lane_mid_fit)
            linear_fit = util.calculate_line_parameters(util.calculate_perpendicular_slope(slope_avg), last_obstacle)
            point_temp = util.calculate_intersection_point_cubic_linear(lane_mid_fit, linear_fit, last_obstacle)

            offset_x, offset_y = util.compute_coordinate_offset(slope_avg, 2)
            result = (point_temp[0]+offset_x, point_temp[1]+offset_y)

        else:
            lane_mid_points = util.make_virtual_lane('right', left_lane_lidar_points, 1.3)
            lane_mid_fit = util.fit_polynomial(lane_mid_points)
            slope_avg = util.calculate_slope_avg_on_cubic_function(lane_mid_points, lane_mid_fit)
            linear_fit = util.calculate_line_parameters(util.calculate_perpendicular_slope(slope_avg), last_obstacle)
            point_temp = util.calculate_intersection_point_cubic_linear(lane_mid_fit, linear_fit, last_obstacle)

            offset_x, offset_y = util.compute_coordinate_offset(slope_avg, 2)
            result = (point_temp[0]+offset_x, point_temp[1]+offset_y) 

        return result

def calculate_closest_obstacle(current_obstacle_list):
    distance_list = [util.calculate_distance((0,0), pt) for pt in current_obstacle_list]
    min_idx = np.argmin(distance_list)
    closest_obstacle = current_obstacle_list[min_idx]

    return closest_obstacle

def calculate_closest_obstacle_dir(left_lane_lidar_points, right_lane_lidar_points, obstacle):
    left_slope_avg = util.calculate_slope_avg_on_cubic_function(left_lane_lidar_points)
    left_linear_fit = util.calculate_line_parameters(util.calculate_perpendicular_slope(left_slope_avg), obstacle)
    left_meet_point = util.calculate_intersection_point_cubic_linear(left_lidar_fit, left_linear_fit, obstacle)

    right_slope_avg = util.calculate_slope_avg_on_cubic_function(right_lane_lidar_points)
    right_linear_fit = util.calculate_line_parameters(util.calculate_perpendicular_slope(right_slope_avg), obstacle)
    right_meet_point = util.calculate_intersection_point_cubic_linear(right_lidar_fit, right_linear_fit, obstacle)

    left_distance = util.calculate_distance(left_meet_point, obstacle)
    right_distance = util.calculate_distance(right_meet_point, obstacle)

    if left_distance < right_distance:
        obstacle_dir = 'left'
    else:
        obstacle_dir = 'right'

    return obstacle_dir

# 기능 : 현재 차선 내에 있는 장애물의 위치를 계산함
def calculate_current_obstacles(left_lidar_fit, right_lidar_fit):
    global obstacles

    current_obstacles = list()
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

    left_lane_idx = util.get_left_lane_index(x_intercept_list)
    right_lane_idx = util.get_right_lane_index(x_intercept_list)

    if left_lane_idx is not None and right_lane_idx is not None:  # 양쪽 차선이 모두 인식된 경우
        left_fit = lane_coefficients[left_lane_idx]
        right_fit = lane_coefficients[right_lane_idx]

        left_plotx = np.polyval(left_fit, ploty)
        right_plotx = np.polyval(right_fit, ploty)

        left_lane_lidar_points = util.convert_point_bevcam2lidar(left_plotx, ploty)
        right_lane_lidar_points = util.convert_point_bevcam2lidar(right_plotx, ploty)

    elif left_lane_idx is None and right_lane_idx is not None:  # 오른쪽 차선만 인식된 경우
        right_fit = lane_coefficients[right_lane_idx]
        right_plotx = np.polyval(right_fit, ploty)
        right_lane_lidar_points = util.convert_point_bevcam2lidar(right_plotx, ploty)

        left_lane_lidar_points = util.make_virtual_lane('left', right_lane_lidar_points)

    elif left_lane_idx is not None and right_lane_idx is None:  # 왼쪽 차선만 인식된 경우
        left_fit = lane_coefficients[left_lane_idx]
        left_plotx = np.polyval(left_fit, ploty)
        left_lane_lidar_points = util.convert_point_bevcam2lidar(left_plotx, ploty)

        right_lane_lidar_points = util.make_virtual_lane('right', left_lane_lidar_points)

    else:
        return None


    return left_lane_lidar_points, right_lane_lidar_points

def publish_path(waypoint, path_pub):
    x_points = [pt[0] for pt in waypoint]
    y_points = [pt[1] for pt in waypoint]

    path_msg = PathInfo()
    path_msg.cnt = 20
    path_msg.x = x_points
    path_msg.y = y_points

    path_pub.publish(path_msg)


if __name__ == '__main__':
    rospy.init_node('avoid_small', anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('--section', action='store_true')
    parser.add_argument('--tunnel', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])

    src, dst, M = util.get_camera_bev_parameters()

    # Variables
    left_lane_lidar_points = None
    right_lane_lidar_points = None
    obstacles = list()
    section = None
    finish_start_time = None
    first_obstacle_passed = False
    second_obstacle_passed = False
    first_obstacle_closed = False
    steering_angle = 0
    ld_value = 7

    # ROS Publisher
    path_pub = rospy.Publisher("/local_path", PathInfo, queue_size=1)
    vel_pub = rospy.Publisher("/missionKPH", Float32, queue_size=1)
    deg_pub = rospy.Publisher("/missionDeg", Float32, queue_size=1)
    ld_pub = rospy.Publisher('/lidar_pp_ld', Int32, queue_size=1)
    avoid_finish_pub = rospy.Publisher('/avoid_finish', Int32, queue_size=1)
    tunnel_avoid_finish_pub = rospy.Publisher('/tunnel_avoid_finish', Int32, queue_size=1)
    waypoint_vis_pub = rospy.Publisher("/waypoint_vis_pub", MarkerArray, queue_size=1)
    obstacle_vis_pub = rospy.Publisher("/obstacle_vis_pub", MarkerArray, queue_size=1)
    lane_vis_pub = rospy.Publisher("/lane_vis_pub", MarkerArray, queue_size=1)
    goal_vis_pub = rospy.Publisher('/goal_vis_pub', Marker, queue_size=1)

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

    if args.tunnel:
        while True:
            print('wait for tunnel section')
            value = rospy.wait_for_message('/tunnel_avoid_start', Int32)
            if value.data == 1:
                print('\n\nget tunnel')
                break

    print('started!!\n')
    brake_start_time = time.time()
    while not rospy.is_shutdown():

        lane_data = rospy.wait_for_message('lane_data_publisher', String)
        lanes_xys = json.loads(lane_data.data)
        lanes_xys = [[(x, y - int(CUT_ROAD_RATIO * IMAGE_HEIGHT)) for x, y in lane] for lane in lanes_xys]

        ret_val = util.get_current_lane_lidar_points(lanes_xys, M, virtual_lane_width=3.5)
        if ret_val is None:
            if left_lane_lidar_points is None or right_lane_lidar_points is None:
                continue
        else:
            left_lane_lidar_points, right_lane_lidar_points = ret_val
        left_lidar_fit = util.fit_polynomial(left_lane_lidar_points)
        right_lidar_fit = util.fit_polynomial(right_lane_lidar_points)

        current_obstacle_list = calculate_current_obstacles(left_lidar_fit, right_lidar_fit)

        # Calculate Goal Point
        goal_point = calculate_goal_point(current_obstacle_list, left_lane_lidar_points, right_lane_lidar_points, first_obstacle_passed)
        waypoint = [[0, 0], goal_point]

        if not first_obstacle_passed and not second_obstacle_passed:
            if len(current_obstacle_list) == 0:
                waypoint = util.generate_lane_center_path(left_lane_lidar_points, right_lane_lidar_points)
                vel = BASE_SPEED
                ld_value = 7

            else:
                if first_obstacle_closed:  #(2)
                    print(22222222)

                    closest_obstacle = calculate_closest_obstacle(current_obstacle_list)
                    closest_obstacle_dir = calculate_closest_obstacle_dir(left_lane_lidar_points, right_lane_lidar_points, closest_obstacle)

                    for obstacle in current_obstacle_list:
                        left_slope_avg = util.calculate_slope_avg_on_cubic_function(left_lane_lidar_points)
                        left_linear_fit = util.calculate_line_parameters(util.calculate_perpendicular_slope(left_slope_avg), obstacle)
                        left_meet_point = util.calculate_intersection_point_cubic_linear(left_lidar_fit, left_linear_fit, obstacle)

                        right_slope_avg = util.calculate_slope_avg_on_cubic_function(right_lane_lidar_points)
                        right_linear_fit = util.calculate_line_parameters(util.calculate_perpendicular_slope(right_slope_avg), obstacle)
                        right_meet_point = util.calculate_intersection_point_cubic_linear(right_lidar_fit, right_linear_fit, obstacle)

                        left_distance = util.calculate_distance(left_meet_point, obstacle)
                        right_distance = util.calculate_distance(right_meet_point, obstacle)

                        obstacle_x, obstacle_y = obstacle

                        if left_distance < right_distance:  # Left Obstacle
                            slope = util.calculate_perpendicular_slope(left_slope_avg)
                            linear_fit = util.calculate_line_parameters(slope, obstacle)
                            inter_x, inter_y = util.calculate_intersection_point_cubic_linear(right_lidar_fit, linear_fit, obstacle)
                            x = obstacle_x + (inter_x - obstacle_x) * 0.6
                            y = obstacle_y + (inter_y - obstacle_y) * 0.6
                            waypoint.append([x, y])

                        else:  # Right Obstacle
                            slope = util.calculate_perpendicular_slope(right_slope_avg)
                            linear_fit = util.calculate_line_parameters(slope, obstacle)
                            inter_x, inter_y = util.calculate_intersection_point_cubic_linear(left_lidar_fit, linear_fit, obstacle)
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

                    closest_obstacle_distance = util.calculate_distance((0,0), closest_obstacle)
                    if closest_obstacle_distance >= 5.5:
                        vel = BASE_SPEED
                        ld_value = 7
                    elif closest_obstacle_distance >= 3:
                        vel = BASE_SPEED
                        ld_value = 3
                    else:
                        first_obstacle_passed = True
                        continue


                else:  # (1)
                    print(11111111)
                    if current_obstacle_list:
                        closest_obstacle = calculate_closest_obstacle(current_obstacle_list)
                        distance = util.calculate_distance((0,0), closest_obstacle)
                        print('distance :', distance)
                        if distance < OBSTACLE_CLOSE_DISTANCE:
                            first_obstacle_closed = True
                            continue

                    waypoint = util.generate_lane_center_path(left_lane_lidar_points, right_lane_lidar_points)
                    vel = BASE_SPEED
                    ld_value = 7


        elif first_obstacle_passed and not second_obstacle_passed:
            print(33333333)

            if len(current_obstacle_list) == 0:
                second_obstacle_passed = True
                finish_start_time = time.time()
                continue

            for obstacle in current_obstacle_list:
                left_slope_avg = util.calculate_slope_avg_on_cubic_function(left_lane_lidar_points)
                left_linear_fit = util.calculate_line_parameters(util.calculate_perpendicular_slope(left_slope_avg), obstacle)
                left_meet_point = util.calculate_intersection_point_cubic_linear(left_lidar_fit, left_linear_fit, obstacle)

                right_slope_avg = util.calculate_slope_avg_on_cubic_function(right_lane_lidar_points)
                right_linear_fit = util.calculate_line_parameters(util.calculate_perpendicular_slope(right_slope_avg), obstacle)
                right_meet_point = util.calculate_intersection_point_cubic_linear(right_lidar_fit, right_linear_fit, obstacle)

                left_distance = util.calculate_distance(left_meet_point, obstacle)
                right_distance = util.calculate_distance(right_meet_point, obstacle)

                obstacle_x, obstacle_y = obstacle
                if left_distance > right_distance:  # Right Obstacle
                    slope = util.calculate_perpendicular_slope(right_slope_avg)
                    linear_fit = util.calculate_line_parameters(slope, obstacle)
                    inter_x, inter_y = util.calculate_intersection_point_cubic_linear(left_lidar_fit, linear_fit, obstacle)
                    x = inter_x + (obstacle_x - inter_x) * 0.5  # 0이면 왼쪽으로 붙고, 1이면 오른쪽으로 붙음
                    y = inter_y + (obstacle_y - inter_y) * 0.5
                    waypoint.append([x, y])
                    break

            if util.calculate_distance((0,0), (obstacle_x, obstacle_y)) <= 1.5:
                print('\n\nsecond obstacle passed!!\n\n')
                second_obstacle_passed = True
                finish_start_time = time.time()
                continue



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

            vel = BASE_SPEED
            ld_value = 3


        elif first_obstacle_passed and second_obstacle_passed:
            print(44444444)

            waypoint = util.generate_lane_center_path(left_lane_lidar_points, right_lane_lidar_points)

            vel = 6
            ld_value = 7

            if time.time() - finish_start_time > AVOID_FINISH_TIME:
                print('finish!!')
                if args.tunnel:
                    tunnel_avoid_finish_pub.publish(1)
                else:
                    avoid_finish_pub.publish(1)
                break

        x_points = [pt[0] for pt in waypoint]
        y_points = [pt[1] for pt in waypoint]
        cs = CubicSpline(x_points, y_points)
        x_new = np.linspace(x_points[0], x_points[-1], 20)
        y_new = cs(x_new)
        waypoint = list(zip(x_new, y_new)) 

        if args.tunnel and time.time() - brake_start_time <= 1:
            vel = 4
            ld_value = 7
            waypoint = util.generate_lane_center_path(left_lane_lidar_points, right_lane_lidar_points)

        ### Visualization

        # Lane Visualization
        lane_vis_points = [pt for i, pt in enumerate(left_lane_lidar_points+right_lane_lidar_points) if i%10==0]
        ob = util.visualization_marker_array(lane_vis_points, (255,255,255), 0.1, 0.1, 0.1)
        lane_vis_pub.publish(ob)

        # Obstacle Visualization
        ob = util.visualization_marker_array(current_obstacle_list, (255,0,0), 0.3, 0.3, 0.3)
        obstacle_vis_pub.publish(ob)

        # Path Visuallization
        ob = util.visualization_marker_array(waypoint, (0,255,0), 0.1, 0.1, 0.1, duration=0.1)
        waypoint_vis_pub.publish(ob)

        # Goal Point Visualization
        if goal_point is not None:
            ob = util.visualization_marker(goal_point, (255,255,0), marker_type_input='sphere')
            goal_vis_pub.publish(ob)
        
        # Publish
        publish_path(waypoint, path_pub)
        vel_pub.publish(vel)
        deg_pub.publish(steering_angle)
        ld_pub.publish(ld_value)


        