#!/usr/bin/env python3
# -- coding: utf-8 --

# Lane Section 1, Tunnel Section 9

import rospy
from std_msgs.msg import Float32, String, Int32, Bool
from visualization_msgs.msg import MarkerArray

from pathlib import Path
import numpy as np
import argparse
import json
import time
import copy
import math

import warnings
warnings.simplefilter('ignore', np.RankWarning)

import sys
sys.path.append('{}/catkin_ws/src/mission/src'.format(Path.home()))
import mission_utils as util
from mission_utils import CUT_ROAD_RATIO, IMAGE_HEIGHT, CAR_OFFSET

STATE_CLEAR = 0
STATE_SLOW_FAR = 1
STATE_SLOW_NEAR = 2
STATE_STOP = 3
STATE_RESUME = 4

SLOW_FAR_VEL = 6
SLOW_NEAR_VEL = 3
STOP_VEL = 0

stop_sign_state = STATE_CLEAR
# Callback Functions

def callback_section(msg):
    global section, road_width
    section = msg.data

    if section == 1 or section == 20:  # Vision, DeliveryA, Parking
        road_width = 4.0
    elif section == 8:  # UTurn
        road_width = 3.8
    else:  # Default
        road_width = 3.5

def callback_tunnel(msg):
    global tunnel_distinguish
    tunnel_distinguish = msg.data

def callback_mission_lane_control(data):
    global mission_lane_control
    mission_lane_control = bool(data.data)

def callback_parking_semi(data):
    global parking_semi_started, section

    if section == 14:
        parking_semi_started = data.data

def callback_parking_final(data):
    global parking_final_started, section

    if section == 15:
        parking_final_started = data.data

# def callback_uturn(data):
#     global uturn_closed
#     uturn_closed = data.data


def callback_stop_sign(msg):
    global stop_sign_state
    try:
        stop_sign_state = int(getattr(msg, "data", STATE_CLEAR))
    except (TypeError, ValueError):
        stop_sign_state = STATE_CLEAR

def check_lane_section(lane_sections, mission_sections):
    global section, mission_lane_control, tunnel_distinguish, left_lane_lidar_points, right_lane_lidar_points

    ret_val = True
    if args.section and section not in lane_sections:
        print('not lane section')
        time.sleep(0.1)
        ret_val = False

    # if args.section and section in mission_sections and section != 10 and not mission_lane_control:
    if args.section and section in mission_sections and not mission_lane_control:
        print('not lane mission')
        time.sleep(0.1)
        ret_val = False

    if ret_val == False:
        left_lane_lidar_points = None
        right_lane_lidar_points = None

    return ret_val





def calculate_angle(left_lane_lidar_points, right_lane_lidar_points):
    plotx = np.array([pt[0] for pt in left_lane_lidar_points])

    lefty = np.array([pt[1] for pt in left_lane_lidar_points])
    righty = np.array([pt[1] for pt in right_lane_lidar_points])
    mid_idx = util.calculate_m2idx(4, plotx)

    """ 2024-07-19 차선 중간 경로 포인트 생성 """
    waypoint_temp = []
    mid_center_x = plotx[mid_idx]
    mid_center_y = (lefty[mid_idx] + righty[mid_idx]) / 2
    start_x = 0 #CAR_OFFSET print("start_x ",start_x)
    start_y = 0
    temp_fit = np.polyfit([start_x, mid_center_x], [start_y, mid_center_y], 1)
    for i in range(len(lefty)):
        if i < mid_idx:
            mid_y = temp_fit[0]*plotx[i] + temp_fit[1]
        else:
            mid_y = (lefty[i] + righty[i]) / 2
        mid_x = plotx[i]
        waypoint_temp.append((mid_x, mid_y))
    """ 차선 중간 경로 포인트 생성 끝 """

    waypoint_x = list()
    waypoint_y = list()
    for pt in waypoint_temp:
        waypoint_x.append(pt[0])
        waypoint_y.append(pt[1])
    waypoint_fit = np.polyfit(waypoint_x, waypoint_y, 3)
    waypoint_ploty = np.polyval(waypoint_fit, plotx)

    waypoint = list()
    for i in range(len(plotx)):
        waypoint.append((plotx[i], waypoint_ploty[i]))

    # ----- Pure Pursuit 준비 -----
    L = 1.04  # wheelbase

    # 화면/차량 좌표계에서 차량 위치 (기존 가정 유지)
    car_location_x, car_location_y = 0.0, 0.0

    # 원하는 Ld(lookahead distance)만 정하면 됨
    distance = 4.0

    # >>> 고정 인덱스(504) 대신, 차량으로부터의 거리 기반으로 타겟 인덱스 선택 <<<
    if len(waypoint) == 0:
        return 0.0, waypoint  # 안전 가드

    wp = np.asarray(waypoint)                    # shape: (N, 2)
    dists = np.hypot(wp[:,0] - car_location_x,
                     wp[:,1] - car_location_y)  # 차량(0,0)으로부터 각 웨이포인트까지 거리

    # distance 이상이 되는 첫 인덱스 선택 (없으면 마지막 인덱스)
    idx_candidates = np.where(dists >= distance)[0]
    if idx_candidates.size > 0:
        target_index = int(idx_candidates[0])
    else:
        target_index = len(waypoint) - 1

    # 각도 계산
    dx = waypoint[target_index][0] - car_location_x
    dy = waypoint[target_index][1] - car_location_y
    temp_alpha = np.arctan2(dy, dx)

    radian = np.arctan(2 * L * np.sin(temp_alpha) / (distance))
    angle = radian * (180.0 / np.pi)
    
    """
    angle_offset = -2.0
    angle += angle_offset
    """

    return angle, waypoint




if __name__ == '__main__':
    rospy.init_node('lane_control', anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('--section', action='store_true')
    parser.add_argument('--tunnel', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])

    # Variables
    src, dst, M = util.get_camera_bev_parameters()
    previous_left_lane_lidar_points = None
    previous_right_lane_lidar_points = None
    left_lane_lidar_points = None
    right_lane_lidar_points = None
    section = None
    parking_final_started = False
    parking_semi_started = False
    # uturn_closed = False
    tunnel_distinguish = 0
    mission_lane_control = False  # 미션 코드에서 차선유지 사용 여부
    lane_sections = [1, 8, 9, 10, 12, 14, 15, 20]
    mission_sections = [8, 9, 12, 20]
    road_width = 3.5

    # ROS Publisher
    lane_vis_pub = rospy.Publisher("/lane_vis_pub", MarkerArray, queue_size=1)
    waypoint_vis_pub = rospy.Publisher("/waypoint_vis_pub", MarkerArray, queue_size=1)
    steering_angle_pub = rospy.Publisher('/steering_angle', Float32, queue_size=10)  # Vision Deg Publisher
    vel_pub = rospy.Publisher('/missionKPH', Float32, queue_size=1)  # Mission Speed Publisher
    deg_pub = rospy.Publisher("/missionDeg", Float32, queue_size=1)  # Mission Deg Publisher
    
    # ROS Subscriber
    rospy.Subscriber("/section", Int32, callback_section)
    rospy.Subscriber('/mission_lane_control', Int32, callback_mission_lane_control)
    rospy.Subscriber('/start_park_final', Bool, callback_parking_final)
    rospy.Subscriber('/start_park_semi', Bool, callback_parking_semi)

    # rospy.Subscriber('/uturn_closed', Bool, callback_uturn)
    rospy.Subscriber('/stop_sign', Int32, callback_stop_sign)

    while not rospy.is_shutdown():

        if not check_lane_section(lane_sections, mission_sections):
            continue

        previous_left_lane_lidar_points = copy.deepcopy(left_lane_lidar_points)
        previous_right_lane_lidar_points = copy.deepcopy(right_lane_lidar_points)

        # lane_data = rospy.wait_for_message('lane_data_publisher', String)
        try:
            lane_data = rospy.wait_for_message('lane_data_publisher', String, timeout=0.5)
        except rospy.ROSException:
            if rospy.is_shutdown():
                break
            continue
        lanes_xys = json.loads(lane_data.data)
        lanes_xys = [[(x, y - int(CUT_ROAD_RATIO * IMAGE_HEIGHT)) for x, y in lane] for lane in lanes_xys]

        ret_val = util.get_current_lane_lidar_points(lanes_xys, M, virtual_lane_width=road_width)
        if ret_val is None:
            if left_lane_lidar_points is None or right_lane_lidar_points is None:
                continue
        else:
            left_lane_lidar_points, right_lane_lidar_points = ret_val


        # 특수 처리
        if section == 1 or section == 20:  # 출발점 근처에서 주차선을 현재 차선으로 보는 경우 방지
            if left_lane_lidar_points is not None and previous_left_lane_lidar_points is not None:
                current_slope = util.fit_polynomial(left_lane_lidar_points, 1)[0]
                previous_slope = util.fit_polynomial(previous_left_lane_lidar_points, 1)[0]

                current_degree = math.degrees(math.atan(current_slope))
                previous_degree = math.degrees(math.atan(previous_slope))
                
                if abs(current_degree - previous_degree) > 20:
                    left_lane_lidar_points = copy.deepcopy(previous_left_lane_lidar_points)

        if section == 14 and not parking_semi_started:  # Parking 시작 전, 왼쪽에 붙어가게
            right_lane_lidar_points = util.make_virtual_lane('right', left_lane_lidar_points, 4.0)
        if section == 14 and parking_semi_started:  # Parking 완료 후, 왼쪽 차선만 보고 주행
            right_lane_lidar_points = util.make_virtual_lane('right', left_lane_lidar_points, 4.0)

        if section == 15 and not parking_final_started:  # Parking 시작 전, 오른쪽에 붙어가게
            left_lane_lidar_points = util.make_virtual_lane('left', right_lane_lidar_points, 2.8)
        if section == 15 and parking_final_started:  # Parking 완료 후, 오른쪽 차선만 보고 주행
            left_lane_lidar_points = util.make_virtual_lane('left', right_lane_lidar_points, 3.5)

        if section == 1 or section == 20:  # 출발점 근처, 왼쪽 차선만 보고 주행
            right_lane_lidar_points = util.make_virtual_lane('right', left_lane_lidar_points, 4.0)

        angle, waypoint_lidar = calculate_angle(left_lane_lidar_points, right_lane_lidar_points)
        # angle += 0.35  # 휠 얼라인먼트 보정

        if section == 1 or section == 14 or section == 15:  # Lane Section + Parking Section
            steering_angle_pub.publish(angle)

        else:  # Mission Section, Speed Decision and Publish
            if section == 8:
                base_vel = 10
            elif section == 9:
                base_vel = 10
            else:
                base_vel = 10

            if section == 9:
                stop_state = stop_sign_state
            else:
                stop_state = STATE_CLEAR

            if stop_state == STATE_STOP:
                commanded_vel = STOP_VEL
            elif stop_state == STATE_SLOW_NEAR:
                commanded_vel = min(base_vel, SLOW_NEAR_VEL)
            elif stop_state == STATE_SLOW_FAR:
                commanded_vel = min(base_vel, SLOW_FAR_VEL)
            else:  # CLEAR or RESUME
                commanded_vel = base_vel

            vel_pub.publish(commanded_vel)
            deg_pub.publish(angle)

        print('angle :', angle)

        # Waypoint Visualization
        waypoint_vis_pt = [pt for i, pt in enumerate(waypoint_lidar) if i%10 == 0]
        ob = util.visualization_marker_array(waypoint_vis_pt, (0,255,0), 0.1, 0.1, 0.1)
        waypoint_vis_pub.publish(ob)

        # Lane Visualization
        lane_vis_points = [pt for i, pt in enumerate(left_lane_lidar_points + right_lane_lidar_points) if i%10 == 0]
        ob = util.visualization_marker_array(lane_vis_points, (255,255,255), 0.1, 0.1, 0.1)
        lane_vis_pub.publish(ob)
