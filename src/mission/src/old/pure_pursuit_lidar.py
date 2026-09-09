#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################### 트랙 주행 중 조향각 계산을 위한 라이다 센서 기반 Pure Pursuit 노드 ###################################

import numpy as np
import math
import rospy

from std_msgs.msg import Float32, Int8, Int32
from visualization_msgs.msg import Marker
from mission.msg import PathInfo
from pathlib import Path
from datetime import datetime

# Parameters
k = 0.22  # look forward gain (with gpsVel)
Ldc = 3.2   ##### 기본 Ld값 #####
Ldc_small = 2.7 
Ldc_big = 3.2
WB = 1.04  # [m] wheel base of vehicle

# 차량이 따라 가야 할 경로 좌표들이 들어갈 리스트
path_x = []
path_y = []
state_h = 1
section = 0   ######## section 변경 #########
cnt = 0
stop_received = False  # stop_sign 메시지가 수신되었는지 여부를 나타내는 플래그
current_v = 0.0

class State:
    def __init__(self, x=-1.1, y=0.0, yaw=0.0, v=0.0):
        self.yaw = yaw
        self.v = v
        self.rear_x = x
        self.rear_y = y

    # 차량으로부터 Target Waypoint까지의 거리 계산
    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)  # 거리 값을 리턴함

class TargetCourse:
    def __init__(self, cx, cy):
        self.cx = cx  # 경로의 x 및 y 좌표 나타내는 list
        self.cy = cy
        self.old_nearest_point_index = None  # 이전에 선택한 가장 가까운 지점의 인덱스 저장하는 변수

    def search_target_index(self, state):
        # To speed up nearest point search, doing it at only first time.  
        if self.old_nearest_point_index is None:  # 처음에는 None
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)

            if len(d) == 0:  # d 배열이 비어 있는지 확인
                rospy.logwarn("No points in the path. Can't determine target index.")
                return None, None

            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        # 이전에 선택한 가장 가까운 지점을 기준으로 다음 타겟 지점을 찾는 과정
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind], self.cy[ind])  # 현재 지점과의 거리
            while True:
                try:
                    distance_next_index = state.calc_distance(self.cx[ind + 1], self.cy[ind + 1])  # 다음 지점과의 거리
                except IndexError:
                    break

                if distance_this_index < distance_next_index + 1:
                    break

                if (ind + 1) < len(self.cx):
                    ind = ind + 1
                else:
                    ind = ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Ld = ld_value

        print('Ld :', Ld)

        # search look ahead target point index
        while Ld > state.calc_distance(self.cx[ind], self.cy[ind]):  # look-ahead 거리 > 현재 지점과 다음 타켓 지점 사이의 거리보다 크면 다음 지점으로
            if (ind + 1) >= len(self.cx):  # 경로의 끝에 도달하면
                break  # not exceed goal
            ind += 1

        return ind, Ld

def pure_pursuit_steer_control(state, trajectory, pind):  # pind는 이전에 선택한 가장 가까운 지점의 인덱스
    ind, Ld = trajectory.search_target_index(state)

    if pind >= ind:  # 특정 지점 이후에 경로를 따라가도록 하기 위함
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal (이미 경로의 끝에 도달했다는 것)
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x)

    # 조향각 계산
    delta = math.atan2(2.0 * WB * math.sin(alpha), Ld) * -100  # * 1.1

    return delta, ind, tx, ty

def state_callback(data):
    global state_h
    state_h = data.data

def section_callback(data):
    global section
    section = data.data

def PathInfo_callback(data):
    global path_x, path_y, section

    path_x = data.x
    path_y = data.y

def ld_callback(data):
    global ld_value
    ld_value = data.data


def Deg2KPH(Deg, V_max):
    V_min = 5  # 최소 속도 (km/h)
    theta_max = 28.169  # 최대 조향각 (도)

    # 비선형 속도 계산 (제곱 함수 적용)
    #deg_speed = V_min + (V_max - V_min) * (1 - math.pow(abs(Deg) / theta_max, 2))

    #지수 함수 속도 계산
    deg_speed = V_min + (V_max - V_min) * math.exp(-2 * (abs(Deg) / theta_max))

    return deg_speed
    

# def publish_Ctl(di):

#     steer_msg = Float32()
#     steer_msg.data = di
#     steer_pub.publish(steer_msg)
#     print("Publishing to /lidarDeg:", steer_msg.data)


if __name__ == '__main__':
    rospy.init_node("pure_pursuit_lidar_final", anonymous=True)

    rospy.Subscriber("/local_path", PathInfo, PathInfo_callback)
    rospy.Subscriber("/state_ob", Int8, state_callback)
    rospy.Subscriber("/section", Int32, section_callback)
    rospy.Subscriber('/lidar_pp_ld', Int32, ld_callback)

    steer_pub = rospy.Publisher("/lidarDeg", Float32, queue_size=1)

    # visualization goal point
    goal_point_pub = rospy.Publisher("/target_point", Marker, queue_size=1)
    rate = rospy.Rate(60)
    
    ld_value = 0

    # initial state
    while not rospy.is_shutdown():
        if stop_received:
            print("Stop sign received, Full Brake!!!")
            stop_received = False  # 플래그를 False로 리셋
            rospy.sleep(1)
            continue

        if state_h == 0: # state_h = 0이면 긴급제동
            pass
        else:
            if len(path_x) == 0:
                print('No Path')
            else:
                if section == 8:
                    path_x = [x for x in path_x if x != 0.0]
                    path_y = [y for y in path_y if y != 0.0]
                state = State(x=-1.1, y=0.0, yaw=0.0, v=5)
                # Calc control input
                target_course = TargetCourse(path_x, path_y)
                target_ind, _ = target_course.search_target_index(state)
                if target_ind is None:
                    continue  # 다음 루프로 건너뛰기

                try:  # 중점 안찍힐때 발생하는 오류 임시 조치
                    target_ind, _ = target_course.search_target_index(state)
                except ValueError:
                    continue

                di, target_ind, target_x, target_y = pure_pursuit_steer_control(state, target_course, target_ind)  #######조향각 도출

                target = Marker()
                target.header.frame_id = "velodyne"
                target.header.stamp = rospy.Time()
                target.id = 1
                target.type = 2
                target.pose.position.x = target_x
                target.pose.position.y = target_y
                target.pose.position.z = 1.0
                target.scale.x = 0.2
                target.scale.y = 0.2
                target.scale.z = 0.2
                target.color.a = 1.0
                target.color.r = 1.0
                target.color.g = 0
                target.color.b = 1.0
                target.lifetime = rospy.Duration(0.3)  ########target point vis

                di = di * 1.3

                di_q = 0

                while cnt < 3:
                    di_q = di_q + di
                    cnt = cnt + 1

                cnt = 0
                di = di_q / 3

                #di -= 3.5
                di = -min(28, max(-28, di))  # 조향각 범위 제한

                #publish_Ctl(di)
                steer_pub.publish(di)
                print("Publishing to /lidarDeg:", di)
                
                goal_point_pub.publish(target)
                

        rate.sleep()