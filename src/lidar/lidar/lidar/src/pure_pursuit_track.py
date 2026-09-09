#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################### 트랙 주행 중 조향각 계산을 위한 라이다 센서 기반 Pure Pursuit 노드 ###################################

import numpy as np
import math
import rospy

from std_msgs.msg import Float32, Int8
from visualization_msgs.msg import Marker
from mission.msg import PathInfo
from erp42_msgs.msg import DriveCmd, ModeCmd, SerialFeedBack
from geometry_msgs.msg import PoseStamped
from ublox_msgs.msg import NavPVT

# Parameters
k = 0.22  # look forward gain (with gpsVel)
Ldc = 3.0  # 최적화 Parameter
WB = 1.04  # [m] wheel base of vehicle
# MAX_SPEED = 12   
# MIN_SPEED = 4   
MAX_SPEED = 6  
MIN_SPEED = 5  
di_count = 0

# 차량이 따라 가야 할 경로 좌표들이 들어갈 리스트
path_x = []
path_y = []
state_h = 1
current_v = 0.0
cog = 0.0  # 차의 진행 방향 (C++ 코드에서 사용됨)

def feedback_cb(msg):
    global current_v
    current_v = msg.speed

def navpvt_callback(msg):
    global current_v, cog
    #current_v = msg.gSpeed * 0.0036  # mm/s 단위를 km/h로 변환
    cog = msg.heading * 1e-5
    cog = cog * math.pi / 180.0

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

    def search_target_index(self, state, di):
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

        if abs(di) >= 25:
            if current_v >= 10:
                Ld = 3.2
            elif current_v >=6:
                Ld = 2.7
            else:
                Ld = 2.5
        else:
            if current_v >= 10:
                Ld = 4.2
            elif current_v >=6:
                Ld = 3.5
            else:
                Ld = + Ldc

        print('Ld :', Ld)

        # search look ahead target point index
        while Ld > state.calc_distance(self.cx[ind], self.cy[ind]):  # look-ahead 거리 > 현재 지점과 다음 타켓 지점 사이의 거리보다 크면 다음 지점으로
            if (ind + 1) >= len(self.cx):  # 경로의 끝에 도달하면
                break  # not exceed goal
            ind += 1

        return ind, Ld

def pure_pursuit_steer_control(state, trajectory, pind, di):  # pind는 이전에 선택한 가장 가까운 지점의 인덱스
    ind, Ld = trajectory.search_target_index(state, di)

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
    delta = math.atan2(2.0 * WB * math.sin(alpha), Ld) * -100

    return delta, ind, tx, ty

def state_callback(data):
    global state_h
    state_h = data.data

def tunnel_path_callback(data):
    global path_x, path_y
    path_x = data.x
    path_y = data.y

###### 속도 계산 방식 최적화 해야됨 #########
def Deg2KPH(Deg, V_max):
    theta_max = 28.169  # 최대 조향각 (도)

    # 방법 1 : 비선형 속도 계산 (제곱 함수 적용) -> 완만한 속도 변화
    # deg_speed = V_min + (V_max - V_min) * (1 - math.pow(abs(Deg) / theta_max, 2))

    # 방법 2 : 지수 함수 속도 계산 -> 더 급격하게 속도 변화
    deg_speed = MIN_SPEED + (V_max - MIN_SPEED) * math.exp(-2 * (abs(Deg) / theta_max))

    return deg_speed

############ 속도 조절 최적화 해야됨 ################
def brake_di_decision(di):
    global current_v, di_count

    # v = 12                 # 방법 1 : 일정 속도

    v = Deg2KPH(di, MAX_SPEED)      # 방법 2 : 조향각에 따른 속도 조절 함수 사용

    speed_error = current_v - v

    Kp_brake = 20       # 최적화 parameter : brake에 실제로 곱해지는 비례 상수

    # di >= 28 횟수에 따른 속도 조절
    if abs(di) >= 28:
        di_count += 1
        if di_count >= 5:
            v = MIN_SPEED
            Kp_brake = 25
    else:
        di_count = 0  # 조건이 충족되지 않으면 카운터를 초기화
        if speed_error >= 4:
            Kp_brake = 25
        else:
            Kp_brake = 20

    brake = speed_error * Kp_brake

    if brake < 0:  # 브레이크 값이 음수일 경우 0으로 처리
        brake = 0

    if current_v < MIN_SPEED:
        v = 25
    print(v)
    v = int(v)
    return v, di, brake

def publish_Ctl(di, v, brake):
    if brake < 0:  # 브레이크 값이 음수일 경우 0으로 설정
        brake = 0

    # Create a driveCMD message
    drive_msg = DriveCmd()
    drive_msg.KPH = v
    """ 251028 1717 original code : switch speed to non fixed
    drive_msg.KPH = int(5)# v로 수정 필요
    """
    drive_msg.Deg = int(-di)
    # drive_msg.Deg = int(-28)
    drive_msg.brake = int(brake)

    
    # Print the actual values being published
    print('KPH:', drive_msg.KPH)
    print('Deg:', drive_msg.Deg)
    print('Brake:', drive_msg.brake)
    # Publish the driveCMD messages
    drive_pub.publish(drive_msg)

    mode = ModeCmd()
    mode.EStop = 0x00
    mode.Gear = 0x00
    mode.MorA = 0x01

    mode_pub.publish(mode)

    

if __name__ == '__main__':
    rospy.init_node("pure_pursuit_lidar", anonymous=True)

    rospy.Subscriber("/local_path", PathInfo, tunnel_path_callback)
    rospy.Subscriber("/state_ob", Int8, state_callback)
    rospy.Subscriber("/ublox_position_receiver/navpvt", NavPVT, navpvt_callback)
    rospy.Subscriber("/erp42_serial/feedback", SerialFeedBack, feedback_cb, queue_size=50)
    # rospy.Subscriber("/gps_speed", Float32, feedback_callback)  # 제거됨

    drive_pub = rospy.Publisher("/erp42_serial/drive", DriveCmd, queue_size=1)
    mode_pub = rospy.Publisher("/erp42_serial/mode", ModeCmd,  queue_size=1)

    # visualization goal point
    goal_point_pub = rospy.Publisher("/target_point", Marker, queue_size=1)
    rate = rospy.Rate(60)

    di = 0
    # initial state
    while not rospy.is_shutdown():
        if state_h == 0:  # state_h = 0이면 긴급제동
            publish_Ctl(0, 0, 200)
        else:
            if len(path_x) == 0:
                print('No Path')
            else:
                path_x = [x for x in path_x if x != 0.0]
                path_y = [y for y in path_y if y != 0.0]

                state = State(x=-1.1, y=0.0, yaw=0.0, v=5)
                # Calc control input
                target_course = TargetCourse(path_x, path_y)
                target_ind, _ = target_course.search_target_index(state, di)
                if target_ind is None:
                    continue  # 다음 루프로 건너뛰기

                try:  # 중점 안찍힐때 발생하는 오류 임시 조치
                    target_ind, _ = target_course.search_target_index(state, di)
                except ValueError:
                    continue

                di, target_ind, target_x, target_y = pure_pursuit_steer_control(state, target_course, target_ind, di)  #######조향각 도출

                target = Marker()
                target.header.frame_id = "velodyne"
                target.header.stamp = rospy.Time()
                target.id = 1
                target.type = 21.0
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
                # di -= 0.35  #-는 왼쪽틀기 , +는 오른쪽 틀기

                di = min(26, max(-26, di))  # 조향각 범위 제한

                v, di, brake = brake_di_decision(di)  # 곡률에 따른 속도 조절

                # Print the calculated velocity
                '''print('Calculated values:')
                print('velocity: ', v)
                print('Deg: ', di)
                print('brake: ', brake)'''
                publish_Ctl(di, v, brake)
                
                #print(target)
                goal_point_pub.publish(target)
                

                # 현재 GPS 속도 출력
                print('current speed:', current_v)

        rate.sleep()

""" 251028 1659 original code (wtf is state ob)
                if state_h == 1:
                    publish_Ctl(di, v, brake)"""