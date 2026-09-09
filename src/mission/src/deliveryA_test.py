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

LANE_TIME = 2.0
STOP_TIME = 5
STOP_DISTANCE = 1.2 - 0.6 #251030 tuned
ROAD_WIDTH = 4.0  # kcity

# 전역 퍼블리셔 (카메라에서 얻은 사인 점의 라이다 좌표 투영지점을 퍼블리시)
sign_projection_vis_pub = None

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

def publish_sign_projection_marker(sign_point):
    """
    카메라에서 인식한 사인의 라이다 좌표계 위치를 마커로 퍼블리시.
    obstacle_vis_pub 과 비슷하게 동작하도록 함.
    """
    global sign_projection_vis_pub
    if sign_projection_vis_pub is None or sign_point is None:
        return

    marker = Marker()
    marker.header.frame_id = "velodyne"  # 라이다 좌표계에 맞게 설정 (기존 obstacle 마커와 동일하게 쓴다는 가정)
    marker.header.stamp = rospy.Time.now()
    marker.ns = "sign_projection"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = sign_point[0]
    marker.pose.position.y = sign_point[1]
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0  # 파란색으로 표시

    sign_projection_vis_pub.publish(marker)



"""
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
                print(text)
                
                start = box[0]
                end = box[2]

                # OCR ByeongK Yebang

                if text == 'A7' or text == 'AT':
                    text = 'A1'

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

            # 1031 DEBUG VISUALIZE IMAGE POINTS
            print("obstacles_temp")
            print(obstacles_temp)
            img_points_vis = img_points.reshape(-1, 2)
            print("img_points_vis")
            print(img_points_vis)
            points_to_pub = util.visualization_marker_array(img_points_vis, (0,0,255), 0.2, 0.2, 0.2)
            imagepoint_vis_pub.publish(points_to_pub)
            
            
            distance_list = [util.calculate_distance(image_point, pt[0]) for pt in img_points]
            distance_min_idx = np.argmin(distance_list)
            sign_point = obstacles_temp[distance_min_idx][:2]

            if sign_point[0] < 1.5:
                sign_point = None

    else:
        obstacles_temp = obstacles_temp[:, :2]
        distance_list = [util.calculate_distance(previous_sign_point, pt) for pt in obstacles_temp]
        min_idx = np.argmin(distance_list)
        if min(distance_list) < 1:
            sign_point = obstacles_temp[min_idx]
        else:
            sign_point = previous_sign_point

    # 여기서도 퍼블리시 (계산된 시점마다)
    if sign_point is not None:
        publish_sign_projection_marker(sign_point)

    # display camera view
    frame = cv2.resize(frame, (640,360))
    cv2.imshow('frame', frame)
    if cv2.waitKey(25) & 0xFF == ord('q'):
        cv2.destroyAllWindows()

    return sign_point
"""

def draw_points_on_frame(frame, img_points, color=(0,0,255), radius=5, thickness=-1):
    """
    img_points: (N, 2) 형태의 이미지 좌표들
    color: BGR
    """
    if img_points is None:
        return
    for pt in img_points:
        x, y = int(pt[0]), int(pt[1])
        cv2.circle(frame, (x, y), radius, color, thickness)


def project_world_points_to_image(world_points, rvec, tvec, cameraMatrix, dist=None):
    """
    world_points: (N,3) or (N,2) -> (N,1,3)로 맞춰서 projectPoints
    return: (N,2) 이미지 좌표
    """
    if dist is None:
        dist = np.array([0,0,0,0], dtype=float)

    world_points = np.array(world_points, dtype=float)
    if world_points.ndim == 2 and world_points.shape[1] == 2:
        # z = 0으로 가정
        zeros = np.zeros((world_points.shape[0], 1), dtype=float)
        world_points = np.hstack([world_points, zeros])

    img_pts, _ = cv2.projectPoints(world_points, rvec, tvec, cameraMatrix, dist)
    img_pts = img_pts.reshape(-1, 2)
    return img_pts


def calculate_sign_point(frame, previous_sign_point):
    global obstacles, target_detected, rvec, tvec, cameraMatrix
    obstacles_temp = np.array(copy.deepcopy(obstacles))

    sign_point = None
    # 표시용 프레임은 원본을 복사해서 그리자
    draw_frame = frame.copy()

    if previous_sign_point is None:

        image_point = None
        result = ocr.ocr(frame, cls=True)[0]
        if result is not None:
            for line in result:
                box = line[0]  # 텍스트의 바운딩 박스 좌표
                text = line[1][0].strip()  # 인식된 텍스트
                score = line[1][1]  # 인식 신뢰도
                print(text)
                
                start = box[0]
                end = box[2]

                # OCR ByeongK Yebang
                if text == 'A7' or text == 'AT':
                    text = 'A1'

                if score > 0.6 and (text=='A1' or text=='A2' or text=='A3'):
                    image_point = ((start[0]+end[0])/2, (start[1]+end[1])/2)

                    if not target_detected:
                        target_detected = True
                        target_sign = 'B' + text[1]
                        f = open('{}/catkin_ws/src/mission/msg/delivery_target_sign.txt'.format(Path.home()), 'w')
                        f.write(target_sign)
                        f.close()

        if image_point is not None and obstacles_temp.size > 0:
            # 1) 장애물 전체를 이미지로 투영
            img_points, _ = cv2.projectPoints(
                obstacles_temp, rvec, tvec, cameraMatrix,
                np.array([0, 0, 0, 0], dtype=float))

            # 디버깅 출력
            print("image_point")
            print(image_point)
            print("obstacles_temp")
            print(obstacles_temp)
            img_points_vis = img_points.reshape(-1, 2)
            print("img_points_vis")
            print(img_points_vis)

            # RViz용 퍼블리시(기존)
            points_to_pub = util.visualization_marker_array(img_points_vis, (0,255,0), 0.2, 0.2, 0.2)
            imagepoint_vis_pub.publish(points_to_pub)

            # (A) 화면에도 그리기: 모든 image_points → 빨강
            draw_points_on_frame(draw_frame, img_points_vis, color=(255,0,0), radius=5, thickness=-1)

            # (B) OCR로 찾은 image_point → 파랑
            cv2.circle(draw_frame, (int(image_point[0]), int(image_point[1])), 7, (0,0,255), -1)

            # 가장 가까운 장애물 찾기
            distance_list = [util.calculate_distance(image_point, pt[0]) for pt in img_points]
            distance_min_idx = np.argmin(distance_list)
            sign_point = obstacles_temp[distance_min_idx][:2]  # (x, y)만

            # x가 너무 가까우면 무시
            if sign_point[0] < 1.5:
                sign_point = None

            # (C) 주황색 찍기용: obstacles_temp 중 x 2~3, y 0~-1
            # world 좌표 필터링
            mask = (obstacles_temp[:,0] >= 2.0) & (obstacles_temp[:,0] <= 3.0) & \
                   (obstacles_temp[:,1] <= 0.0) & (obstacles_temp[:,1] >= -1.0)
            filtered_world = obstacles_temp[mask]  # (M,3) 또는 (M,2)

            if filtered_world.size > 0:
                filtered_img = project_world_points_to_image(filtered_world, rvec, tvec, cameraMatrix)
                # 주황색 (BGR: 0,165,255)
                draw_points_on_frame(draw_frame, filtered_img, color=(0,165,255), radius=6, thickness=-1)

    else:
        # 이전 sign_point 기준으로 다시 선택
        obstacles_temp = obstacles_temp[:, :2]
        distance_list = [util.calculate_distance(previous_sign_point, pt) for pt in obstacles_temp]
        min_idx = np.argmin(distance_list)
        if min(distance_list) < 1:
            sign_point = obstacles_temp[min_idx]
        else:
            sign_point = previous_sign_point

        # 이 경우에도 화면에 기존 장애물들 찍어줄 수 있음 (optional)
        # world -> image
        if obstacles_temp.size > 0:
            obs2d_img = project_world_points_to_image(obstacles_temp, rvec, tvec, cameraMatrix)
            draw_points_on_frame(draw_frame, obs2d_img, color=(0,0,255), radius=6, thickness=-1)

    # 여기서도 퍼블리시 (계산된 시점마다)
    if sign_point is not None:
        publish_sign_projection_marker(sign_point)
        # sign_point가 실제로 어디였는지 이미지에도 표시 (초록)
        sp_img = project_world_points_to_image([sign_point], rvec, tvec, cameraMatrix)
        draw_points_on_frame(draw_frame, sp_img, color=(0,0,0), radius=3, thickness=-1)

    # display camera view (디버깅 그려진 버전으로)
    #draw_frame = cv2.resize(draw_frame, (640,360))
    cv2.imshow('frame', draw_frame)
    if cv2.waitKey(25) & 0xFF == ord('q'):
        cv2.destroyAllWindows()

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
    imagepoint_vis_pub = rospy.Publisher('/imagepoint_vis_pub', MarkerArray, queue_size=1)
    # 카메라 사인 -> 라이다 투영 마커 퍼블리셔
    sign_projection_vis_pub = rospy.Publisher('/sign_projection_vis_pub', Marker, queue_size=1)

    # ROS Subscriber
    rospy.Subscriber('/object_info', ObjectInfo, callback_obstacle)
    rospy.Subscriber("/section", Int32, callback_section)
    rospy.Subscriber('/lidarDeg', Float32, callback_deg)

    # Camera and LIDAR parameters
    rvec = np.array([
        [-0.00901827, -0.99986437,  0.0137809 ],
        [ 0.03088284, -0.01405338, -0.99942421],
        [ 0.99948233, -0.00858748,  0.03100539]
    ])
    tvec = np.array([
        [-0.02671437],
        [ 0.19031678],
        [ 0.6092148 ]])
    cameraMatrix = np.array([
    [916.509137,     0.,         660.08973047],
    [  0.,         917.33875912, 335.60822492],
    [  0.,           0.        ,   1.        ]])

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
    stop_flag = False

    if args.section:
        while True:
            if section == 20:
                print('delivery A section activatd')
                break
            else:
                print('not delivery A section')
                time.sleep(0.1)
    
        while True:  # auto 키면 시작
            feedback = rospy.wait_for_message('/erp42_serial/feedback', SerialFeedBack)
            if feedback.MorA == 1:
                break
            else:
                print("not in auto mode")
                time.sleep(0.1)
    

    print('started!!')
    mission_lane_control_pub.publish(1)
    lane_control()
    mission_lane_control_pub.publish(0)
    print('lane finish!')

    start_time = time.time()
    print('first criteria lane found')
    while not rospy.is_shutdown():
        image_msg = rospy.wait_for_message('/usb_cam1/image_raw', Image)
        frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        # Calculate Sign Point
        sign_point = calculate_sign_point(frame, sign_point)
        
        if sign_point is not None and sign_point[0] <= STOP_DISTANCE:
            print('sign_point :', sign_point, "[STOP]")
            """
            print('stop!!')
            delivery_finished = True
            stop_flag = True
            stop_start_time = time.time()
            while True:
                if time.time() - stop_start_time > STOP_TIME:
                    vel_pub.publish(0)
                    break
                else:
                    vel_pub.publish(0)
                    time.sleep(0.1)
                    continue
            stop_flag = False
            delivery_finish_time = time.time()
            continue
            """

        else:
            print('sign_point :', sign_point)
            vel = 5
            ld_value = 7
        
        if sign_point is not None:
            ob = util.visualization_marker(sign_point, (255,0,0), 0.2, 0.2, 0.2)
            obstacle_vis_pub.publish(ob)
            # 추가: 카메라-라이다 투영 마커도 같이 퍼블리시
            publish_sign_projection_marker(sign_point)
        
        