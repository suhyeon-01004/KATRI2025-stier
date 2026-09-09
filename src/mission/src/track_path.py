#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from mission.msg import PathInfo

import numpy as np
import cv2
from ultralytics import YOLO
from cv_bridge import CvBridge
from object_detector.msg import ObjectInfo
import time
import copy
import json
from scipy.spatial import Delaunay
import scipy
import math
from pathlib import Path

import sys
sys.path.append('{}/catkin_ws/src/mission/src'.format(Path.home()))
from mission_utils import *


class Cone:
    def __init__(self, x, y, color):
        self.x = x
        self.y = y
        self.color = color
        self.pixel_x = None
        self.pixel_y = None
        self.count = 10

    def get_point(self):
        return (self.x, self.y)

    def set_point(self, x, y):
        self.x = x
        self.y = y

    def decrease_count(self):
        self.count -= 1


# 정면 이미지 좌표를 받아 라바콘의 라이다 좌표 계산
def convert_vision2lidar(blue_cone_pixels, yellow_cone_pixels, M):
    blue_lidar_coords = list()
    yellow_lidar_coords = list()
    
    transformed_blue_x_list = list()
    transformed_blue_y_list = list()
    transformed_yellow_x_list = list()
    transformed_yellow_y_list = list()
    

    blue_cone_pixels = np.array(blue_cone_pixels, dtype=np.float32)
    yellow_cone_pixels = np.array(yellow_cone_pixels, dtype=np.float32)

    for blue_cone_pixel in blue_cone_pixels:
        blue_cone_pixel = blue_cone_pixel.reshape(-1, 1, 2)
        transformed_blue_point = cv2.perspectiveTransform(blue_cone_pixel, M).reshape(-1, 2)[0]
        transformed_blue_x_list.append(transformed_blue_point[0])
        transformed_blue_y_list.append(-(transformed_blue_point[1]-WARP_SIZE))

    for yellow_cone_pixel in yellow_cone_pixels:
        yellow_cone_pixel = yellow_cone_pixel.reshape(-1, 1, 2)
        transformed_yellow_point = cv2.perspectiveTransform(yellow_cone_pixel, M).reshape(-1, 2)[0]
        transformed_yellow_x_list.append(transformed_yellow_point[0])
        transformed_yellow_y_list.append(-(transformed_yellow_point[1]-WARP_SIZE))

    blue_lidar_coords = convert_point_bevcam2lidar(transformed_blue_x_list, transformed_blue_y_list)
    yellow_lidar_coords = convert_point_bevcam2lidar(transformed_yellow_x_list, transformed_yellow_y_list)
    
    return blue_lidar_coords, yellow_lidar_coords


def cone_predict_callback(data):
    global predict_result
    predict_result = json.loads(data.data)

def callback_obstacle(msg):
    global obstacles

    obstacle_list = []

    cx = msg.centerX
    cy = msg.centerY
    #lenx = msg.lengthX
    #leny = msg.lengthY
    #print(lenx)

    for i in range(msg.objectCounts):
    	#if cx[i] >= 0.3:
        center_x = cx[i]
        center_y = cy[i]

        obstacle_list.append((center_x, center_y))

    obstacles = obstacle_list

# 들로네 삼각분할과 경로 생성을 수행하는 함수
def making_path(blue_cones_center, yellow_cones_center):
    # 들로네 삼각분할을 수행하는 내부 함수
    def triangulation(ob):
        coordinate = [(a.x, a.y) for a in ob]
        points = np.array(coordinate)
        center_x = []
        center_y = []

        if len(points) < 3:
            return center_x, center_y, []

        try:
            tri = Delaunay(points)
            idx = tri.simplices

            for i in idx:
                for j in range(3):
                    if ob[i[j]].color != ob[i[j-1]].color:
                        center_x.append((ob[i[j]].x + ob[i[j-1]].x) / 2)
                        center_y.append((ob[i[j]].y + ob[i[j-1]].y) / 2)

        except scipy.spatial.qhull.QhullError:
            rospy.logwarn("Delaunay triangulation failed, skipping this set of points.")
            # 에러가 발생했을 때 빈 리스트를 반환합니다.
            return center_x, center_y, []

        return center_x, center_y, idx

    def to_track_msg(x_list, y_list):
        path_msg = PathInfo()
        path_msg.cnt = len(y_list)
        path_msg.x = x_list
        path_msg.y = y_list
        return path_msg

    cone_data = blue_cones_center + yellow_cones_center
    local_path = PathInfo()

    if len(cone_data) > 2:
        center_x, center_y, idx = triangulation(cone_data)
        center = list(zip(center_x, center_y))

        if center_x and center_y:
            sorted_indices = np.argsort(center_x)
            s_cx = np.array(center_x)[sorted_indices]
            s_cy = np.array(center_y)[sorted_indices]

            sorted_cx = s_cx.tolist()
            sorted_cy = s_cy.tolist()

            center_x = sorted_cx
            center_y = sorted_cy
        else:
            center_x = [0.0]
            center_y = [0.0]
    elif len(cone_data) == 2 and cone_data[0].color != cone_data[1].color:
        a = cone_data[0]
        b = cone_data[1]
        center_x = [(a.x + b.x) / 2]
        center_y = [(a.y + b.y) / 2]
    else:
        center_x = [0.0]
        center_y = [0.0]

    linspace_num = 20
    x_new = [0.0 for _ in range(linspace_num)]
    y_new = [0.0 for _ in range(linspace_num)]
    for j in range(len(center_x)):
        if j > 19:
            break
        x_new[j] = center_x[j]
        y_new[j] = center_y[j]

    local_path = to_track_msg(x_new, y_new)
    return local_path, x_new, y_new, center_x, center_y



def calculate_previous_cones_first():
    global obstacles

    while not rospy.is_shutdown():
        if len(obstacles) > 0:
            break

    bridge = CvBridge()
    image_msg = rospy.wait_for_message("/usb_cam2/image_raw", Image)
    frame = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    
    result = model.predict(frame)[0]
    boxes = result.boxes.xyxy
    confs = [int(conf * 100) for conf in result.boxes.conf]

    blue_cone_pixels = list()
    yellow_cone_pixels = list()
    for i, box in enumerate(boxes):
        if confs[i] < 70:
            continue

        box = [int(pt) for pt in box]
        start = ((box[0]), box[1])
        end = (box[2], box[3])
        coord_x = (start[0] + end[0]) / 2
        coord_y = end[1]

        cone_pixels = frame[int(0.4 * box[1] + 0.6 * box[3]):int(0.1 * box[1] + 0.9 * box[3]),
                            int(0.6 * box[0] + 0.4 * box[2]):int(0.4 * box[0] + 0.6 * box[2]), :]
        cone_pixels = cone_pixels.reshape(-1, 3)

        B_avg = sum(cone_pixels[:, 0]) / len(cone_pixels)
        G_avg = sum(cone_pixels[:, 1]) / len(cone_pixels)
        R_avg = sum(cone_pixels[:, 2]) / len(cone_pixels)
        blue_distance = (255 - B_avg) ** 2 + G_avg ** 2 + R_avg ** 2
        yellow_distance = B_avg ** 2 + (255 - G_avg) ** 2 + (255 - R_avg) ** 2

        if blue_distance < yellow_distance:  # Blue Cone
            blue_cone_pixels.append((int(coord_x), int(coord_y-CUT_ROAD_RATIO*IMAGE_HEIGHT)))
            color = (255, 0, 0)
            cv2.rectangle(frame, start, end, color=color, thickness=2)
            cv2.putText(frame, 'Blue', (box[0], box[3] + 15), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)
            cv2.putText(frame, '({}%)'.format(confs[i]), (box[0], box[3] + 40), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)
        else:  # Yellow Cone
            yellow_cone_pixels.append((int(coord_x), int(coord_y-CUT_ROAD_RATIO*IMAGE_HEIGHT)))
            color = (0, 255, 255)
            cv2.rectangle(frame, start, end, color=color, thickness=2)
            cv2.putText(frame, 'Yellow', (box[0], box[3] + 15), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)
            cv2.putText(frame, '({}%)'.format(confs[i]), (box[0], box[3] + 40), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)

    blue_lidar_est_coords, yellow_lidar_est_coords = convert_vision2lidar(blue_cone_pixels, yellow_cone_pixels, M)

    current_obstacles = copy.deepcopy(obstacles)
    current_obstacles = np.array(current_obstacles)

    blue_cones = list()
    yellow_cones = list()

    for est_coord in blue_lidar_est_coords:
        est_coord = np.array(est_coord)
        # print('current_obstacles :', current_obstacles)
        # print('est_coord :', est_coord)
        distance_list = np.linalg.norm(current_obstacles - est_coord, axis=1)
        distance_min_idx = np.argmin(distance_list)
        if distance_list[distance_min_idx] <= 1:
            selected_cone_coord = current_obstacles[distance_min_idx]
            blue_cones.append(Cone(selected_cone_coord[0], selected_cone_coord[1], 'blue'))

    if len(blue_cones) == 0:
        blue_cones.append(Cone(0, -1, 'blue'))

    for est_coord in yellow_lidar_est_coords:
        est_coord = np.array(est_coord)
        distance_list = np.linalg.norm(current_obstacles - est_coord, axis=1)
        distance_min_idx = np.argmin(distance_list)
        if distance_list[distance_min_idx] <= 1:
            selected_cone_coord = current_obstacles[distance_min_idx]
            yellow_cones.append(Cone(selected_cone_coord[0], selected_cone_coord[1], 'yellow'))

    if len(yellow_cones) == 0:
        yellow_cones.append(Cone(0, 1, 'yellow'))

    return blue_cones, yellow_cones

def temptemp(distance_list):
    if distance_list:
        if min(distance_list) < 0.8:
            return True

    return False


if __name__ == '__main__':
    rospy.init_node('track_path', anonymous=True)

    # ROS Publisher
    path_pub = rospy.Publisher("/local_path", PathInfo, queue_size=1)
    pub_cones_center = rospy.Publisher('/cones_center', Float32MultiArray, queue_size=1)
    blue_cone_vis_pub = rospy.Publisher('/blue_cone_vis_pub', MarkerArray, queue_size=1)
    yellow_cone_vis_pub = rospy.Publisher('/yellow_cone_vis_pub', MarkerArray, queue_size=1)
    blue_cone_vis_pure_pub = rospy.Publisher('/blue_cone_vis_pure_pub', MarkerArray, queue_size=1)
    yellow_cone_vis_pure_pub = rospy.Publisher('/yellow_cone_vis_pure_pub', MarkerArray, queue_size=1)
    waypoint_vis_pub = rospy.Publisher("/waypoint_vis_pub", MarkerArray, queue_size=1)

    # ROS Subscriber
    rospy.Subscriber('/object_info', ObjectInfo, callback_obstacle)
    rospy.Subscriber('cone_predict_pub', String, cone_predict_callback)
    
    # YOLO Cone Model
    model = YOLO('{}/catkin_ws/ModelFiles/RubberCone_YOLOv10m_1280.pt'.format(Path.home()))
    model.predict(np.zeros((720, 1280, 3)), verbose=False)

    src, dst, M = get_camera_bev_parameters()

    predict_result = None
    obstacles = list()

    previous_blue_cones, previous_yellow_cones = calculate_previous_cones_first()

    bridge = CvBridge()
    while not rospy.is_shutdown():
        image_msg = rospy.wait_for_message("/usb_cam2/image_raw", Image)
        frame = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        
        result = model.predict(frame)[0]
        boxes = result.boxes.xyxy
        classes = [int(pt) for pt in result.boxes.cls]
        confs = [int(conf * 100) for conf in result.boxes.conf]

        blue_cone_pixels = list()
        yellow_cone_pixels = list()
        for i, box in enumerate(boxes):
            if confs[i] < 70:
                continue

            box = [int(pt) for pt in box]
            start = (box[0], box[1])
            end = (box[2], box[3])
            coord_x = (start[0] + end[0]) / 2
            coord_y = end[1]

            cone_pixels = frame[int(0.4 * box[1] + 0.6 * box[3]):int(0.1 * box[1] + 0.9 * box[3]),
                                int(0.6 * box[0] + 0.4 * box[2]):int(0.4 * box[0] + 0.6 * box[2]), :]
            cone_pixels = cone_pixels.reshape(-1, 3)

            B_avg = sum(cone_pixels[:, 0]) / len(cone_pixels)
            G_avg = sum(cone_pixels[:, 1]) / len(cone_pixels)
            R_avg = sum(cone_pixels[:, 2]) / len(cone_pixels)
            blue_distance = (255 - B_avg) ** 2 + G_avg ** 2 + R_avg ** 2
            yellow_distance = B_avg ** 2 + (255 - G_avg) ** 2 + (255 - R_avg) ** 2

            if blue_distance < yellow_distance:  # Blue Cone
                blue_cone_pixels.append((int(coord_x), int(coord_y-CUT_ROAD_RATIO*IMAGE_HEIGHT)))
                color = (255, 0, 0)
                cv2.rectangle(frame, start, end, color=color, thickness=2)
                cv2.putText(frame, 'Blue', (box[0], box[3] + 15), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)
                cv2.putText(frame, '({}%)'.format(confs[i]), (box[0], box[3] + 40), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)
            else:  # Yellow Cone
                yellow_cone_pixels.append((int(coord_x), int(coord_y-CUT_ROAD_RATIO*IMAGE_HEIGHT)))
                color = (0, 255, 255)
                cv2.rectangle(frame, start, end, color=color, thickness=2)
                cv2.putText(frame, 'Yellow', (box[0], box[3] + 15), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)
                cv2.putText(frame, '({}%)'.format(confs[i]), (box[0], box[3] + 40), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)

        blue_lidar_est_coords, yellow_lidar_est_coords = convert_vision2lidar(blue_cone_pixels, yellow_cone_pixels, M)

        current_obstacles = copy.deepcopy(obstacles)
        current_obstacles = np.array(current_obstacles)

        blue_cones = list()
        yellow_cones = list()
        current_obstacles_delete_indexes = list()
        matched_indices = set()

        for est_coord in blue_lidar_est_coords:
            est_coord = np.array(est_coord)
            distance_list = np.linalg.norm(current_obstacles - est_coord, axis=1)
            distance_min_idx = np.argmin(distance_list)
            if distance_list[distance_min_idx] <= 1:
                selected_cone_coord = current_obstacles[distance_min_idx]
                current_obstacles_delete_indexes.append(distance_min_idx)
                blue_cones.append(Cone(selected_cone_coord[0], selected_cone_coord[1], 'blue'))
                matched_indices.add(distance_min_idx)  # 매칭된 인덱스 저장

        for est_coord in yellow_lidar_est_coords:
            est_coord = np.array(est_coord)
            distance_list = np.linalg.norm(current_obstacles - est_coord, axis=1)
            distance_min_idx = np.argmin(distance_list)
            if distance_list[distance_min_idx] <= 1:
                selected_cone_coord = current_obstacles[distance_min_idx]
                current_obstacles_delete_indexes.append(distance_min_idx)
                yellow_cones.append(Cone(selected_cone_coord[0], selected_cone_coord[1], 'yellow'))
                matched_indices.add(distance_min_idx)  # 매칭된 인덱스 저장

        current_obstacles_deleted = [pt for i, pt in enumerate(current_obstacles) if i not in current_obstacles_delete_indexes]

        blue_delete_indexes = list()
        for i, previous_cone in enumerate(previous_blue_cones):
            previous_cone_point = previous_cone.get_point()
            distance_list = [calculate_distance(previous_cone_point, cone.get_point()) for cone in blue_cones]
            if temptemp(distance_list):
                blue_delete_indexes.append(i)

        for i, previous_cone in enumerate(previous_blue_cones):
            if i not in blue_delete_indexes:
                distance_list = [calculate_distance(previous_cone.get_point(), pt) for pt in current_obstacles_deleted]
                if temptemp(distance_list):
                    closest_point = current_obstacles_deleted[np.argmin(distance_list)]
                    previous_cone.set_point(closest_point[0], closest_point[1])
                    blue_cones.append(previous_cone)
                else:
                    previous_cone.decrease_count()
                    if previous_cone.count > 0:
                        blue_cones.append(previous_cone)


        yellow_delete_indexes = list()
        for i, previous_cone in enumerate(previous_yellow_cones):
            previous_cone_point = previous_cone.get_point()
            distance_list = [calculate_distance(previous_cone_point, cone.get_point()) for cone in yellow_cones]
            if temptemp(distance_list):
                yellow_delete_indexes.append(i)

        for i, previous_cone in enumerate(previous_yellow_cones):
            if i not in yellow_delete_indexes:
                distance_list = [calculate_distance(previous_cone.get_point(), pt) for pt in current_obstacles_deleted]
                if temptemp(distance_list):
                    closest_point = current_obstacles_deleted[np.argmin(distance_list)]
                    previous_cone.set_point(closest_point[0], closest_point[1])
                    yellow_cones.append(previous_cone)
                else:
                    previous_cone.decrease_count()
                    if previous_cone.count > 0:
                        yellow_cones.append(previous_cone)

        if len(blue_cones) == 0:
            gray_coords = list()
            for idx, coord in enumerate(current_obstacles):
                if idx not in matched_indices:
                    if coord[0] < 2 and abs(coord[1]) < 3:  # 원하는 범위 내의 장애물 선택
                        gray_coords.append(coord)

            gray_cone_coords = list()
            for coord in gray_coords:
                gray_cone_coords.append(Cone(coord[0], coord[1], 'gray'))
            
            # 회색 콘을 처리하여 색상 재분류
            for cone in gray_cone_coords:
                blue_std_p = Cone(0.0, -1.0, 'blue')    # 파란색 기준점
                yellow_std_p = Cone(0.0, 1.0, 'yellow')  # 노란색 기준점

                distance_to_blue = math.sqrt((cone.x - blue_std_p.x) ** 2 + (cone.y - blue_std_p.y) ** 2)
                distance_to_yellow = math.sqrt((cone.x - yellow_std_p.x) ** 2 + (cone.y - yellow_std_p.y) ** 2)

                if distance_to_blue < distance_to_yellow:
                    cone.color = 'blue'
                    blue_cones.append(cone)

        if len(yellow_cones) == 0:
            gray_coords = list()
            for idx, coord in enumerate(current_obstacles):
                if idx not in matched_indices:
                    if coord[0] < 2 and abs(coord[1]) < 3:  # 원하는 범위 내의 장애물 선택
                        gray_coords.append(coord)

            gray_cone_coords = list()
            for coord in gray_coords:
                gray_cone_coords.append(Cone(coord[0], coord[1], 'gray'))
            
            # 회색 콘을 처리하여 색상 재분류
            for cone in gray_cone_coords:
                blue_std_p = Cone(0.0, -1.0, 'blue')    # 파란색 기준점
                yellow_std_p = Cone(0.0, 1.0, 'yellow')  # 노란색 기준점

                distance_to_blue = math.sqrt((cone.x - blue_std_p.x) ** 2 + (cone.y - blue_std_p.y) ** 2)
                distance_to_yellow = math.sqrt((cone.x - yellow_std_p.x) ** 2 + (cone.y - yellow_std_p.y) ** 2)

                if distance_to_blue > distance_to_yellow:
                    cone.color = 'yellow'
                    yellow_cones.append(cone)

        previous_blue_cones = copy.deepcopy(blue_cones)
        previous_yellow_cones = copy.deepcopy(yellow_cones)
                
        # 
        if len(blue_cones) == 0:
            blue_cones.append(Cone(0, -1, 'blue'))
        # elif len(blue_cones) > 0:
        #     distance_list = [calculate_distance((0,0), cone.get_point()) for cone in blue_cones]
        #     if min(distance_list) > 3:
        #         blue_cones.append(Cone(1, 1, 'blue'))

        if len(yellow_cones) == 0:
            yellow_cones.append(Cone(0, 1, 'yellow'))
        # elif len(yellow_cones) > 0:
        #     distance_list = [calculate_distance((0,0), cone.get_point()) for cone in yellow_cones]
        #     if min(distance_list) > 3:
        #         yellow_cones.append(Cone(1, -1, 'yellow'))
        
        local_path, path_x, path_y, _, _ = making_path(blue_cones, yellow_cones)

        waypoint = list()
        for i in range(len(path_x)):
            if path_x[i] != 0.0 and path_y[i] != 0.0:
                waypoint.append((path_x[i], path_y[i]))

        if len(waypoint) > 0:
            distance_list = [calculate_distance((0,0), pt) for pt in waypoint]
            if min(distance_list) > 2:
                blue_cones.append(Cone(0, -1, 'blue'))
                yellow_cones.append(Cone(0, 1, 'yellow'))

        path_pub.publish(local_path)



        
        # Visualization
        blue_coords = [cone.get_point() for cone in blue_cones]
        yellow_coords = [cone.get_point() for cone in yellow_cones]
        ob = visualization_marker_array(blue_coords, (0,0,255), 0.2, 0.2, 0.2, 'sphere', 0.1)
        blue_cone_vis_pub.publish(ob)
        ob = visualization_marker_array(yellow_coords, (255,255,0), 0.2, 0.2, 0.2, 'sphere', 0.1)
        yellow_cone_vis_pub.publish(ob)
        ob = visualization_marker_array(list(zip(path_x, path_y)), (0,255,0), 0.1, 0.1, 0.1)
        waypoint_vis_pub.publish(ob)
        
        ob = visualization_marker_array(blue_lidar_est_coords, (135,206,235), 0.2, 0.2, 0.2, 'cube', 0.3)
        blue_cone_vis_pure_pub.publish(ob)
        ob = visualization_marker_array(yellow_lidar_est_coords, (255,127,0), 0.2, 0.2, 0.2, 'cube', 0.1)
        yellow_cone_vis_pure_pub.publish(ob)

        frame = cv2.resize(frame, (640,360))
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit()
