#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Tunnel Section 9

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32, Float32, Bool
from mission.msg import PathInfo
from object_detector.msg import ObjectInfo
from cv_bridge import CvBridge
from ublox_msgs.msg import NavPVT

from scipy.spatial import Delaunay
from ultralytics import YOLO
from pathlib import Path
import numpy as np
import argparse
import scipy
import time
import math
import json
import copy
import cv2

import sys
sys.path.append('{}/catkin_ws/src/mission/src'.format(Path.home()))
import mission_utils as util
from mission_utils import CUT_ROAD_RATIO, IMAGE_HEIGHT, WARP_SIZE

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, estimate_variance, initial_estimate):
        self.Q = process_variance
        self.R = measurement_variance
        self.P = estimate_variance
        self.X = initial_estimate

    def update(self, measurement):
        P_prior = self.P + self.Q
        K = P_prior / (P_prior + self.R)
        self.X = self.X + K * (measurement - self.X)
        self.P = (1 - K) * P_prior
        return self.X

class Cone:
    def __init__(self, x, y, color):
        self.x = x
        self.y = y
        self.color = color

    def get_point(self):
        return (self.x, self.y)

# Callback Functions

def callback_section(msg):
    global section
    section = msg.data

def angle_difference(angle1, angle2):
        diff = (angle2 - angle1 + 540) % 360 - 180
        return diff

def callback_hd(msg):
    global cur_hd, prev_heading  # 전역 변수로 선언
    try:
        raw_heading = msg.heading * 1e-5
        
        if prev_heading is None:
            prev_heading = raw_heading
        
        heading_diff = angle_difference(prev_heading, raw_heading)
        filtered_heading_diff = heading_kalman_filter.update(heading_diff)
        new_heading = (prev_heading + filtered_heading_diff) % 360
        
        cur_hd = math.pi / 2 - (new_heading * math.pi / 180)
        prev_heading = new_heading

        # 헤딩 값 로그 추가
        #rospy.loginfo(f"Raw Heading: {raw_heading:.2f}, Current Heading(rad): {cur_hd:.2f}, Current Heading(deg): {math.degrees(cur_hd):.2f}")

    except Exception as e:
        rospy.logerr(f"Error in callback_hd: {str(e)}")

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


def lane_control():
    while not rospy.is_shutdown():
        image_msg = rospy.wait_for_message('/usb_cam1/image_raw', Image)
        frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        lane_data = rospy.wait_for_message('lane_data_publisher', String)
        lanes_xys = json.loads(lane_data.data)
        lanes_xys = [[(x, y - int(CUT_ROAD_RATIO * IMAGE_HEIGHT)) for x, y in lane] for lane in lanes_xys]
        
        result = cone_model.predict(frame, verbose=False)[0]
        boxes = result.boxes.xyxy
        x_list = list()
        y_list = list()

        for box in boxes:
            box = [int(pt) for pt in box]
            start = (box[0], box[1])
            end = (box[2], box[3])

            cone_pixels = frame[int(0.4 * box[1] + 0.6 * box[3]):int(0.1 * box[1] + 0.9 * box[3]),
                                int(0.6 * box[0] + 0.4 * box[2]):int(0.4 * box[0] + 0.6 * box[2]), :]
            cone_pixels = cone_pixels.reshape(-1, 3)

            B_avg = sum(cone_pixels[:, 0]) / len(cone_pixels)
            G_avg = sum(cone_pixels[:, 1]) / len(cone_pixels)
            R_avg = sum(cone_pixels[:, 2]) / len(cone_pixels)
            # red_distance = (B_avg) ** 2 + G_avg ** 2 + (255-R_avg) ** 2
            # blue_distance = (B_avg-255) ** 2 + G_avg ** 2 + (R_avg) ** 2
            # yellow_distance = B_avg ** 2 + (255 - G_avg) ** 2 + (255 - R_avg) ** 2
            # distance_list = [red_distance, blue_distance, yellow_distance]
            # min_idx = np.argmin(distance_list)

            if B_avg > 100:
                color_type = 'blue'
            else:
                if abs(G_avg - R_avg) > 80:
                    color_type = 'orange'
                else:
                    color_type = 'yellow'


            if color_type == 'orange':  # Orange Cone
                pixel_coord = [[int((start[0]+end[0])/2), int(end[1]-CUT_ROAD_RATIO*IMAGE_HEIGHT)]]
                points = np.array(pixel_coord, dtype=np.float32)
                points = points.reshape(-1, 1, 2)
                transformed_points = cv2.perspectiveTransform(points, M).reshape(-1, 2)[0]
                transformed_points[1] = -(transformed_points[1] - WARP_SIZE)
                x_list.append(transformed_points[0])
                y_list.append(transformed_points[1])

        est_coord_list = util.convert_point_bevcam2lidar(x_list, y_list)
        try:
            left_lane_lidar_points, right_lane_lidar_points = util.get_current_lane_lidar_points(lanes_xys, M, 3.8)
        except:
            continue
        left_lane_lidar_fit = util.fit_polynomial(left_lane_lidar_points, 1)
        right_lane_lidar_fit = util.fit_polynomial(right_lane_lidar_points, 1)
        left_lane_lidar_fit[1] -= 0.5
        right_lane_lidar_fit[1] += 0.5
        est_coord_list = [pt for pt in est_coord_list if np.polyval(right_lane_lidar_fit, pt[0]) <= pt[1] <= np.polyval(left_lane_lidar_fit, pt[0])]
        if len(est_coord_list) > 0:
            distance_list = [util.calculate_distance((0,0), pt) for pt in est_coord_list]
            if 8 <= min(distance_list) <= 15:
                uturn_closed_pub.publish(True)
            elif min(distance_list) < 8:
                break

def get_orange_obstacles(frame):
    global obstacles
    obstacles_temp = copy.deepcopy(obstacles)

    if len(obstacles_temp) == 0:
        return list(), None

    result = cone_model.predict(frame, verbose=False)[0]

    boxes = [[int(pt) for pt in box] for box in result.boxes.xyxy]

    pixel_list = list()
    for i, box in enumerate(boxes):
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
        # red_distance = (B_avg) ** 2 + G_avg ** 2 + (255-R_avg) ** 2
        # blue_distance = (B_avg-255) ** 2 + G_avg ** 2 + (R_avg) ** 2
        # yellow_distance = B_avg ** 2 + (255 - G_avg) ** 2 + (255 - R_avg) ** 2
        # distance_list = [red_distance, blue_distance, yellow_distance]
        # min_idx = np.argmin(distance_list)

        if B_avg > 100:
            color_type = 'blue'
        else:
            if abs(G_avg - R_avg) > 80:
                color_type = 'orange'
            else:
                color_type = 'yellow'

        if color_type == 'orange':  # Orange Cone
            pixel_list.append((int(coord_x), int(coord_y-CUT_ROAD_RATIO*IMAGE_HEIGHT)))

    transformed_x_list = list()
    transformed_y_list = list()
    pixel_list = np.array(pixel_list, dtype=np.float32)

    for cone_pixel in pixel_list:
        cone_pixel = cone_pixel.reshape(-1, 1, 2)
        transformed_point = cv2.perspectiveTransform(cone_pixel, M).reshape(-1, 2)[0]
        transformed_x_list.append(transformed_point[0])
        transformed_y_list.append(-(transformed_point[1]-WARP_SIZE))

    est_coords = util.convert_point_bevcam2lidar(transformed_x_list, transformed_y_list)

    lidar_coords = list()
    for est_coord in est_coords:
        est_coord = np.array(est_coord)
        distance_list = np.linalg.norm(obstacles_temp - est_coord, axis=1)
        distance_min_idx = np.argmin(distance_list)
        if distance_list[distance_min_idx] <= 1:
            selected_cone_coord = obstacles_temp[distance_min_idx]
            lidar_coords.append(Cone(selected_cone_coord[0], selected_cone_coord[1], 'orange'))

    return lidar_coords, len(pixel_list)



# 들로네 삼각분할과 경로 생성을 수행하는 함수
def making_path(orange_cones_center, yellow_cones_center):
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

    cone_data = orange_cones_center + yellow_cones_center
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




if __name__ == '__main__':
    rospy.init_node('uturn', anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('--section', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])

    heading_kalman_filter = KalmanFilter(0.1, 0.1, 0.1, 0)
    prev_heading = None 

    # ROS Publisher
    path_pub = rospy.Publisher("/local_path", PathInfo, queue_size=1)    
    vel_pub = rospy.Publisher('/missionKPH', Float32, queue_size=1)
    deg_pub = rospy.Publisher("/missionDeg", Float32, queue_size=1)
    ld_pub = rospy.Publisher('/lidar_pp_ld', Int32, queue_size=1)
    waypoint_vis_pub = rospy.Publisher("/waypoint_vis_pub", MarkerArray, queue_size=1)
    mission_finish_pub = rospy.Publisher('/avoid_finish', Int32, queue_size=1)
    mission_lane_control_pub = rospy.Publisher('/mission_lane_control', Int32, queue_size=1)
    obstacle_vis_pub = rospy.Publisher("/obstacle_vis_pub", MarkerArray, queue_size=1)
    uturn_closed_pub = rospy.Publisher('/uturn_closed', Bool, queue_size=1)
    avoid_finish_pub = rospy.Publisher('/avoid_finish', Int32, queue_size=1)

    # ROS Subscriber
    rospy.Subscriber('/object_info', ObjectInfo, callback_obstacle)
    rospy.Subscriber('/lidarDeg', Float32, callback_deg)
    rospy.Subscriber("/section", Int32, callback_section)
    rospy.Subscriber('/ublox_position_receiver/navpvt', NavPVT, callback_hd)

    obstacles = list()
    started = False
    finished = False
    section = None
    finish_start_time = None
    src, dst, M = util.get_camera_bev_parameters()
    steering_angle = 0
    bridge = CvBridge()
    ploty = np.linspace(0, 5, 20)
    cur_hd = None  # 

    cone_model = YOLO('{}/catkin_ws/ModelFiles/RubberCone_YOLOv10m_1280.pt'.format(Path.home()))
    cone_model.predict(np.zeros((720, 1280, 3)), verbose=False)

    if args.section:
        while True:
            if section == 8:
                break
            else:
                print('not uturn section')
                time.sleep(0.1)

    print('started!!')

    mission_lane_control_pub.publish(1)
    lane_control()
    mission_lane_control_pub.publish(0)
    
    start_time = time.time()
    while not rospy.is_shutdown():
        image_msg = rospy.wait_for_message("/usb_cam1/image_raw", Image)
        frame = bridge.imgmsg_to_cv2(image_msg, "bgr8")

        lane_data = rospy.wait_for_message('lane_data_publisher', String)
        lanes_xys = json.loads(lane_data.data)
        lanes_xys = [[(x, y - int(CUT_ROAD_RATIO * IMAGE_HEIGHT)) for x, y in lane] for lane in lanes_xys]

        current_obstacles, len_cone = get_orange_obstacles(frame)
        local_path, path_x, path_y, _, _ = making_path(current_obstacles, [Cone(1,-1, 'yellow')])
        waypoint = list(zip(path_x, path_y))

        #print('len_cone :', len_cone)

        # slope_list = list()
        # for lane_points in lanes_xys:
        #     points = np.array(lane_points, dtype=np.float32)
        #     points = points.reshape(-1, 1, 2)
        #     transformed_points = cv2.perspectiveTransform(points, M).reshape(-1, 2)
        #     transformed_points[:, 1] = -(transformed_points[:, 1] - WARP_SIZE)

        #     x_list = transformed_points[:, 0]
        #     y_list = transformed_points[:, 1]
        #     coefficients = np.polyfit(y_list, x_list, 1)

        #     plotx = np.polyval(coefficients, ploty)
        #     lidar_points = util.convert_point_bevcam2lidar(plotx, ploty)
        #     lidar_fit = util.fit_polynomial(lidar_points, 1)
        #     slope_list.append(lidar_fit[0])

        # if len(slope_list) > 0:  # slope 정보가 있을 때만 평균 계산, 없는 경우 이전에 계산했던 값 사용
        #     slope_avg = sum(slope_list) / len(slope_list)  
        #     lane_degree = math.degrees(math.atan(slope_avg))
        #     print('abs(lane_degree) :', abs(lane_degree))
        #     if time.time() - start_time > 5 and abs(lane_degree) < 15 and len(lanes_xys) > 2:
        #         deg_pub.publish(0)
        #         avoid_finish_pub.publish(1)
        #         break

        if finished:
            if time.time() - finish_start_time <= 2:
                vel_pub.publish(3)
                deg_pub.publish(-24)
                continue
            else:
                avoid_finish_pub.publish(1)
                break
        
        print('math.degrees(cur_hd) :', math.degrees(cur_hd))
        if -70.0 < math.degrees(cur_hd) < -28.0:
            rospy.loginfo(f"Finish condition met - Current Heading: {math.degrees(cur_hd):.2f} degrees")
            avoid_finish_pub.publish(1)
            break
            # finish_start_time = time.time()
            # finished = True
            # continue

        # Publish Path
        x_points = [pt[0] for pt in waypoint]
        y_points = [pt[1] for pt in waypoint]
        path_msg = PathInfo()
        path_msg.cnt = 20
        path_msg.x = x_points
        path_msg.y = y_points
        path_pub.publish(path_msg)

        # Publish
        vel_pub.publish(8)
        if steering_angle > -10:
            steering_angle = -10
        deg_pub.publish(steering_angle)
        ld_pub.publish(3)


        ### Visualization

        # Waypoint Visualization
        ob = util.visualization_marker_array(waypoint, (0,255,0), 0.2, 0.2, 0.2, duration=0.1)
        waypoint_vis_pub.publish(ob)

        # Obstacle Visualization
        ob = util.visualization_marker_array([pt.get_point() for pt in current_obstacles], (255,0,0), 0.1, 0.1, 0.1, duration=0.1)
        obstacle_vis_pub.publish(ob)

        

        
        