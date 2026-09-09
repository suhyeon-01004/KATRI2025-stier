
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String, Int32, Float32
import sensor_msgs.point_cloud2 as pc2
from mission.msg import PathInfo
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

from ultralytics import YOLO
from pathlib import Path
import numpy as np
import argparse
import time
import json
import copy
import cv2

import warnings
warnings.simplefilter('ignore', np.RankWarning)

import sys
sys.path.append('{}/catkin_ws/src/mission/src'.format(Path.home()))
import mission_utils as util
from mission_utils import CUT_ROAD_RATIO, IMAGE_HEIGHT, WARP_SIZE, XM_PER_PIXEL

SLOW_DISTANCE = 20
FIRST_AVOID_DISTANCE = 12
SECOND_AVOID_DISTANCE = 9
ROAD_WIDTH = 3.5
BASE_SPEED = 10

### Callback Functions

def callback_section(msg):
    global section
    section = msg.data

def callback_pointcloud(data):
    global pointcloud_msg
    pointcloud_msg = data


def callback_deg(data):
    global steering_angle
    steering_angle = data.data



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
        coefficients = np.polyfit(y_list, x_list, 3)
        
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

def calculate_criteria_lane_dir(x_intercept_list, criteria_lane_idx):
    left_lane_idx = util.get_left_lane_index(x_intercept_list)
    right_lane_idx = util.get_right_lane_index(x_intercept_list)

    if left_lane_idx == criteria_lane_idx:
        return 'left'
    elif right_lane_idx == criteria_lane_idx:
        return 'right'

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

            pixel_coord = [[int((start[0]+end[0])/2), int(end[1]-CUT_ROAD_RATIO*IMAGE_HEIGHT)]]
            points = np.array(pixel_coord, dtype=np.float32)
            points = points.reshape(-1, 1, 2)
            transformed_points = cv2.perspectiveTransform(points, M).reshape(-1, 2)[0]
            transformed_points[1] = -(transformed_points[1] - WARP_SIZE)
            x_list.append(transformed_points[0])
            y_list.append(transformed_points[1])

        est_coord_list = util.convert_point_bevcam2lidar(x_list, y_list)        
        ret_val = util.get_current_lane_lidar_points(lanes_xys, M, virtual_lane_width=3.5)
        if ret_val is None:
            if left_lane_lidar_points is None or right_lane_lidar_points is None:
                continue
        else:
            left_lane_lidar_points, right_lane_lidar_points = ret_val
        left_lane_lidar_fit = util.fit_polynomial(left_lane_lidar_points, 1)
        right_lane_lidar_fit = util.fit_polynomial(right_lane_lidar_points, 1)
        est_coord_list = [pt for pt in est_coord_list if np.polyval(right_lane_lidar_fit, pt[0]) <= pt[1] <= np.polyval(left_lane_lidar_fit, pt[0])]

        if len(est_coord_list) > 0:
            distance_list = [util.calculate_distance((0,0), pt) for pt in est_coord_list]
            if min(distance_list) <= SLOW_DISTANCE:
                break

def get_closest_pointcloud(left_lane_lidar_points, right_lane_lidar_points):
    global pointcloud_msg

    if pointcloud_msg is None:
        return None

    objPoints = np.array([[x, y] for x, y, z in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True) if x > 1 and z > -0.3])

    left_lane_fit = util.fit_polynomial(left_lane_lidar_points, 3)
    right_lane_fit = util.fit_polynomial(right_lane_lidar_points, 3)

    points = list()
    for pt in objPoints:
        if np.polyval(right_lane_fit, pt[0])+0.5 <= pt[1] <= np.polyval(left_lane_fit, pt[0])-0.5:
            points.append(pt)

    ret_val = None
    if len(points) > 0:
        distance_list = [util.calculate_distance((0,0), pt) for pt in points]
        min_idx = np.argmin(distance_list)
        min_point = points[min_idx]
        min_distance = distance_list[min_idx]
        ret_val = min_point, min_distance

    return ret_val



def publish_path(waypoint, path_pub):
    x_points = [pt[0] for pt in waypoint]
    y_points = [pt[1] for pt in waypoint]
    path_msg = PathInfo()
    path_msg.cnt = 20
    path_msg.x = x_points
    path_msg.y = y_points
    path_pub.publish(path_msg)
    

if __name__ == '__main__':
    rospy.init_node('avoid_car', anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('--section', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])

    bridge = CvBridge()
    first_obstacle_passed = False
    second_obstacle_passed = False
    first_obstacle_closed = False
    second_obstacle_closed = False
    obstacles = list()
    pointcloud_msg = None
    section = None
    finish_start_time = None
    steering_angle = 0
    ld_value = 0

    # BEV Values
    src, dst, M = util.get_camera_bev_parameters()

    # ROS Publisher
    path_pub = rospy.Publisher("/local_path", PathInfo, queue_size=1)
    ld_pub = rospy.Publisher('/lidar_pp_ld', Int32, queue_size=1)
    avoid_finish_pub = rospy.Publisher('/avoid_finish', Int32, queue_size=1)
    mission_lane_control_pub = rospy.Publisher('/mission_lane_control', Int32, queue_size=1)
    vel_pub = rospy.Publisher('/missionKPH', Float32, queue_size=1)
    deg_pub = rospy.Publisher("/missionDeg", Float32, queue_size=1)
    waypoint_vis_pub = rospy.Publisher("/waypoint_vis_pub", MarkerArray, queue_size=1)
    criteria_lane_vis_pub = rospy.Publisher("/criteria_lane_vis_pub", MarkerArray, queue_size=1)
    obstacle_vis_pub = rospy.Publisher('/obstacle_vis_pub', MarkerArray, queue_size=1)

    # ROS Subscriber
    rospy.Subscriber('/lidarDeg', Float32, callback_deg)
    rospy.Subscriber("/section", Int32, callback_section)
    rospy.Subscriber('/velodyne_points', PointCloud2, callback_pointcloud)

    cone_model = YOLO('{}/catkin_ws/ModelFiles/RubberCone_YOLOv10m_1280.pt'.format(Path.home()))
    cone_model.predict(np.zeros((720, 1280, 3)), verbose=False)

    if args.section:
        while True:
            if section == 12:
                break
            else:
                print('not avoid section')
                time.sleep(0.1)

    print('enter avoid section')
    mission_lane_control_pub.publish(1)
    lane_control()
    mission_lane_control_pub.publish(0)

    brake_start_time = time.time()
    criteria_lane_x_intercept = calculate_criteria_lane_first()
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
            coefficients = np.polyfit(y_list, x_list, 3)
            
            lane_coefficients.append(coefficients)
            x_intercept_list.append(coefficients[-1])

        ret_val = calculate_criteria_lane(criteria_lane_x_intercept, x_intercept_list)
        if ret_val is None:
            pass
        else:
            criteria_lane_idx, criteria_lane_x_intercept = ret_val
            criteria_lane_dir = calculate_criteria_lane_dir(x_intercept_list, criteria_lane_idx)
            ploty = np.linspace(0, WARP_SIZE-1, WARP_SIZE)
            criteria_lane_fit = lane_coefficients[criteria_lane_idx]
            criteria_plotx = np.polyval(criteria_lane_fit, ploty)
            criteria_lidar_points = util.convert_point_bevcam2lidar(criteria_plotx, ploty)

        # 회피 코드 시작
        if not first_obstacle_passed:  # 1번째 장애물 통과 전
            right_lane_lidar_points = copy.deepcopy(criteria_lidar_points)
            left_lane_lidar_points = util.make_virtual_lane('left', right_lane_lidar_points, ROAD_WIDTH)
            ret_val = get_closest_pointcloud(left_lane_lidar_points, right_lane_lidar_points)
            if ret_val is None:
                nearest_obstacle_point = None
                nearest_distance = float('inf')
            else:
                nearest_obstacle_point, nearest_distance = ret_val                
            print('nearest_distance :', nearest_distance)

            if nearest_distance >= FIRST_AVOID_DISTANCE and not first_obstacle_closed:  # (1)
                print(11111111)
                waypoint = util.generate_lane_center_path(left_lane_lidar_points, right_lane_lidar_points)
                vel = BASE_SPEED
                ld_value = 6

            if nearest_distance < FIRST_AVOID_DISTANCE or first_obstacle_closed:  # (2)
                print(22222222)
                
                if not first_obstacle_closed:
                    first_obstacle_closed = True

                if criteria_lane_dir == 'left':
                    first_obstacle_passed = True
                    continue
                
                waypoint_temp = util.make_virtual_lane('right', criteria_lidar_points, ROAD_WIDTH/2)
                waypoint_end_point = None
                for pt in waypoint_temp:
                    if util.calculate_distance((0,0), pt) >= 8:
                        waypoint_end_point = pt
                        break
                if waypoint_end_point is None:
                    waypoint_end_point = waypoint_temp[-1]
                
                temp_fit = np.polyfit([0, waypoint_end_point[0]], [0, waypoint_end_point[1]], 1)

                plotx = np.linspace(0, waypoint_end_point[0], 20)
                ploty = np.polyval(temp_fit, plotx)
                waypoint = [pt for pt in zip(plotx, ploty)]

                vel = BASE_SPEED
                ld_value = 8


        if first_obstacle_passed and not second_obstacle_passed:  # 1번째 장애물 통과 후
            
            left_lane_lidar_points = copy.deepcopy(criteria_lidar_points)
            right_lane_lidar_points = util.make_virtual_lane('right', left_lane_lidar_points, ROAD_WIDTH)
            ret_val = get_closest_pointcloud(left_lane_lidar_points, right_lane_lidar_points)
            if ret_val is None:
                nearest_obstacle_point = None
                nearest_distance = float('inf')
            else:
                nearest_obstacle_point, nearest_distance = ret_val      
            print('nearest_distance  :', nearest_distance)

            if nearest_distance >= SECOND_AVOID_DISTANCE and not second_obstacle_closed:  # (3)
                print(33333333)
                waypoint = util.generate_lane_center_path(left_lane_lidar_points, right_lane_lidar_points)
                vel = BASE_SPEED
                ld_value = 7

            if nearest_distance < SECOND_AVOID_DISTANCE or second_obstacle_closed:  # (4)
                print(44444444)
                if not second_obstacle_closed:
                    second_obstacle_closed = True

                if criteria_lane_dir == 'right':
                    second_obstacle_passed = True  # 대형 정적 회피 완료
                    finish_start_time = time.time()
                    continue
                
                # 현재 장애물과 가장 가까운 장애물과 기준 차선의 여러 좌표 중 가장 가까운 차선 위의 좌표
                nearest_criteria_lane_point_idx = np.argmin([util.calculate_distance(pt, nearest_obstacle_point) for pt in criteria_lidar_points])
                nearest_criteria_lane_point = criteria_lidar_points[nearest_criteria_lane_point_idx]
                criteria_lane_lidar_fit = util.fit_polynomial(criteria_lidar_points)
                target_point = util.calculate_points_at_distance(criteria_lane_lidar_fit, nearest_criteria_lane_point, 2.2)[1]
                
                temp_fit = np.polyfit([0, target_point[0]], [0, target_point[1]], 1)

                plotx = np.linspace(0, target_point[0], 20)
                ploty = np.polyval(temp_fit, plotx)
                waypoint = [pt for pt in zip(plotx, ploty)]

                vel = BASE_SPEED
                ld_value = 7

        if second_obstacle_passed:
            print(55555555)
            right_lane_lidar_points = copy.deepcopy(criteria_lidar_points)
            left_lane_lidar_points = util.make_virtual_lane('left', right_lane_lidar_points)
            waypoint = util.generate_lane_center_path(left_lane_lidar_points, right_lane_lidar_points)
            vel = 25
            ld_value = 8

            if time.time() - finish_start_time > 3:
                print('finish!!!')
                avoid_finish_pub.publish(1)
                break

        if time.time() - brake_start_time <= 1:
            print('brake!!')
            vel = 4
            ld_value = 7
            waypoint = util.generate_lane_center_path(left_lane_lidar_points, right_lane_lidar_points)


        ### Visualization
        # Criteria Lane Visualization
        lane_vis_points = [pt for i, pt in enumerate(criteria_lidar_points) if i%10 == 0]
        ob = util.visualization_marker_array(lane_vis_points, (255,255,255), 0.1, 0.1, 0.1)
        criteria_lane_vis_pub.publish(ob)

        # Nearest Obstacle Visualization
        if nearest_obstacle_point is not None:
            ob = util.visualization_marker_array([nearest_obstacle_point], (255,0,0), 0.3, 0.3, 0.3, 'sphere', 0.1)
            obstacle_vis_pub.publish(ob)

        # Waypoint Visualization
        ob = util.visualization_marker_array(waypoint, (0,255,0), 0.1, 0.1, 0.1)
        waypoint_vis_pub.publish(ob)

        # Publish
        publish_path(waypoint, path_pub)
        ld_pub.publish(ld_value)
        vel_pub.publish(vel)
        deg_pub.publish(steering_angle)
