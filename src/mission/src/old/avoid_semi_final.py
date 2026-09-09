#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from object_detector.msg import ObjectInfo
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from scipy.interpolate import CubicSpline
from pathlib import Path
import numpy as np
import argparse
import time
import json
import copy
import cv2
import math as m

from mission.msg import PathInfo

import warnings
warnings.simplefilter('ignore', np.RankWarning)

import sys
sys.path.append('{}/catkin_ws/src/mission/src'.format(Path.home()))
import mission_utils as util
from mission_utils import CUT_ROAD_RATIO, IMAGE_HEIGHT, WARP_SIZE

LANE_OFFSET = 1.4  # 왼쪽 차선으로부터 몇 미터에 회피 주행경로 좌표를 찍을지 값

def callback_deg(data):
    global steering_angle
    steering_angle = data.data

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.lenx = 0
        self.leny = 0
        self.parent = None
        self.cost = 0.0

class RRTStar:
    def __init__(self, start, goal, dist_to_connect=0.5, num_of_cand_node=7, path_len=20, arrive_dis=0.3, angle=28):
        self.start = start  # Node object
        self.goal = goal  # Node object
        self.dist_to_connect = dist_to_connect
        self.num_of_cand_node = num_of_cand_node
        self.path_len = path_len
        self.arrive = arrive_dis
        self.angle = m.radians(angle)
        self.start.parent = Node(-dist_to_connect, 0)
        self.parent_cand = [self.start]
        self.path = np.zeros((self.path_len, 1), dtype=object)
        self.obstacle_list = []
        self.lane_list = []
        self.combined_ob = []
        self.offset = 1
        self.lane_offset = 0.1
        self.min_dis_ob = 0.75
        self.parent = self.start

    def update_obstacles(self, obstacle_list):
        self.obstacle_list = obstacle_list
        self.update_combined_ob()

    def update_lanes(self, lane_list):
        self.lane_list = lane_list
        self.update_combined_ob()

    def update_combined_ob(self):
        self.combined_ob = self.obstacle_list + self.lane_list

    def generate_random_node(self, node):
        obs = self.combined_ob
        cand_nodes = []
        t = m.atan2((node.y - node.parent.y), (node.x - node.parent.x))
        theta = t
        for i in range(self.num_of_cand_node):
            i_theta = self.angle * (i - ((self.num_of_cand_node - 1) / 2)) / ((self.num_of_cand_node - 1) / 2) + theta
            s = self.dist_to_connect

            cand_node = Node(node.x + s * m.cos(i_theta), node.y + s * m.sin(i_theta))
            cand_node.parent = node
            if any(np.hypot(cand_node.x - ob.x, cand_node.y - ob.y) < self.min_dis_ob for ob in obs):
                continue
            cand_nodes.append(cand_node)

        dis = [m.hypot(node.x - self.goal.x, node.y - self.goal.y) for node in cand_nodes]
        pairs = sorted(zip(cand_nodes, dis), key=lambda x: x[1])

        return pairs

    def find_path(self):
        for i in range(self.path_len):
            node = None

            if not self.parent_cand:
                # No parent candidates, cannot proceed
                break

            for parent_node in self.parent_cand:
                pairs = self.generate_random_node(parent_node)
                nodes = [pair[0] for pair in pairs]

                if len(nodes) == 0:
                    continue
                else:
                    node = nodes[0]
                    break

            if node is None:
                # Cannot find a node
                break

            self.parent_cand = nodes

            dist_to_goal = m.hypot(self.goal.x - node.x, self.goal.y - node.y)

            if dist_to_goal < self.arrive + 1e-6:
                self.path[i][0] = node
                self.parent = node
                break
            else:
                if i == self.path_len - 1:
                    self.path[i][0] = node
                    self.parent = node
                    break
                else:
                    self.path[i][0] = node
                    self.parent = node

        path = []
        current_node = self.parent

        while current_node is not None:
            path.append((current_node.x, current_node.y))
            current_node = current_node.parent

        path.reverse()

        self.parent = self.start
        self.parent_cand = [self.start]

        return path

    def final_path(self):
        path = self.find_path()
        if not path or len(path) < 2:
            return None

        x = np.array([a[0] for a in path])
        y = np.array([a[1] for a in path])

        x_int = np.linspace(x[0], x[-1], num=20)
        try:
            cubic = CubicSpline(x, y)
            y_cubic = cubic(x_int)
        except ValueError:
            y_cubic = np.zeros(len(x_int))

        smoothed_path = list(zip(x_int, y_cubic))
        return smoothed_path

def callback_section(msg):
    global section
    section = msg.data

# 장애물 정보 콜백 함수
def callback_ob(msg):
    global obstacles

    # 장애물 리스트 초기화
    obstacle_list = []

    cx = msg.centerX
    cy = msg.centerY
    lenx = msg.lengthX
    leny = msg.lengthY

    for i in range(msg.objectCounts):
        delta_x = int(lenx[i] // 0.75 + 5)
        delta_y = int(leny[i] // 0.75 + 5)
        x_gap = np.linspace(cx[i] - lenx[i] / 2, cx[i] + lenx[i] / 2, num=delta_x)
        y_gap = np.linspace(cy[i] - leny[i] / 2, cy[i] + leny[i] / 2, num=delta_y)
        for j in range(delta_y):
            ob_minx = (cx[i] - lenx[i] / 2, y_gap[j])
            ob_maxx = (cx[i] + lenx[i] / 2, y_gap[j])
            obstacle_list.append(ob_minx)
            obstacle_list.append(ob_maxx)
        for j in range(delta_x):
            ob_miny = (x_gap[j], cy[i] - leny[i] / 2)
            ob_maxy = (x_gap[j], cy[i] + leny[i] / 2)
            obstacle_list.append(ob_miny)
            obstacle_list.append(ob_maxy)
        
        # 장애물을 앞뒤로 연장
        pt1 = (cx[i] - lenx[i] / 2 - 1, cy[i])
        obstacle_list.append(pt1)
        pt2 = (cx[i] - lenx[i] / 2 - 1, cy[i] + leny[i] / 4)
        obstacle_list.append(pt2)
        pt3 = (cx[i] - lenx[i] / 2 - 1, cy[i] - leny[i] / 4)
        obstacle_list.append(pt3)
        pt4 = (cx[i] - lenx[i] / 2 - 0.5, cy[i] - leny[i] / 2)
        obstacle_list.append(pt4)
        pt5 = (cx[i] - lenx[i] / 2 - 0.5, cy[i] + leny[i] / 2)
        obstacle_list.append(pt5)

    obstacles = obstacle_list  # Update global obstacles

def calculate_goal_point(left_lidar_fit, right_lidar_fit):
    x_goal = 15.0  # 라이다 x좌표가 15인 점

    y_left = np.polyval(left_lidar_fit, x_goal)
    y_right = np.polyval(right_lidar_fit, x_goal)
    y_mid = (y_left + y_right) / 2.0

    return Node(x_goal, y_mid)

def publish_path(waypoint, path_pub):
    x_points = [pt[0] for pt in waypoint]
    y_points = [pt[1] for pt in waypoint]

    path_msg = PathInfo()
    path_msg.cnt = len(x_points)
    path_msg.x = x_points
    path_msg.y = y_points

    path_pub.publish(path_msg)
    vel_pub.publish(6)
    deg_pub.publish(steering_angle)


def visualization_marker_array(data, color, scale_x, scale_y, scale_z, marker_type='cube', duration=0.1, namespace=''):
    marker_array = MarkerArray()
    for idx, point in enumerate(data):
        marker = Marker()
        marker.header.frame_id = "velodyne"
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = idx
        if marker_type == 'cube':
            marker.type = Marker.CUBE
        elif marker_type == 'line_strip':
            marker.type = Marker.LINE_STRIP
        else:
            marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0
        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.scale.z = scale_z
        marker.color.a = 1.0
        marker.color.r = color[0]/255.0
        marker.color.g = color[1]/255.0
        marker.color.b = color[2]/255.0
        marker.lifetime = rospy.Duration(duration)
        marker_array.markers.append(marker)
    return marker_array

def clear_markers(pub, namespace=''):
    # Function to delete previous markers
    marker_array = MarkerArray()
    marker = Marker()
    marker.header.frame_id = "velodyne"
    marker.header.stamp = rospy.Time.now()
    marker.ns = namespace
    marker.action = Marker.DELETEALL
    marker_array.markers.append(marker)
    pub.publish(marker_array)

if __name__ == '__main__':
    rospy.init_node('avoid_small_node', anonymous=True)

    parser = argparse.ArgumentParser()
    parser.add_argument('--section', action='store_true')
    parser.add_argument('--tunnel', action='store_true')
    args = parser.parse_args(rospy.myargv()[1:])

    src, dst, M = util.get_camera_bev_parameters()

    left_lidar_points = None
    right_lidar_points = None
    obstacles = []
    section = None
    started = True
    obstacle_closed = False
    finished = False
    start_time = None
    steering_angle = 0

    # ROS Publisher
    path_pub = rospy.Publisher("/local_path", PathInfo, queue_size=1)
    avoid_finish_pub = rospy.Publisher('/avoid_finish', Int32, queue_size=1)
    tunnel_avoid_finish_pub = rospy.Publisher('/tunnel_avoid_finish', Int32, queue_size=1)
    waypoint_vis_pub = rospy.Publisher("/waypoint_vis_pub", MarkerArray, queue_size=1)
    obstacle_vis_pub = rospy.Publisher("/obstacle_vis_pub", MarkerArray, queue_size=1)
    lane_vis_pub = rospy.Publisher("/lane_vis_pub", MarkerArray, queue_size=1)
    vel_pub = rospy.Publisher("/missionKPH", Float32, queue_size=1)
    deg_pub = rospy.Publisher("/missionDeg", Float32, queue_size=1)

    # ROS Subscriber
    rospy.Subscriber('/object_info', ObjectInfo, callback_ob)
    rospy.Subscriber("/section", Int32, callback_section)
    rospy.Subscriber('/lidarDeg', Float32, callback_deg)

    if args.section:
        while not rospy.is_shutdown():
            if section == 2:
                break
            else:
                print('not avoid section')
                time.sleep(0.1)

    if args.tunnel:
        while not rospy.is_shutdown():
            print('wait for tunnel section')
            value = rospy.wait_for_message('/tunnel_distinguish', Int32)
            if value.data == 1:
                print('\n\nget tunnel')
                break

    print('started!!\n')

    bridge = CvBridge()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        image_msg = rospy.wait_for_message('/usb_cam1/image_raw', Image)
        frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        lane_data = rospy.wait_for_message('lane_data_publisher', String)
        lanes_xys = json.loads(lane_data.data)
        lanes_xys = [[(x, y - int(CUT_ROAD_RATIO * IMAGE_HEIGHT)) for x, y in lane] for lane in lanes_xys]

        ret_val = util.get_current_lane_lidar_points(lanes_xys, M)
        if ret_val is None:
            print('lane not detected!!')
            continue
        else:
            left_lidar_points, right_lidar_points = ret_val

        # 차선 데이터의 점 개수를 각각 25개로 줄입니다.
        if len(left_lidar_points) > 25:
            indices = np.linspace(0, len(left_lidar_points) - 1, 25).astype(int)
            left_lidar_points = [left_lidar_points[i] for i in indices]
        if len(right_lidar_points) > 25:
            indices = np.linspace(0, len(right_lidar_points) - 1, 25).astype(int)
            right_lidar_points = [right_lidar_points[i] for i in indices]

        left_lidar_fit = util.fit_polynomial(left_lidar_points)
        right_lidar_fit = util.fit_polynomial(right_lidar_points)

        # Lane Visualization
        clear_markers(lane_vis_pub, namespace='lane')
        lane_vis_points = left_lidar_points + right_lidar_points
        ob = visualization_marker_array(lane_vis_points, (255, 255, 255), 0.1, 0.1, 0.1, namespace='lane')
        lane_vis_pub.publish(ob)

        # 장애물 데이터 가져오기
        obstacles_temp = copy.deepcopy(obstacles)
        current_obstacle_list = util.get_current_obstacles(left_lidar_points, right_lidar_points, obstacles_temp)

        # Obstacle Visualization
        clear_markers(obstacle_vis_pub, namespace='obstacle')
        ob = visualization_marker_array(current_obstacle_list, (255, 0, 0), 0.3, 0.3, 0.3, 'cube', 0.1, namespace='obstacle')
        obstacle_vis_pub.publish(ob)

        if started:
            if current_obstacle_list:
                closest_obstacle = current_obstacle_list[0]
                distance = util.calculate_distance((0, 0), closest_obstacle)
                if distance <= 3:
                    print('라이다에 잡힘, 거리 3 이하')
                    started = False
                    continue
                else:
                    print('라이다에 잡힘, 거리 3 초과')
            else:
                print('No obstacles detected.')

            waypoint_temp = util.make_virtual_lane('left', right_lidar_points, 1.0)
            waypoint_fit = util.fit_polynomial(waypoint_temp)
            x_list = np.linspace(0, 7, 20)
            y_list = np.polyval(waypoint_fit, x_list)
            waypoint = list(zip(x_list, y_list))

            # Path Visualization
            clear_markers(waypoint_vis_pub, namespace='waypoint')
            ob = visualization_marker_array(waypoint, (0, 255, 0), 0.2, 0.2, 0.2, 'cube', 0.1, namespace='waypoint')
            waypoint_vis_pub.publish(ob)

            # Publish Path
            publish_path(waypoint, path_pub)

            continue

        if finished:
            time_elapsed = time.time() - start_time
            if 0 <= time_elapsed < 2:
                waypoint_temp = util.make_virtual_lane('right', left_lidar_points, LANE_OFFSET)
                waypoint_fit = util.fit_polynomial(waypoint_temp)
                x_list = np.linspace(0, 7, 20)
                y_list = np.polyval(waypoint_fit, x_list)
                waypoint = list(zip(x_list, y_list))

            elif time_elapsed < 3:
                waypoint = util.generate_lane_center_path(left_lidar_points, right_lidar_points)

            else:
                print('finish!')
                if args.tunnel:
                    tunnel_avoid_finish_pub.publish(1)
                else:
                    avoid_finish_pub.publish(1)

                break  # 종료

            # Path Visualization
            clear_markers(waypoint_vis_pub, namespace='waypoint')
            ob = visualization_marker_array(waypoint, (0, 255, 0), 0.2, 0.2, 0.2, 'cube', 0.1, namespace='waypoint')
            waypoint_vis_pub.publish(ob)

            # Publish Path
            publish_path(waypoint, path_pub)

            continue

        # After started and Before finished

        # Convert obstacles to Node objects
        obstacle_nodes = [Node(x, y) for x, y in current_obstacle_list]

        # Convert lane points to Node objects
        lane_points = left_lidar_points + right_lidar_points
        lane_nodes = [Node(x, y) for x, y in lane_points]

        # Calculate Goal Point
        goal_node = calculate_goal_point(left_lidar_fit, right_lidar_fit)
        start_node = Node(0.0, 0.0)

        # Create RRTStar instance
        rrt_star = RRTStar(start_node, goal_node)
        rrt_star.update_obstacles(obstacle_nodes)
        rrt_star.update_lanes(lane_nodes)

        # Compute path
        path = rrt_star.final_path()

        if path is not None:
            # Path Visualization
            clear_markers(waypoint_vis_pub, namespace='waypoint')
            ob = visualization_marker_array(path, (0, 255, 0), 0.2, 0.2, 0.2, 'cube', 0.1, namespace='waypoint')
            waypoint_vis_pub.publish(ob)

            # Publish Path
            publish_path(path, path_pub)
        else:
            print('No valid path found by RRT*')

        rate.sleep()
