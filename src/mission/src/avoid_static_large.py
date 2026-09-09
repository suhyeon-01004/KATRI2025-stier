#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from object_detector.msg import ObjectInfo
from std_msgs.msg import Int32, Float32MultiArray 
from visualization_msgs.msg import Marker, MarkerArray
from ublox_msgs.msg import NavPVT
from geometry_msgs.msg import Point

circle_pub = None

class ObstacleDistanceChecker:
    def __init__(self):
        self.rddf_coords_1 = self.load_rddf("/home/stier/catkin_ws/src/control/stier/paths/lane1111.txt")
        self.rddf_coords_2 = self.load_rddf("/home/stier/catkin_ws/src/control/stier/paths/lane2222.txt")

        self.cur_position_x = None
        self.cur_position_y = None
        self.cur_hd = None
        #칼만필터
        self.hd_est = 0.0
        self.hd_P = 0.1
        self.hd_Q = 1e-3
        self.hd_R = 5e-3
        #비교를 위한 칼만 필터값
        self.hd_est_slow = 0.0
        self.hd_P_slow = 0.1

        self.lane1_flag = 0
        self.lane2_flag = 0
        self.input_c = 0

        self.target_pub = rospy.Publisher("/target_lane", Int32, queue_size=1)
        self.marker_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=10)
        self.prev_obstacle_count = 0 # 초기화

        self.section = None

        rospy.Subscriber("/object_info", ObjectInfo, self.callback_ob)
        rospy.Subscriber("/utm", PoseStamped, self.callback_gps)
        rospy.Subscriber("/ublox_position_receiver/navpvt", NavPVT, self.callback_hd)
        rospy.Subscriber("/current_lane", Int32, self.callback_input_c)
        rospy.Subscriber("/section", Int32, self.callback_section)

    def load_rddf(self, path):
        with open(path, "r") as f:
            return [tuple(map(float, line.strip().split("\t"))) for line in f.readlines()]

    def callback_section(self, msg):
        self.section = msg.data

    def callback_gps(self, msg):
        self.cur_position_x = msg.pose.position.x
        self.cur_position_y = msg.pose.position.y

    def callback_hd(self, msg):
        # 1) 측정값을 라디안으로 변환 (NavPVT.heading 은 1e‐5deg 단위)
        raw_heading = msg.heading * 1e-5 * math.pi / 180.0

        # 2) 예측 단계
        self.hd_P = self.hd_P + self.hd_Q

        # 3) 칼만 이득 계산
        K = self.hd_P / (self.hd_P + self.hd_R)

        # 4) “각도 차이” (innovation)을 [-π, +π] 범위로 래핑
        #    예: diff = raw - est 를 무조건 -π~+π로 줄여서 큰 점프 방지
        diff = raw_heading - self.hd_est
        # atan2(sin(diff), cos(diff)) 를 쓰면 항상 -π~+π 로 정규화됨
        diff = math.atan2(math.sin(diff), math.cos(diff))

        # 5) 상태 갱신: 이전 추정치 + K * (래핑된 차이)
        self.hd_est = self.hd_est + K * diff

        # 6) 공분산 갱신
        self.hd_P = (1 - K) * self.hd_P

        # 7) 최종 헤딩을 역시 -π~+π로 정규화 (선택적)
        self.hd_est = math.atan2(math.sin(self.hd_est), math.cos(self.hd_est))

        self.cur_hd = self.hd_est

        # --- 실험용 필터 1: 매우 부드러움 ---
        Q_slow, R_slow = 1e-4, 1e-2
        P_slow = self.hd_P_slow + Q_slow
        K_slow = P_slow / (P_slow + R_slow)
        diff_slow = math.atan2(math.sin(raw_heading - self.hd_est_slow), math.cos(raw_heading - self.hd_est_slow))
        self.hd_est_slow += K_slow * diff_slow
        self.hd_est_slow = math.atan2(math.sin(self.hd_est_slow), math.cos(self.hd_est_slow))
        self.hd_P_slow = (1 - K_slow) * P_slow

        msg = Float32MultiArray()
        msg.data = [
            raw_heading * 180 / math.pi,     # 원본
            self.hd_est * 180 / math.pi         # 기본 필터 (주행용)
        ]
        

    def callback_input_c(self, msg):
        self.input_c = msg.data

    def callback_ob(self, msg):
        if self.cur_position_x is None or self.cur_hd is None:
            return

        # 절대→상대 좌표 변환
        rel_rddf_1 = []
        rel_rddf_2 = []
        for (xr, yr) in self.rddf_coords_1:
            dx = xr - self.cur_position_x
            dy = yr - self.cur_position_y
            nx = dx * math.cos(self.cur_hd) - dy * math.sin(self.cur_hd)
            ny = dx * math.sin(self.cur_hd) + dy * math.cos(self.cur_hd)
            rel_rddf_1.append((ny, -nx))
        for (xr, yr) in self.rddf_coords_2:
            dx = xr - self.cur_position_x
            dy = yr - self.cur_position_y
            nx = dx * math.cos(self.cur_hd) - dy * math.sin(self.cur_hd)
            ny = dx * math.sin(self.cur_hd) + dy * math.cos(self.cur_hd)
            rel_rddf_2.append((ny, -nx))

        DIST_THRESHOLD = 0.9
        visualize_rddf_circles(rel_rddf_1, rel_rddf_2, DIST_THRESHOLD)
        cx = msg.centerX
        cy = msg.centerY
        count = msg.objectCounts
        lenx = msg.lengthX

        flags = []
        for rddf_coords in [rel_rddf_1, rel_rddf_2]:
            flag = 0
            for i in range(count):
                if cx[i] < 0.2 or cx[i] > 6.0:
                    continue
                dists = [
                    np.linalg.norm(np.array([cx[i] - rddf_x, cy[i] - rddf_y]))
                    for (rddf_x, rddf_y) in rddf_coords
                ]
                if dists and min(dists) < DIST_THRESHOLD:
                    flag = 1
                    break
            flags.append(flag)

        self.lane1_flag = flags[0]
        self.lane2_flag = flags[1]

        marker_array = MarkerArray()
        for i in range(count):
            marker = Marker()
            marker.header.frame_id = "velodyne"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = cx[i]
            marker.pose.position.y = cy[i]
            marker.pose.position.z = 0.5
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.lifetime = rospy.Duration(0.4) # 초기화
            marker_array.markers.append(marker)
        # --- 이전 프레임의 잔여 id를 명시적으로 삭제 ---
        delete_arr = MarkerArray()
        for j in range(count, self.prev_obstacle_count):
            dm = Marker()
            dm.header.frame_id = "velodyne"
            dm.header.stamp = rospy.Time.now()
            dm.ns = "obstacles"
            dm.id = j
            dm.action = Marker.DELETE  # 2
            delete_arr.markers.append(dm)
        if delete_arr.markers:
            self.marker_pub.publish(delete_arr)

        # 이번 프레임 퍼블리시
        self.marker_pub.publish(marker_array)
        self.prev_obstacle_count = count

        # 상대 RDDF를 MarkerArray로 만들어 “velodyne” 프레임에 시각화
        rddf_marker_array = MarkerArray()
        # – rel_rddf_1 (첫 번째 차선) 노드들을 노란색 구체로 찍는다
        for idx, (rx, ry) in enumerate(rel_rddf_1):
            m = Marker()
            m.header.frame_id = "velodyne"
            m.header.stamp = rospy.Time.now()
            m.ns = "rddf_lane1"
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = rx
            m.pose.position.y = ry
            m.pose.position.z = 0.1
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 0.0
            rddf_marker_array.markers.append(m)

        # – rel_rddf_2 (두 번째 차선) 노드들을 파란색 구체로 찍는다
        for idx, (rx, ry) in enumerate(rel_rddf_2):
            m = Marker()
            m.header.frame_id = "velodyne"
            m.header.stamp = rospy.Time.now()
            m.ns = "rddf_lane2"
            m.id = 1000 + idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = rx
            m.pose.position.y = ry
            m.pose.position.z = 0.1
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color.a = 1.0
            m.color.r = 0.0
            m.color.g = 0.0
            m.color.b = 1.0
            rddf_marker_array.markers.append(m)

        # 새로 만든 RDDF 마커를 퍼블리시
        self.marker_pub.publish(rddf_marker_array)


class ObstacleInROIChecker:
    def __init__(self, roi_name, roi_bounds):
        self.roi_name = roi_name
        self.roi = roi_bounds
        self.flag = 0
        self.marker_pub = rospy.Publisher(f"/{roi_name}_marker", Marker, queue_size=1)
        self.outline_pub = rospy.Publisher(f"/{roi_name}_outline", Marker, queue_size=1)
        rospy.Subscriber("/object_info", ObjectInfo, self.callback_ob)
        self.publish_roi_marker()
        self.publish_roi_outline()

    def callback_ob(self, msg):
        self.flag = 0
        cx = msg.centerX
        cy = msg.centerY
        count = msg.objectCounts

        for i in range(count):
            x, y = cx[i], cy[i]
            if (
                self.roi["x"][0] <= x <= self.roi["x"][1]
                and self.roi["y"][0] <= y <= self.roi["y"][1]
            ):
                self.flag = 1
                break

        self.publish_roi_outline()

    def publish_roi_marker(self):
        marker = Marker()
        marker.header.frame_id = "velodyne"
        marker.header.stamp = rospy.Time.now()
        marker.ns = self.roi_name
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = (self.roi["x"][0] + self.roi["x"][1]) / 2.0
        marker.pose.position.y = (self.roi["y"][0] + self.roi["y"][1]) / 2.0
        marker.pose.position.z = 0.0
        marker.scale.x = abs(self.roi["x"][1] - self.roi["x"][0])
        marker.scale.y = abs(self.roi["y"][1] - self.roi["y"][0])
        marker.scale.z = 0.1
        marker.color.a = 0.5
        if self.roi_name == "ROI_3":
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        self.marker_pub.publish(marker)

    def publish_roi_outline(self):
        outline = Marker()
        outline.header.frame_id = "velodyne"
        outline.header.stamp = rospy.Time.now()
        outline.ns = f"{self.roi_name}_outline"
        outline.id = 0
        outline.type = Marker.LINE_LIST
        outline.action = Marker.ADD
        outline.scale.x = 0.05
        outline.color.a = 1.0
        if self.roi_name == "ROI_3":
            outline.color.r = 1.0
            outline.color.g = 0.0
            outline.color.b = 1.0
        else:
            outline.color.r = 0.0
            outline.color.g = 1.0
            outline.color.b = 0.0

        x0, x1 = self.roi["x"]
        y0, y1 = self.roi["y"]

        p1 = Point(x=x0, y=y0, z=0.0)
        p2 = Point(x=x1, y=y0, z=0.0)
        p3 = Point(x=x1, y=y1, z=0.0)
        p4 = Point(x=x0, y=y1, z=0.0)

        outline.points.extend([p1, p2, p2, p3, p3, p4, p4, p1])
        self.outline_pub.publish(outline)


def get_target_lane(A, B, C, D, E, before_target_lane):
    if C == 1:
        if A == 1 and E == 0:
            return 2
        elif E == 1:
            return 1
    elif C == 2:
        if B == 1 and D == 0:
            return 1
        elif D == 1:
            return 2
    return before_target_lane

def visualize_rddf_circles(rel_rddf_1, rel_rddf_2, threshold):
    marker_array = MarkerArray()
    num_points = 36

    for idx, (cx, cy) in enumerate(rel_rddf_1):
        m = Marker()
        m.header.frame_id = "velodyne"
        m.header.stamp = rospy.Time.now()
        m.ns = "rddf1_circle"
        m.id = idx
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color.a = 1.0
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0
        for i in range(num_points + 1):
            theta = 2 * math.pi * i / num_points
            x = cx + threshold * math.cos(theta)
            y = cy + threshold * math.sin(theta)
            m.points.append(Point(x=x, y=y, z=0.0))
        marker_array.markers.append(m)

    for idx, (cx, cy) in enumerate(rel_rddf_2):
        m = Marker()
        m.header.frame_id = "velodyne"
        m.header.stamp = rospy.Time.now()
        m.ns = "rddf2_circle"
        m.id = 1000 + idx
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color.a = 1.0
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 1.0
        for i in range(num_points + 1):
            theta = 2 * math.pi * i / num_points
            x = cx + threshold * math.cos(theta)
            y = cy + threshold * math.sin(theta)
            m.points.append(Point(x=x, y=y, z=0.0))
        marker_array.markers.append(m)
    if circle_pub is not None:
        circle_pub.publish(marker_array)


# 퍼블리셔도 클래스 바깥에 선언하라.
# circle_pub = None  # rospy.init_node 이후에 실제 객체로 초기화할 것이므로 None으로 둠


if __name__ == "__main__":
    rospy.init_node("obstacle_distance_checker_main", anonymous=True)

    circle_pub       = rospy.Publisher("/rddf_circles", MarkerArray, queue_size=1)
    node             = ObstacleDistanceChecker()
    roi_checker_1    = ObstacleInROIChecker("ROI_1", {"x": (-1.0, 0.0), "y": (2.0, 4.0)})     #letf
    roi_checker_2    = ObstacleInROIChecker("ROI_2", {"x": (-1.0, 0.0), "y": (-4.0,-2.0)})      #right
    #roi_checker_3    = ObstacleInROIChecker("ROI_3", {"x": (0.5, 2), "y": (-0.5, 0.5)})   #ESTOP
    rddf_marker_pub  = rospy.Publisher("/rddf_marker", MarkerArray, queue_size=1)
    estop_pub        = rospy.Publisher("/ESTOP",      Int32,     queue_size=1)

    rate = rospy.Rate(10)
    before_target_lane = 1
    
    while not rospy.is_shutdown():
        if (node.section != 12):
            print("[WARN] section is not 12")
            rate.sleep()
            continue
        A = node.lane1_flag
        B = node.lane2_flag
        C = node.input_c
        D = roi_checker_1.flag #left
        E = roi_checker_2.flag #right
        #F = roi_checker_3.flag

        #estop_pub.publish(Int32(F))

        result = get_target_lane(A, B, C, D, E, before_target_lane)
        node.target_pub.publish(Int32(result))
        before_target_lane = result

        rospy.loginfo(f"\nLANE1\t:\t{A}\nLANE2\t:\t{B}\nCURRENT\t:\t{C}\nLEFT\t:\t{D}\nRIGHT\t:\t{E}\nTARGET\t:\t{result}\n")
        rate.sleep()
