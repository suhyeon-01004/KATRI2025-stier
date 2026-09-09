#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from enum import Enum
from std_msgs.msg import Int32
from ublox_msgs.msg import NavPVT
from mission.msg import ObstacleArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA  # 이 줄을 추가해주세요
from collections import deque
import math as m

class State(Enum):
    STRAIGHT = 0
    FULL_STEERING = 1
    FINISH_TURN = 2

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

class ImprovedLogger:
    def __init__(self, node_name, log_interval=1.0):
        self.node_name = node_name
        self.log_interval = log_interval
        self.log_buffer = {
            'state': deque(maxlen=1),
            'section': deque(maxlen=1),
            'cones': deque(maxlen=1),
            'heading': deque(maxlen=1),
            'cone_positions': deque(maxlen=1),
            'cones_coords': deque(maxlen=1)  # 새로운 카테고리 추가
        }
        self.last_log_time = rospy.get_time()

    def log(self, category, message):
        self.log_buffer[category].append(message)

    def publish_logs(self):
        current_time = rospy.get_time()
        if current_time - self.last_log_time >= self.log_interval:
            log_message = " | ".join([
                item for sublist in self.log_buffer.values()
                for item in sublist if item
            ])
            if log_message:
                rospy.loginfo(f"[{self.node_name}] {log_message}")
            self.last_log_time = current_time

class UTurnController:
    def __init__(self):
        self.state = State.STRAIGHT
        self.gray_cones = []
        self.ob = []
        self.section = 0
        self.cur_hd = 0
        self.cur_position_x = 0.0  
        self.cur_position_y = 0.0  
        self.heading_kalman_filter = KalmanFilter(0.1, 0.1, 0.1, 0)
        self.cones_in_range = 0

        self.minx1, self.maxx1 = 0, 8.0
        self.miny1, self.maxy1 = -0.7, 0.7
        self.minx2, self.maxx2 = 0.7, 2.5
        self.miny2, self.maxy2 = -3.0, 0.0
        
        self.full_steer_pub = rospy.Publisher("/uturn_msgs", Int32, queue_size=1)
        self.marker_pub = rospy.Publisher('/detection_range', Marker, queue_size=1)

        rospy.Subscriber('/obstacle_points', ObstacleArray, self.callback_cluster)
        rospy.Subscriber('/section', Int32, self.callback_section)
        rospy.Subscriber('/ublox_position_receiver/navpvt', NavPVT, self.callback_hd)
        rospy.Subscriber('/utm', PoseStamped, self.callback_gps)

        self.logger = ImprovedLogger('uturn_controller')

    def callback_section(self, msg):
        self.section = msg.data
        self.logger.log('section', f"Current section: {self.section}")

    def callback_hd(self, msg):
        
        try:
            raw_heading = msg.heading * 1e-5 * m.pi / 180  # 헤딩 값을 라디안으로 변환
            self.cur_hd = m.pi / 2 - self.heading_kalman_filter.update(raw_heading)  # 칼만 필터로 헤딩 값 안정화
            self.logger.log('heading', f"Current heading: {self.cur_hd}")
        except Exception as e:
            rospy.logerr(f"Error in callback_hd: {str(e)}")
    
    def callback_gps(self, msg):
        self.cur_position_x = msg.pose.position.x
        self.cur_position_y = msg.pose.position.y
    
    def absolute_to_relative(self, abs_x, abs_y):
        """절대 좌표를 현재 위치와 헤딩을 기준으로 상대 좌표로 변환"""
        delta_x = abs_x - self.cur_position_x
        delta_y = abs_y - self.cur_position_y

        rel_x = delta_x * m.cos(self.cur_hd) + delta_y * m.sin(self.cur_hd)
        rel_y = -delta_x * m.sin(self.cur_hd) + delta_y * m.cos(self.cur_hd)

        return rel_x, rel_y
    
    def callback_cluster(self, msg):
        try:
            self.ob = []

            for point in msg.points:
                abs_x, abs_y = point.x, point.y
                rel_x, rel_y = self.absolute_to_relative(abs_x, abs_y)
                cone = Cone(rel_x, rel_y, 'gray')
                
                self.ob.append(cone)

            self.logger.log('cones', f"Number of detected cones: {len(self.ob)}")
            cone_coords = [f"({cone.x:.2f}, {cone.y:.2f})" for cone in self.ob]
            self.logger.log('cones_coords', f"Cone coordinates: {', '.join(cone_coords)}")

        except Exception as e:
            rospy.logerr(f"Error in callback_cluster: {str(e)}")

    def handle_straight(self):
        self.logger.log('state', "State: STRAIGHT")
        
        cones_in_range = [cone for cone in self.ob if self.minx1 < cone.x < self.maxx1 and self.miny1 < cone.y < self.maxy1]
        cone_positions = [f"({cone.x:.2f}, {cone.y:.2f})" for cone in cones_in_range]
        self.logger.log('state', f"Transitioning to FULL_STEERING state. Cones in range: {', '.join(cone_positions)}")
        if cones_in_range:
            cone_positions = [f"({cone.x:.2f}, {cone.y:.2f})" for cone in cones_in_range]
            self.logger.log('state', f"Transitioning to FULL_STEERING state. Cones in range: {', '.join(cone_positions)}")
            self.state = State.FULL_STEERING
        else:
            self.full_steer_pub.publish(0)
            self.logger.log('state', "Remaining in STRAIGHT state. No cones in target range.")
        
    def handle_full_steering(self):
        self.logger.log('state', "State: FULL_STEERING")
        
        # 새로운 감지 영역 내 장애물 확인
        obstacles_in_new_range = [cone for cone in self.ob if self.minx2 < cone.x < self.maxx2 and self.miny2 < cone.y < self.maxy2]
        
        if obstacles_in_new_range:
            self.full_steer_pub.publish(1)
            self.logger.log('state', "Obstacles detected in new range. Publishing 1.")
        else:
            self.full_steer_pub.publish(2)
            self.logger.log('state', "No obstacles in new range. Publishing 2.")
        
        if 0 < self.cur_hd < 0.5:
            self.state = State.FINISH_TURN
            self.logger.log('state', "Transitioning to FINISH_TURN state")

    def handle_finish_turn(self):
        self.logger.log('state', "State: FINISH_TURN")
        self.full_steer_pub.publish(3)
        rospy.loginfo("UTurn complete, shutting down node.")
        rospy.signal_shutdown("UTurn complete")

    def publish_detection_range(self):
        marker = Marker()
        marker.header.frame_id = "velodyne"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "detection_range"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # 선의 두께

        # 기존 감지 영역 (빨간색)
        points_red = [
            Point(self.minx1, self.miny1, 0), Point(self.maxx1, self.miny1, 0),
            Point(self.maxx1, self.miny1, 0), Point(self.maxx1, self.maxy1, 0),
            Point(self.maxx1, self.maxy1, 0), Point(self.minx1, self.maxy1, 0),
            Point(self.minx1, self.maxy1, 0), Point(self.minx1, self.miny1, 0)
        ]

        # 새로운 감지 영역 (파란색)
        points_blue = [
            Point(self.minx2, self.miny2, 0), Point(self.maxx2, self.miny2, 0),
            Point(self.maxx2, self.miny2, 0), Point(self.maxx2, self.maxy2, 0),
            Point(self.maxx2, self.maxy2, 0), Point(self.minx2, self.maxy2, 0),
            Point(self.minx2, self.maxy2, 0), Point(self.minx2, self.miny2, 0)
        ]

        marker.points = points_red + points_blue

        # 색상 설정 (빨간색과 파란색)
        colors = [ColorRGBA(1, 0, 0, 1) for _ in range(8)]  # 빨간색
        colors += [ColorRGBA(0, 0, 1, 1) for _ in range(8)]  # 파란색
        marker.colors = colors

        self.marker_pub.publish(marker)

    def update(self):
        if self.section == 8:
            if self.state == State.STRAIGHT:
                self.handle_straight()
            elif self.state == State.FULL_STEERING:
                self.handle_full_steering()
            elif self.state == State.FINISH_TURN:
                self.handle_finish_turn()
            
            # 감지 범위 발행
            self.publish_detection_range()
        else:
            self.logger.log('section', f"Not in section 8. Current section: {self.section}")
        self.logger.publish_logs()

def main():
    rospy.init_node('uturn_controller')
    controller = UTurnController()
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        controller.update()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
