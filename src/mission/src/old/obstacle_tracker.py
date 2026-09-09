#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math as m
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Header
from ublox_msgs.msg import NavPVT
from object_detector.msg import ObjectInfo
from geometry_msgs.msg import Point
from mission.msg import ObstacleArray

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

class ObstacleTracker:
    def __init__(self):
        self.obstacles = []
        self.threshold = 0.4
        self.cur_hd = 0.0
        self.cur_position_x = 0.0
        self.cur_position_y = 0.0
        self.section = 0
        self.lidar_gps_offset_x = 0.3
        self.detection_range_x = (0, 10)
        self.detection_range_y = (-5, 5)
        self.current_detected_obstacles = set()

        self.heading_kalman_filter = KalmanFilter(0.01, 0.1, 1.0, 0.0)

        rospy.init_node('obstacle_tracker')
        rospy.Subscriber('/section', Int32, self.callback_section)
        rospy.Subscriber('/utm', PoseStamped, self.callback_gps)
        rospy.Subscriber('/ublox_position_receiver/navpvt', NavPVT, self.callback_hd)
        rospy.Subscriber('/object_info', ObjectInfo, self.callback_cluster)
        self.marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        self.obstacle_pub = rospy.Publisher('/obstacle_points', ObstacleArray, queue_size=10)

        self.rate = rospy.Rate(20)

    def callback_section(self, msg):
        self.section = msg.data

    def callback_gps(self, msg):
        self.cur_position_x = msg.pose.position.x
        self.cur_position_y = msg.pose.position.y

    def callback_hd(self, msg):
        raw_heading = msg.heading * 1e-5 * m.pi / 180
        self.cur_hd = m.pi / 2 - self.heading_kalman_filter.update(raw_heading)

    def callback_cluster(self, msg):
        self.current_detected_obstacles.clear()
        for i in range(msg.objectCounts):
            abs_x, abs_y = self.relative_to_absolute(msg.centerX[i], msg.centerY[i])
            self.update_obstacle_list(abs_x, abs_y)
        self.update_obstacles()
        self.publish_obstacle_markers()
        self.publish_obstacles()

    def relative_to_absolute(self, rel_x, rel_y):
        corrected_rel_x = rel_x + self.lidar_gps_offset_x * m.cos(self.cur_hd)
        corrected_rel_y = rel_y + self.lidar_gps_offset_x * m.sin(self.cur_hd)
        abs_x = corrected_rel_x * m.cos(self.cur_hd) - corrected_rel_y * m.sin(self.cur_hd) + self.cur_position_x
        abs_y = corrected_rel_x * m.sin(self.cur_hd) + corrected_rel_y * m.cos(self.cur_hd) + self.cur_position_y
        return abs_x, abs_y
    
    def is_in_detection_range(self, rel_x, rel_y):
        return (self.detection_range_x[0] <= rel_x <= self.detection_range_x[1] and
                self.detection_range_y[0] <= rel_y <= self.detection_range_y[1])

    def update_obstacle_list(self, abs_x, abs_y):
        rel_x, rel_y = self.absolute_to_relative(abs_x, abs_y)
        if not self.is_in_detection_range(rel_x, rel_y):
            return

        merged = False
        for obs in self.obstacles:
            distance = m.sqrt((abs_x - obs['x']) ** 2 + (abs_y - obs['y']) ** 2)
            if distance < self.threshold:
                obs['x'], obs['y'] = abs_x, abs_y
                obs['confidence'] = min(1.0, obs['confidence'] + 0.2)
                self.current_detected_obstacles.add(id(obs))
                merged = True
                break
        
        if not merged:
            new_obstacle = {'x': abs_x, 'y': abs_y, 'confidence': 0.5}
            self.obstacles.append(new_obstacle)
            self.current_detected_obstacles.add(id(new_obstacle))

    def update_obstacles(self):
        for obs in self.obstacles:
            rel_x, rel_y = self.absolute_to_relative(obs['x'], obs['y'])
            if self.is_in_detection_range(rel_x, rel_y):
                if id(obs) not in self.current_detected_obstacles:
                    obs['confidence'] = max(0, obs['confidence'] - 0.4)

        self.obstacles = [obs for obs in self.obstacles if obs['confidence'] > 0.05]

    def publish_obstacle_markers(self):
        marker_array = MarkerArray()
        for i, obs in enumerate(self.obstacles):
            rel_x, rel_y = self.absolute_to_relative(obs['x'], obs['y'])
            marker = self.create_marker(i, rel_x, rel_y, 0.0, 1.0, 0.0)
            marker_array.markers.append(marker)
            text_marker = self.create_text_marker(i + 1000, rel_x, rel_y, obs['confidence'] * 100)
            marker_array.markers.append(text_marker)
        self.marker_pub.publish(marker_array)

    def create_marker(self, marker_id, x, y, r, g, b):
        marker = Marker()
        marker.header.frame_id = "velodyne"
        marker.header.stamp = rospy.Time.now()
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.5
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = r, g, b
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def create_text_marker(self, marker_id, x, y, probability):
        marker = Marker()
        marker.header.frame_id = "velodyne"
        marker.header.stamp = rospy.Time.now()
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = f"{probability:.1f}%"
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 1.0
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def absolute_to_relative(self, abs_x, abs_y):
        delta_x = abs_x - self.cur_position_x
        delta_y = abs_y - self.cur_position_y
        rel_x = delta_x * m.cos(self.cur_hd) + delta_y * m.sin(self.cur_hd)
        rel_y = -delta_x * m.sin(self.cur_hd) + delta_y * m.cos(self.cur_hd)
        rel_x -= self.lidar_gps_offset_x * m.cos(self.cur_hd)
        rel_y -= self.lidar_gps_offset_x * m.sin(self.cur_hd)
        return rel_x, rel_y

    def publish_obstacles(self):
        obstacle_array = ObstacleArray()
        obstacle_array.header = Header(stamp=rospy.Time.now())
        for obs in self.obstacles:
            point = Point(x=obs['x'], y=obs['y'], z=0.0)
            obstacle_array.points.append(point)
        self.obstacle_pub.publish(obstacle_array)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    tracker = ObstacleTracker()
    tracker.run()
