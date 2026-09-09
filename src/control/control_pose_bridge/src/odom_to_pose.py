#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class OdomToPose:
    def __init__(self):
        in_topic  = rospy.get_param("~in", "/gps_utm_odom")
        out_topic = rospy.get_param("~out", "/gps_utm_pose")
        self.pub = rospy.Publisher(out_topic, PoseStamped, queue_size=10)
        rospy.Subscriber(in_topic, Odometry, self.cb, queue_size=10)

    def cb(self, odom):
        pose = PoseStamped()
        pose.header = odom.header
        pose.header.frame_id = "utm"
        pose.pose = odom.pose.pose
        self.pub.publish(pose)

if __name__ == "__main__":
    rospy.init_node("odom_to_pose")
    OdomToPose()
    rospy.spin()
