#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool
from ublox_msgs.msg import NavSTATUS  # 환경에 따라 타입명이 NavSTATUS/NavStatus 다를 수 있음

class GPSQualityMonitor:
    def __init__(self):
        self.req_ok_samples = rospy.get_param("~req_ok_samples", 5)
        self.req_bad_samples = rospy.get_param("~req_bad_samples", 3)
        self.flags2_threshold = rospy.get_param("~flags2_threshold", 72)
        self.fixstat_required = rospy.get_param("~fixstat_required", 3)

        self.ok_count = 0
        self.bad_count = 0
        self.gps_ok = False

        self.pub = rospy.Publisher("/gps_ok", Bool, queue_size=1, latch=True)
        self.pub.publish(Bool(self.gps_ok))

        rospy.Subscriber("/ublox_position_receiver/navstatus", NavSTATUS, self.cb, queue_size=20)

    def cb(self, msg):
        # rosmsg show ublox_msgs/NavSTATUS 로 실제 필드명 확인 필요
        fixStat = getattr(msg, "fixStat", 0)
        flags2  = getattr(msg, "flags2", 0)

        if (fixStat == self.fixstat_required) and (flags2 >= self.flags2_threshold):
            self.ok_count += 1
            self.bad_count = 0
            if not self.gps_ok and self.ok_count >= self.req_ok_samples:
                self.gps_ok = True
                self.pub.publish(Bool(True))
        else:
            self.bad_count += 1
            self.ok_count = 0
            if self.gps_ok and self.bad_count >= self.req_bad_samples:
                self.gps_ok = False
                self.pub.publish(Bool(False))

if __name__ == "__main__":
    rospy.init_node("gps_quality_monitor")
    GPSQualityMonitor()
    rospy.spin()
