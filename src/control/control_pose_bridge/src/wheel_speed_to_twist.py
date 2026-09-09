#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
from erp42_msgs.msg import SerialFeedBack

class WheelSpeedToTwist:
    def __init__(self):
        self.pub = rospy.Publisher("/wheel/twist", TwistWithCovarianceStamped, queue_size=20)
        self.kph_to_mps = 1000.0/3600.0
        rospy.Subscriber("/erp42_serial/feedback", SerialFeedBack, self.cb, queue_size=50)

        # 파라미터(옵션): 공분산 값 (vx에만 소신호, 나머지 크게)
        self.vx_var = rospy.get_param("~vx_variance", 0.05)  # m^2/s^2 정도에서 시작

    def cb(self, msg):
        kph = float(getattr(msg, "speed", 0.0))  # 네가 준 필드명 speed(kph)
        mps = kph * self.kph_to_mps

        t = TwistWithCovarianceStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.twist.twist.linear.x = mps

        # covariance(6x6 row-major). vx만 신뢰, 나머지는 크게 둠.
        cov = [0.0]*36
        cov[0] = self.vx_var         # vx
        cov[7] = 1e6                 # vy
        cov[14] = 1e6                # vz
        cov[21] = 1e6                # wx
        cov[28] = 1e6                # wy
        cov[35] = 1e6                # wz
        t.twist.covariance = cov

        self.pub.publish(t)

if __name__ == "__main__":
    rospy.init_node("wheel_speed_to_twist")
    WheelSpeedToTwist()
    rospy.spin()
