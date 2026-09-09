#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32
from erp42_msgs.msg import DriveCmd  # 실제 패키지 메시지 타입에 맞게 수정 필요
import time

class TimedDriveNode:
    def __init__(self):
        # launch 파일에서 파라미터 가져오기
        self.speed = rospy.get_param("~speed", 0)
        self.steer = rospy.get_param("~steer", 0)
        self.duration = rospy.get_param("~duration", 3.0)  # 초 단위

        # 퍼블리셔 생성 (driveCMD 토픽, 메시지 타입은 실제 사용하는 걸로 변경)
        self.pub = rospy.Publisher("/erp42_serial/drive", DriveCmd, queue_size=10)

        rospy.loginfo("TimedDriveNode initialized with speed=%d, steer=%d, duration=%.2f",
                      self.speed, self.steer, self.duration)

    def run(self):
        msg = DriveCmd()
        msg.KPH = self.speed
        msg.Deg = self.steer
        msg.brake = 0

        start_time = time.time()
        rate = rospy.Rate(20)  # 20Hz

        while not rospy.is_shutdown():
            elapsed = time.time() - start_time

            if elapsed < self.duration:
                # duration 동안 입력 speed, steer 유지
                msg.KPH = self.speed
                msg.Deg = self.steer
                msg.brake = 0
            else:
                # 시간이 끝나면 멈추고 브레이크
                msg.KPH = 0
                msg.Deg = 0
                msg.brake = 200

            self.pub.publish(msg)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("timed_drive_node")
    node = TimedDriveNode()
    node.run()
