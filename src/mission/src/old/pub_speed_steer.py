#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32
import argparse

def main():
    rospy.init_node("pub_speed_steer.py", anonymous=True)

    # CLI 인자: 코드에서 직접 숫자 박을 거면 이 부분 말고 아래 DEFAULT_* 값만 바꿔도 됨.
    parser = argparse.ArgumentParser()
    parser.add_argument("--kph", type=float, help="missionKPH 값 (km/h)")
    parser.add_argument("--deg", type=float, help="missionDeg 값 (deg)")
    parser.add_argument("--rate", type=float, default=10.0, help="퍼블리시 Hz (기본 10Hz)")
    parser.add_argument("--oneshot", action="store_true", help="한 번만 퍼블리시하고 종료")
    args = parser.parse_args(rospy.myargv()[1:])

    # 여기만 바꾸면 코드 내에서 고정값으로 사용 가능
    DEFAULT_KPH = 10.0
    DEFAULT_DEG = 8.0

    kph_val = args.kph if args.kph is not None else DEFAULT_KPH
    deg_val = args.deg if args.deg is not None else DEFAULT_DEG

    vel_pub = rospy.Publisher("/missionKPH", Float32, queue_size=1)
    deg_pub = rospy.Publisher("/missionDeg", Float32, queue_size=1)

    # latched 퍼블리셔가 필요하면 queue_size 대신 latch=True 옵션 사용 가능:
    # vel_pub = rospy.Publisher("/missionKPH", Float32, queue_size=1, latch=True)
    # deg_pub = rospy.Publisher("/missionDeg", Float32, queue_size=1, latch=True)

    rate = rospy.Rate(args.rate)

    # 퍼블리시 루프
    if args.oneshot:
        vel_pub.publish(kph_val)
        deg_pub.publish(deg_val)
        rospy.loginfo("Published once: missionKPH=%.3f, missionDeg=%.3f", kph_val, deg_val)
    else:
        rospy.loginfo("Start publishing @ %.1f Hz: missionKPH=%.3f, missionDeg=%.3f", args.rate, kph_val, deg_val)
        while not rospy.is_shutdown():
            vel_pub.publish(kph_val)
            deg_pub.publish(deg_val)
            rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

