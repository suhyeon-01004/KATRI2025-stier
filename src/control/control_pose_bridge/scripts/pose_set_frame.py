#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node('pose_set_frame')
    in_t  = rospy.get_param('~in',  '/utm')
    out_t = rospy.get_param('~out', '/gps_utm_pose')
    frame = rospy.get_param('~frame','utm')
    pub = rospy.Publisher(out_t, PoseStamped, queue_size=20)

    def cb(msg):
        # frame 교정
        msg.header.frame_id = frame
        # 쿼터니언이 0,0,0,0이면 RViz 에러 → w=1로 보정
        q = msg.pose.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            msg.pose.orientation.w = 1.0
        pub.publish(msg)

    rospy.Subscriber(in_t, PoseStamped, cb, queue_size=50)
    rospy.spin()

if __name__ == '__main__':
    main()
