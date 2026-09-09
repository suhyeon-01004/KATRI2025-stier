#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_srvs.srv import Empty, EmptyResponse
import math

class VizAnchor:
    def __init__(self):
        self.fixed_frame   = rospy.get_param("~fixed_frame",   "viz")   # RViz Fixed Frame
        self.map_frame     = rospy.get_param("~map_frame",     "utm")   # 현재 데이터 프레임
        self.pose_topic    = rospy.get_param("~pose_topic",    "/pose_for_pp")
        self.align_yaw     = rospy.get_param("~align_yaw",     False)   # 원점에서 차량 진행방향을 +X로 정렬할지
        self.pub_hz        = rospy.get_param("~publish_hz",    10.0)

        self.anchor_set = False
        self.ax = self.ay = 0.0
        self.ayaw = 0.0

        self.br = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.cb_pose, queue_size=1)
        self.srv = rospy.Service("~reset", Empty, self.on_reset)

        self.timer = rospy.Timer(rospy.Duration(1.0/self.pub_hz), self.on_timer)
        rospy.loginfo("viz_anchor: fixed_frame=%s, map_frame=%s, pose_topic=%s, align_yaw=%s",
                      self.fixed_frame, self.map_frame, self.pose_topic, str(self.align_yaw))

    def on_reset(self, _req):
        self.anchor_set = False
        rospy.loginfo("viz_anchor: anchor reset requested; will set on next pose")
        return EmptyResponse()

    @staticmethod
    def yaw_from_quat(q):
        import tf.transformations as t
        (r,p,y) = t.euler_from_quaternion([q.x,q.y,q.z,q.w])
        return y

    def cb_pose(self, msg: PoseStamped):
        if not self.anchor_set:
            self.ax = msg.pose.position.x
            self.ay = msg.pose.position.y
            self.ayaw = self.yaw_from_quat(msg.pose.orientation) if self.align_yaw else 0.0
            self.anchor_set = True
            rospy.loginfo("viz_anchor: anchor set x=%.3f y=%.3f yaw=%.2fdeg",
                          self.ax, self.ay, math.degrees(self.ayaw))

    def on_timer(self, _evt):
        if not self.anchor_set:
            return
        # parent=fixed(viz), child=map(utm)
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.fixed_frame   # parent
        t.child_frame_id  = self.map_frame     # child

        # viz->utm 변환: 평행이동 = 앵커, 회전 = 앵커 yaw(옵션)
        t.transform.translation.x = self.ax
        t.transform.translation.y = self.ay
        t.transform.translation.z = 0.0

        q = tf_conversions.transformations.quaternion_from_euler(0,0,self.ayaw)
        t.transform.rotation.x = q[0]; t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]; t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node("viz_anchor")
    VizAnchor()
    rospy.spin()
