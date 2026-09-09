#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, math
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

def load_rddf(rddf_path):
    pts = []
    with open(rddf_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'): continue
            xs = line.split()
            if len(xs) < 2: continue
            x, y = float(xs[0]), float(xs[1])
            pts.append((x, y))
    return pts

class RDDFViz:
    def __init__(self):
        self.frame = rospy.get_param("~frame_id", "viz")
        self.rddf_path = rospy.get_param("~rddf_path")  # 꼭 넣어줘
        self.lookahead = rospy.get_param("~lookahead", 1.0)

        self.rddf = load_rddf(self.rddf_path)
        if not self.rddf:
            rospy.logerr("RDDF empty: %s", self.rddf_path)
            raise SystemExit

        self.pub_path = rospy.Publisher("rddf_path", Path, queue_size=1, latch=True)
        self.pub_mark = rospy.Publisher("rddf_markers", MarkerArray, queue_size=1)
        self.sub_pose = rospy.Subscriber("/pose_for_pp", PoseStamped, self.cb_pose, queue_size=10)

        self.publish_static_path()
        rospy.Timer(rospy.Duration(0.1), self.on_timer)  # 10Hz

        self.last_pose = None

    def publish_static_path(self):
        path = Path()
        path.header.frame_id = self.frame
        path.header.stamp = rospy.Time.now()
        for (x, y) in self.rddf:
            ps = PoseStamped()
            ps.header.frame_id = self.frame
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.pub_path.publish(path)

    def cb_pose(self, msg):
        self.last_pose = msg

    def nearest_idx(self, x, y):
        # 간단/빠른 구현: 선형 탐색(점 수가 많으면 필요시 kdtree로 교체)
        best_i, best_d2 = 0, float('inf')
        for i,(px,py) in enumerate(self.rddf):
            d2 = (px-x)*(px-x) + (py-y)*(py-y)
            if d2 < best_d2:
                best_d2, best_i = d2, i
        return best_i

    def on_timer(self, _):
        if self.last_pose is None: return
        x = self.last_pose.pose.position.x
        y = self.last_pose.pose.position.y

        # 최근접 + 룩어헤드 근사(아주 간단히 인덱스 전진)
        i0 = self.nearest_idx(x, y)
        # 경로 점 간 평균 간격이 크지 않다는 가정에서 룩어헤드 거리만큼 전진
        # 좀 더 정확히 하려면 호-거리 누적로직을 쓰면 됨.
        step = max(1, int(self.lookahead / 0.2))  # 0.2m 간격 가정, 필요시 파라미터화
        itgt = min(i0 + step, len(self.rddf)-1)
        tx, ty = self.rddf[itgt]

        ma = MarkerArray()

        # 최근접점 마커
        m0 = Marker()
        m0.header.frame_id = self.frame
        m0.header.stamp = rospy.Time.now()
        m0.ns = "rddf"
        m0.id = 1
        m0.type = Marker.SPHERE
        m0.action = Marker.ADD
        m0.pose.position.x = self.rddf[i0][0]
        m0.pose.position.y = self.rddf[i0][1]
        m0.pose.orientation.w = 1.0
        m0.scale.x = m0.scale.y = m0.scale.z = 0.3
        m0.color.r, m0.color.g, m0.color.b, m0.color.a = 1.0, 0.7, 0.0, 0.8  # 주황
        ma.markers.append(m0)

        # 타겟점 마커
        m1 = Marker()
        m1.header.frame_id = self.frame
        m1.header.stamp = m0.header.stamp
        m1.ns = "rddf"
        m1.id = 2
        m1.type = Marker.SPHERE
        m1.action = Marker.ADD
        m1.pose.position.x = tx
        m1.pose.position.y = ty
        m1.pose.orientation.w = 1.0
        m1.scale.x = m1.scale.y = m1.scale.z = 0.35
        m1.color.r, m1.color.g, m1.color.b, m1.color.a = 0.0, 0.8, 0.2, 0.9  # 초록
        ma.markers.append(m1)

        # 룩어헤드 원 (라인스트립)
        circle = Marker()
        circle.header.frame_id = self.frame
        circle.header.stamp = m0.header.stamp
        circle.ns = "rddf"
        circle.id = 3
        circle.type = Marker.LINE_STRIP
        circle.action = Marker.ADD
        circle.scale.x = 0.03
        circle.color.r, circle.color.g, circle.color.b, circle.color.a = 0.2, 0.5, 1.0, 0.8
        N = 60
        for k in range(N+1):
            th = 2.0*math.pi*k/N
            p = Point()
            p.x = x + self.lookahead * math.cos(th)
            p.y = y + self.lookahead * math.sin(th)
            circle.points.append(p)
        ma.markers.append(circle)

        self.pub_mark.publish(ma)

if __name__ == "__main__":
    rospy.init_node("rddf_visualizer")
    RDDFViz()
    rospy.spin()
