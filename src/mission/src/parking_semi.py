#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from dataclasses import dataclass
from typing import Optional
from pathlib import Path

import rospy
from std_msgs.msg import Int32, String, Bool
from object_detector.msg import ObjectInfo
import sys
sys.path.append(str(Path.home() / "catkin_ws/src/mission/src"))
from visualization_msgs.msg import Marker, MarkerArray  # 시각화를 위한 추가
from geometry_msgs.msg import Point  # 시각화를 위한 추가
from mission_utils import visualization_marker_array  # 시각화를 위한 추가

@dataclass
class SectorParams:
    radius_m: float
    center_deg: float
    half_width_deg: float

class ObliqueParkingFromObjectInfoPP:
    def __init__(self):
        # -------------------- 파라미터 --------------------
        self.section_target = int(rospy.get_param("~parking_section", 14))  # 기본 섹션 4

        # 기본을 '부채꼴'로. 진행축 기준 우측 57° → 중심 -28.5°, 반폭 28.5°
        self.region_mode = str(rospy.get_param("~region_mode", "sector")).lower()

        self.sector = SectorParams(
            radius_m=float(rospy.get_param("~radius_m", 5.5)),    # 기본 반경 4.5 m
            center_deg=float(rospy.get_param("~center_deg", -28.5)),
            half_width_deg=float(rospy.get_param("~half_width_deg", 28.5)),
        )
        self.dt = float(rospy.get_param("~dt", 0.05))

        # -------------------- 상태 --------------------
        self.section: int = -1
        self.last_obj: Optional[ObjectInfo] = None
        self._status_last: Optional[str] = None
        self._status_now: str = "초기화"

        self._last_start_msg: Optional[bool] = None
        self._start_sent: bool = False
        self.required_free_count = max(1, int(rospy.get_param("~free_confirmation", 3)))
        self.free_count: int = 0
        self.saw_blocked: bool = False

        # -------------------- 통신 --------------------
        self.sub_section = rospy.Subscriber("/section", Int32, self.cb_section, queue_size=1)
        self.sub_obj     = rospy.Subscriber("/object_info", ObjectInfo, self.cb_obj, queue_size=1)

        self.pub_start  = rospy.Publisher("/start_park_semi", Bool, queue_size=1, latch=True)
        self.pub_status = rospy.Publisher("/parking_status", String, queue_size=1, latch=True)

        self.roi_pub = rospy.Publisher("/roi_vis", MarkerArray, queue_size=1)  # 시각화를 위한 추가
        self._roi_tick = 0  # ROI 표시 주기 낮추기(선택) # 시각화를 위한 추가

        rospy.loginfo(f"[parking] section={self.section_target}, region={self.region_mode}, "
                      f"R={self.sector.radius_m}m, center={self.sector.center_deg}deg, "
                      f"half={self.sector.half_width_deg}deg")

    # --------------------------- 콜백 ---------------------------
    def cb_section(self, msg: Int32):
        self.section = int(msg.data)

    def cb_obj(self, msg: ObjectInfo):
        self.last_obj = msg

    # --------------------------- 유틸 ---------------------------
    @staticmethod
    def _wrap_pi(a: float) -> float:
        while a > math.pi: a -= 2.0 * math.pi
        while a < -math.pi: a += 2.0 * math.pi
        return a

    def in_sector(self, x: float, y: float) -> bool:
        r = math.hypot(x, y)
        if self.sector.radius_m > 0.0 and r > self.sector.radius_m: return False
        th = math.atan2(y, x)  # x: 전(+), y: 좌(+)
        th0 = math.radians(self.sector.center_deg)
        return abs(self._wrap_pi(th - th0)) <= math.radians(self.sector.half_width_deg)

    def in_right_half(self, x: float, y: float) -> bool:
        if x < 0.0 or y > 0.0: return False
        if self.sector.radius_m > 0.0 and math.hypot(x, y) > self.sector.radius_m: return False
        return True

    def classify(self) -> Optional[bool]:
        if self.last_obj is None: return None
        n  = int(getattr(self.last_obj, "objectCounts", 0))
        xs = getattr(self.last_obj, "centerX", [])
        ys = getattr(self.last_obj, "centerY", [])
        cnt = min(n, len(xs), len(ys))
        inside = self.in_right_half if self.region_mode == "right_half" else self.in_sector
        for i in range(cnt):
            try:
                if inside(float(xs[i]), float(ys[i])):
                    return False  # 막힘
            except Exception:
                continue
        return True  # 빈공간

    # --------------------------- 퍼블리시 / 로그 ---------------------------
    def publish_status(self, txt: str):
        self._status_now = txt
        if txt != self._status_last:
            self.pub_status.publish(String(data=txt))
            rospy.loginfo(f"[parking] 상태: {txt}")
            self._status_last = txt

    def publish_start_signal(self, value: bool):
        if value:
            if self._start_sent:
                return
            self.pub_start.publish(Bool(data=True))
            self._last_start_msg = True
            self._start_sent = True
        else:
            if self._start_sent:
                return
            if self._last_start_msg is None or self._last_start_msg:
                self.pub_start.publish(Bool(data=False))
                self._last_start_msg = False

    def reset_start_signal(self):
        self._start_sent = False
        self._last_start_msg = None


    # === ADD: ROI 도형 생성 & 퍼블리시 유틸 ===
    def _make_sector_outline(self, radius_m, center_deg, half_width_deg, n=64):
        import math
        th0 = math.radians(center_deg - half_width_deg)
        th1 = math.radians(center_deg + half_width_deg)
        ts  = [th0 + (th1 - th0) * i / (n - 1) for i in range(n)]
        arc = [(radius_m * math.cos(t), radius_m * math.sin(t)) for t in ts]
        # 중심→왼 경계, 호, 중심→오른 경계, 다시 중심
        outline = [(0.0, 0.0), arc[0]] + arc + [arc[-1], (0.0, 0.0)]
        return outline, arc

    def _make_right_half_outline(self, radius_m, n=64):
        import math
        th0, th1 = -math.pi/2, 0.0  # y<0(오른쪽), x>0(전방)
        ts  = [th0 + (th1 - th0) * i / (n - 1) for i in range(n)]
        arc = [(radius_m * math.cos(t), radius_m * math.sin(t)) for t in ts]
        # 중심→(R,0), 호(시계 반대→시계 방향 정렬용 역순), 중심→(0,-R), 다시 중심
        outline = [(0.0, 0.0), (radius_m, 0.0)] + arc[::-1] + [(0.0, -radius_m), (0.0, 0.0)]
        return outline, arc

    def _make_linestrip_marker(self, pts_xy, color=(255, 215, 0), width=0.05, lifetime=0.5, ns="roi_outline", mid=0):
        m = Marker()
        m.header.frame_id = "velodyne"
        m.header.stamp = rospy.Time.now()
        m.ns = ns
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = width
        m.color.r = float(color[0]) / 255.0
        m.color.g = float(color[1]) / 255.0
        m.color.b = float(color[2]) / 255.0
        m.color.a = 1.0
        m.lifetime = rospy.Duration(lifetime)
        m.points = [Point(x=px, y=py, z=0.0) for (px, py) in pts_xy]
        return m

    def _publish_roi(self):
        mode = getattr(self, "region_mode", "sector").lower()
        if mode == "right_half":
            outline, _ = self._make_right_half_outline(self.sector.radius_m)
        else:
            outline, _ = self._make_sector_outline(
                self.sector.radius_m, self.sector.center_deg, self.sector.half_width_deg
            )

        outline_marker = self._make_linestrip_marker(outline, color=(255, 215, 0), width=0.05, lifetime=0.5)

        points_array = None
        if self.last_obj is not None:
            try:
                n = int(getattr(self.last_obj, "objectCounts", 0))
                xs = getattr(self.last_obj, "centerX", [])
                ys = getattr(self.last_obj, "centerY", [])
                pts = []
                for i in range(min(n, len(xs), len(ys))):
                    pts.append((float(xs[i]), float(ys[i])))
                if pts:
                    points_array = visualization_marker_array(
                        pts,
                        color=(0, 180, 255),
                        lenx=0.3,
                        leny=0.3,
                        lenz=0.3,
                        marker_type_input='sphere',
                        duration=0.5,
                    )
            except Exception as exc:  # best-effort viz only
                rospy.logwarn_throttle(1.0, "ROI points viz failed: %s", str(exc))

        msg = MarkerArray()
        msg.markers.append(outline_marker)
        if points_array is not None:
            offset = len(msg.markers)
            for i, m in enumerate(points_array.markers):
                m.id = i + offset
                m.ns = "roi_points"
                m.header.frame_id = "velodyne"
                msg.markers.append(m)
        self.roi_pub.publish(msg)


    # --------------------------- 메인 루프 ---------------------------
    def run(self):
        rate = rospy.Rate(1.0 / self.dt)
        self.publish_status("초기화")
        self.publish_start_signal(False)

        while not rospy.is_shutdown():
            if self.section != self.section_target:
                self.publish_status(f"대기(섹션 {self.section} → {self.section_target} 필요)")
                self.free_count = 0
                self.saw_blocked = False
                if self._start_sent:
                    self.reset_start_signal()
                self.publish_start_signal(False)
                rate.sleep()
                continue

            # ROI 시각화는 같은 섹션일 때만 주기적으로 실행
            self._roi_tick = (self._roi_tick + 1) % 2  # 루프가 10Hz라면 5Hz로
            if self._roi_tick == 0:
                try:
                    self._publish_roi()
                except Exception as exc:
                    rospy.logwarn_throttle(1.0, "ROI viz failed: %s", str(exc))

            cls = self.classify()  # True=free, False=blocked, None=입력없음
            if cls is None:
                self.publish_status("센서 데이터 대기")
                self.free_count = 0
                self.publish_start_signal(False)
            elif cls is False:
                self.publish_status("장애물 감지")
                self.free_count = 0
                self.saw_blocked = True
                self.publish_start_signal(False)
            else:
                if not self.saw_blocked:
                    self.publish_status("빈 공간 감지(선행 장애물 없음)")
                    self.free_count = 0
                    self.publish_start_signal(False)
                else:
                    self.free_count += 1
                    if self.free_count >= self.required_free_count:
                        self.publish_status("주차 공간 확정")
                        self.publish_start_signal(True)
                    else:
                        self.publish_status("주차 공간 감지(확인 중)")
                        self.publish_start_signal(False)

            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("parking_semi")
    node = ObliqueParkingFromObjectInfoPP()
    node.run()
