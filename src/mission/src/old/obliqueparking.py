#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
obliqueparking.py — ERP42용 사선 주차(즉시 반응판)
- /object_info(ObjectInfo)를 읽어 부채꼴(기본 r=4.5 m, ±57°) 안이 비면 **즉시 정지**.
- 엣지(edge) 요구, 프레임 누적 없음. free가 들어온 그 루프에서 바로 0,0 + /start_park(True).
- 섹션이 3(LIDAR_ONLY)일 때만 동작.

토픽
- sub: /section(Int32), /object_info(object_detector/ObjectInfo)
- pub: /missionKPH(Float32), /missionDeg(Float32), /start_park(Bool, latch=True),
       /parking_status(String, latch=True), /parking_debug(String)

파라미터(~)
- parking_section:int = 3
- radius_m:float = 4.5
- center_deg:float = 0.0
- half_width_deg:float = 57.0
- search_kph:float = 3.0
- search_deg:float = 0.0
- dt:float = 0.05
- search_timeout_s:float = -1.0   # >0이면 탐색이 이 시간 초과 시 강제 정지(옵션)

주의
- object_detector 측 섹션3 부채꼴 반경도 4.5 m로 맞추어야 일관.
"""

import math
import time
from dataclasses import dataclass
from typing import Optional

import rospy
from std_msgs.msg import Int32, String, Float32, Bool
from object_detector.msg import ObjectInfo


@dataclass
class SectorParams:
    radius_m: float
    center_deg: float
    half_width_deg: float


@dataclass
class CommandParams:
    search_kph: float
    search_deg: float


class ObliqueParkingFromObjectInfoPP:
    def __init__(self):
        # ---- 파라미터 ----
        self.section_target = int(rospy.get_param("~parking_section", 3))
        self.sector = SectorParams(
            radius_m=float(rospy.get_param("~radius_m", 4.5)),
            center_deg=float(rospy.get_param("~center_deg", 0.0)),
            half_width_deg=float(rospy.get_param("~half_width_deg", 57.0)),
        )
        self.cmd = CommandParams(
            search_kph=float(rospy.get_param("~search_kph", 3.0)),
            search_deg=float(rospy.get_param("~search_deg", 0.0)),
        )
        self.dt = float(rospy.get_param("~dt", 0.05))
        self.search_timeout_s = float(rospy.get_param("~search_timeout_s", -1.0))

        # ---- 상태 ----
        self.section: int = -1
        self.last_obj: Optional[ObjectInfo] = None
        self.parked: bool = False
        self.t0_search = time.time()
        self._status_last: Optional[str] = None

        # ---- 통신 ----
        self.sub_section = rospy.Subscriber("/section", Int32, self.cb_section, queue_size=1)
        self.sub_obj     = rospy.Subscriber("/object_info", ObjectInfo, self.cb_obj, queue_size=1)

        self.pub_kph    = rospy.Publisher("/missionKPH", Float32, queue_size=10)
        self.pub_deg    = rospy.Publisher("/missionDeg", Float32, queue_size=10)
        self.pub_start  = rospy.Publisher("/start_park", Bool, queue_size=1, latch=True)
        self.pub_status = rospy.Publisher("/parking_status", String, queue_size=1, latch=True)
        self.pub_debug  = rospy.Publisher("/parking_debug", String, queue_size=10)

    # --------------------------- 콜백 ---------------------------
    def cb_section(self, msg: Int32):
        self.section = int(msg.data)

    def cb_obj(self, msg: ObjectInfo):
        self.last_obj = msg

    # --------------------------- 유틸 ---------------------------
    @staticmethod
    def _wrap_pi(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def in_sector(self, x: float, y: float) -> bool:
        """(x: 전방+, y: 좌측+) 점이 부채꼴 내부인지 검사"""
        r = math.hypot(x, y)
        if r > self.sector.radius_m:
            return False
        th = math.atan2(y, x)  # x: 전방(+), y: 좌측(+)
        th0 = math.radians(self.sector.center_deg)
        dth = abs(self._wrap_pi(th - th0))
        return dth <= math.radians(self.sector.half_width_deg)

    def classify(self) -> Optional[bool]:
        """True=free, False=blocked, None=입력없음"""
        if self.last_obj is None:
            return None
        n = int(getattr(self.last_obj, "objectCounts", 0))
        xs = getattr(self.last_obj, "centerX", [] )
        ys = getattr(self.last_obj, "centerY", [] )
        cnt = min(n, len(xs), len(ys))
        for i in range(cnt):
            try:
                if self.in_sector(float(xs[i]), float(ys[i])):
                    return False
            except Exception:
                continue
        return True

    def publish_status(self, txt: str):
        if txt != self._status_last:
            self.pub_status.publish(String(data=txt))
            self._status_last = txt

    def pub_pp_cmd(self, kph: float, deg: float):
        self.pub_kph.publish(Float32(data=float(kph)))
        self.pub_deg.publish(Float32(data=float(deg)))

    # --------------------------- 메인 루프 ---------------------------
    def run(self):
        rate = rospy.Rate(1.0 / self.dt)
        self.publish_status("초기화")
        self.pub_start.publish(Bool(data=False))
        self.t0_search = time.time()

        while not rospy.is_shutdown():
            # 1) 섹션 확인(필수)
            if self.section != self.section_target:
                self.publish_status(f"대기(섹션 {self.section} → {self.section_target} 필요)")
                self.pub_pp_cmd(0.0, 0.0)
                self.pub_start.publish(Bool(data=False))
                rate.sleep()
                continue

            # 2) 분류
            cls = self.classify()  # True=free, False=blocked, None=입력없음

            if cls is None:
                # 센서 입력 없음 → 탐색 유지
                self.publish_status("직진")
                self.pub_pp_cmd(self.cmd.search_kph, self.cmd.search_deg)
                self.pub_start.publish(Bool(data=False))
                self.pub_debug.publish(String(data="no_object_info → SEARCH"))

            elif cls is False:
                # 막힘 → 탐색 유지
                self.publish_status("직진")
                self.pub_pp_cmd(self.cmd.search_kph, self.cmd.search_deg)
                self.pub_start.publish(Bool(data=False))
                self.pub_debug.publish(String(data="blocked_in_sector → SEARCH"))

            else:
                # free → 즉시 정지 + start_park (한 번만 래치)
                if not self.parked:
                    self.publish_status("주차 진입")
                    self.pub_pp_cmd(0.0, 0.0)
                    self.pub_start.publish(Bool(data=True))
                    self.pub_debug.publish(String(data="free_detected → STOP+start_park"))
                    self.parked = True
                else:
                    # 이미 주차 상태 유지 시 계속 0,0 유지
                    self.publish_status("주차 유지")
                    self.pub_pp_cmd(0.0, 0.0)
                    self.pub_start.publish(Bool(data=True))

            # 3) 탐색 타임아웃(옵션)
            if self.search_timeout_s > 0 and (not self.parked):
                if (time.time() - self.t0_search) > self.search_timeout_s:
                    self.publish_status("탐색 타임아웃→정지")
                    self.pub_pp_cmd(0.0, 0.0)
                    self.pub_start.publish(Bool(data=True))
                    self.pub_debug.publish(String(data="search_timeout → STOP+start_park"))
                    self.parked = True

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("obliqueparking_from_object_info_pp")
    node = ObliqueParkingFromObjectInfoPP()
    node.run()
