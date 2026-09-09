#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math as m
from collections import deque
from std_msgs.msg import String, Float32, Bool, Int32
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from object_detector.msg import ObjectInfo
from dynamic_reconfigure.msg import Config


# ---------- 시각화 유틸리티 ----------
def make_box_marker_xy(x_min, x_max, y_min, y_max, frame, ns, mid, rgba, z=0.0, lw=0.05):
    xs = [x_min, x_max, x_max, x_min, x_min]
    ys = [y_min, y_min, y_max, y_max, y_min]
    mkr = Marker()
    mkr.header.frame_id = frame
    mkr.header.stamp = rospy.Time.now()
    mkr.ns = ns
    mkr.id = mid
    mkr.type = Marker.LINE_STRIP
    mkr.action = Marker.ADD
    mkr.scale.x = lw
    mkr.color.r, mkr.color.g, mkr.color.b, mkr.color.a = rgba
    for x, y in zip(xs, ys):
        mkr.points.append(Point(x=x, y=y, z=z))
    return mkr


def make_points_marker(pts, frame, ns, mid, scale=0.12, rgba=(1, 1, 1, 1), z=0.0):
    mkr = Marker()
    mkr.header.frame_id = frame
    mkr.header.stamp = rospy.Time.now()
    mkr.ns = ns
    mkr.id = mid
    mkr.type = Marker.SPHERE_LIST
    mkr.action = Marker.ADD
    mkr.scale.x = mkr.scale.y = mkr.scale.z = scale
    mkr.color.r, mkr.color.g, mkr.color.b, mkr.color.a = rgba
    for x, y in pts:
        mkr.points.append(Point(x=x, y=y, z=z))
    return mkr


def make_text_marker(x, y, text, frame, ns, mid, z=0.35, scale=0.28, rgba=(1, 1, 1, 0.95)):
    mkr = Marker()
    mkr.header.frame_id = frame
    mkr.header.stamp = rospy.Time.now()
    mkr.ns = ns
    mkr.id = mid
    mkr.type = Marker.TEXT_VIEW_FACING
    mkr.action = Marker.ADD
    mkr.pose.position.x = x
    mkr.pose.position.y = y
    mkr.pose.position.z = z
    mkr.scale.z = scale
    mkr.color.r, mkr.color.g, mkr.color.b, mkr.color.a = rgba
    mkr.text = text
    return mkr


def y_center_half(y_min, y_max):
    half = 0.5 * (y_max - y_min)
    ctr = 0.5 * (y_max + y_min)
    return ctr, half


class GoStopNode(object):
    def __init__(self):
        rospy.init_node("hj_go_stop_lidar", anonymous=True)

        # ===== 프레임/ROI =====
        self.fixed_frame = rospy.get_param("~fixed_frame", "velodyne")
        self.roi_ns = rospy.get_param("~roi_ns", "/object_detector")
        self.roi = {
            "xMin": rospy.get_param(self.roi_ns + "/xMin", 0.0),
            "xMax": rospy.get_param(self.roi_ns + "/xMax", 8.0),
            "yMin": rospy.get_param(self.roi_ns + "/yMin", -2.0),
            "yMax": rospy.get_param(self.roi_ns + "/yMax", 2.0),
            "zMin": rospy.get_param(self.roi_ns + "/zMin", -0.5),
            "zMax": rospy.get_param(self.roi_ns + "/zMax", 2.0),
        }
        rospy.Subscriber(self.roi_ns + "/parameter_updates", Config, self.cb_roi_update, queue_size=1)
        self.roi_auto_refresh_hz = rospy.get_param("~roi_auto_refresh_hz", 2.0)
        if self.roi_auto_refresh_hz > 0:
            rospy.Timer(rospy.Duration(1.0 / self.roi_auto_refresh_hz), self.cb_roi_poll)

        self.use_roi_boxes = rospy.get_param("~use_roi_boxes", True)
        self.use_roi_as_slow_exact = rospy.get_param("~use_roi_as_slow_exact", True)

        # ===== 속도/입력 보정 =====
        self.v_normal = rospy.get_param("~v_normal", 20.0)
        self.v_slow = rospy.get_param("~v_slow", 10.0)
        self.v_stop = 0.0

        self.input_swap_xy = rospy.get_param("~input_swap_xy", False)
        self.input_flip_x = rospy.get_param("~input_flip_x", False)
        self.input_flip_y = rospy.get_param("~input_flip_y", False)
        self.input_yaw_deg = rospy.get_param("~input_yaw_offset_deg", 0.0)
        self.lidar_offset_x = rospy.get_param("~lidar_offset_x", 0.3)

        # ===== 시간적 안정화 =====
        self.min_frames_stop = rospy.get_param("~min_frames_stop", 3)
        self.min_frames_slow = rospy.get_param("~min_frames_slow", 2)
        self.decision_window = rospy.get_param("~decision_window", 5)
        self.stop_cooldown = rospy.get_param("~stop_cooldown", 10)

        # ===== 사이즈 게이트 =====
        self.min_cluster_size_stop = rospy.get_param("~min_cluster_size_stop", 80)
        self.min_cluster_size_slow = rospy.get_param("~min_cluster_size_slow", 40)
        self.enforce_size_filter = rospy.get_param("~enforce_size_filter", False)

        # ===== 최고점 z 기반 분류 파라미터 =====
        # 기본값을 0.3으로 낮춤(사람 >= 0.3, 드럼 < 0.3 가정)
        self.use_topz_gate = rospy.get_param("~use_topz_gate", True)
        self.z_thresh_ped = rospy.get_param("~z_thresh_ped", 0.3)
        self.z_min_drum = rospy.get_param("~z_min_drum", -0.5)
        self.z_max_drum = rospy.get_param("~z_max_drum", 0.5)
        self.topz_gate_policy = rospy.get_param("~topz_gate_policy", "safe")  # safe|loose (애매시 human)

        # ===== 시각화/타임아웃 =====
        self.viz_hz = rospy.get_param("~viz_hz", 10.0)
        self.input_timeout = rospy.get_param("~input_timeout", 0.8)

        # 내부 상태
        self.history = deque(maxlen=self.decision_window)
        self.stop_streak = 0
        self.slow_streak = 0
        self.cooldown_ctr = 0
        self.size_field_name = None

        self.latest_pts = []
        self.latest_action = "PASS"
        self.latest_topz = []        # 각 클러스터의 maxZ
        self.latest_classes = []     # "human_like"/"drum_like"/"unknown"
        self.latest_mission = "구분불가"  # 동적(사람)/정적(드럼)/구분불가
        self._last_dbg = {}
        self.last_obj_time = None

        # pubs/subs
        self.state_pub = rospy.Publisher("/drive_state", String, queue_size=1)
        self.stop_pub = rospy.Publisher("/stop_flag", Bool, queue_size=1)
        self.vel_pub = rospy.Publisher("/missionKPH", Float32, queue_size=1)
        self.mov_pub = rospy.Publisher("/moving_stop_cmd", Int32, queue_size=1)  # 1:감속, 2:정지, 3:재출발
        self.vis_pub = rospy.Publisher("/go_stop_vis", MarkerArray, queue_size=1)
        rospy.Subscriber("/object_info", ObjectInfo, self.cb_object_info, queue_size=10)

        # 초기 1회
        self.update_boxes_from_roi(force=True)
        self.publish_markers(self.latest_pts, self.latest_action, [], [], self.latest_mission)

        # 시각화 타이머
        rospy.Timer(rospy.Duration(1.0 / self.viz_hz), self.cb_viz_timer)

    # ---------- ROI 동적 반영 ----------
    def cb_roi_update(self, msg: Config):
        changed = False
        for d in msg.doubles:
            if d.name in self.roi and self.roi[d.name] != d.value:
                self.roi[d.name] = d.value
                changed = True
        if changed and self.use_roi_boxes:
            self.update_boxes_from_roi(force=True)
            self.publish_markers(self.latest_pts, self.latest_action, self.latest_topz, self.latest_classes, self.latest_mission)

    def cb_roi_poll(self, _evt):
        changed = False
        for k, old in self.roi.items():
            v = rospy.get_param(self.roi_ns + "/" + k, old)
            if v != old:
                self.roi[k] = v
                changed = True
        if changed and self.use_roi_boxes:
            self.update_boxes_from_roi(force=True)
            self.publish_markers(self.latest_pts, self.latest_action, self.latest_topz, self.latest_classes, self.latest_mission)

    # ---------- ROI → 박스 ----------
    def update_boxes_from_roi(self, force=False):
        if not self.use_roi_boxes and not force:
            return
        xMin, xMax = self.roi["xMin"], self.roi["xMax"]
        yMin, yMax = self.roi["yMin"], self.roi["yMax"]

        # SLOW 박스
        if self.use_roi_as_slow_exact:
            self.slow_x_min, self.slow_x_max = xMin, xMax
            self.slow_y_min, self.slow_y_max = yMin, yMax
        else:
            self.slow_x_min = max(0.0, xMin)
            self.slow_x_max = xMax
            y_half = max(0.8, max(abs(yMin), abs(yMax)))
            self.slow_y_min, self.slow_y_max = -y_half, +y_half

        # STOP 박스(보수 축소)
        self.stop_x_min = max(0.0, self.slow_x_min)
        self.stop_x_max = max(3.0, min(self.slow_x_max, self.slow_x_min + 6.0))
        ctr, half = y_center_half(self.slow_y_min, self.slow_y_max)
        half_stop = max(0.8, min(1.2, half))
        self.stop_y_min, self.stop_y_max = ctr - half_stop, ctr + half_stop

    # ---------- 입력 보정 ----------
    def preprocess_xy(self, x, y):
        if self.input_swap_xy:
            x, y = y, x
        if self.input_flip_x:
            x = -x
        if self.input_flip_y:
            y = -y
        if self.input_yaw_deg != 0.0:
            th = m.radians(self.input_yaw_deg)
            c, s = m.cos(th), m.sin(th)
            x, y = c * x - s * y, s * x + c * y
        x += self.lidar_offset_x
        return x, y

    # ---------- 클러스터 크기 ----------
    def get_cluster_size(self, msg, i):
        if self.size_field_name:
            arr = getattr(msg, self.size_field_name, None)
            if arr is not None and len(arr) > i:
                try:
                    return int(arr[i])
                except:
                    pass
        for name in ("clusterSize", "sizes", "pointCount", "numPoints"):
            arr = getattr(msg, name, None)
            if arr is not None and len(arr) > i:
                try:
                    val = int(arr[i])
                    if val >= 0:
                        self.size_field_name = name
                        return val
                except:
                    pass
        pts = getattr(msg, "points", None)
        if pts is not None and len(pts) > i:
            try:
                val = len(pts[i])
                if val >= 0:
                    self.size_field_name = "points(len)"
                    return val
            except:
                pass
        return 0

    # ---------- 최고점 z 추출 ----------
    def extract_top_z(self, msg, i):
        # 1) maxZ[]
        maxZ = getattr(msg, "maxZ", None)
        if maxZ is not None and len(maxZ) > i:
            try:
                return float(maxZ[i])
            except:
                pass
        # 2) points[i].z 중 최대
        pts = getattr(msg, "points", None)
        if pts is not None and len(pts) > i and len(pts[i]) > 0 and hasattr(pts[i][0], "z"):
            try:
                return float(max(p.z for p in pts[i]))
            except:
                pass
        return None

    def classify_by_topz(self, zmax):
        if (zmax is None) or (not self.use_topz_gate):
            return "unknown"
        if zmax >= self.z_thresh_ped:
            return "human_like"
        if self.z_min_drum <= zmax <= self.z_max_drum:
            return "drum_like"
        return "human_like" if self.topz_gate_policy == "safe" else "unknown"

    # ---------- 주 입력 콜백 ----------
    def cb_object_info(self, msg):
        self.last_obj_time = rospy.Time.now()

        # 1) 좌표 보정 + 전방만 채택
        pts, sizes, topzs, classes = [], [], [], []
        for i in range(msg.objectCounts):
            x = msg.centerX[i]
            y = msg.centerY[i]
            x, y = self.preprocess_xy(x, y)
            if x < 0.0:  # 전방만
                continue
            sz = self.get_cluster_size(msg, i)
            zmax = self.extract_top_z(msg, i)
            cls = self.classify_by_topz(zmax)

            pts.append((x, y))
            sizes.append(sz)
            topzs.append(zmax)
            classes.append(cls)

        # 2) 사이즈 게이트 여부
        size_field_known = (self.size_field_name is not None)
        use_size_gate = True if self.enforce_size_filter else size_field_known

        # 3) 카운팅(+ 클래스 정책)
        n_stop = n_slow = 0
        raw_stop = raw_slow = 0
        human_cnt = drum_cnt = unk_cnt = 0
        z_ge = z_lt = 0  # zmax 기준(0.3) 미션 구간 추정용

        for (x, y), sz, cls, zmax in zip(pts, sizes, classes, topzs):
            in_stop = self.in_box(x, y, self.stop_x_min, self.stop_x_max, self.stop_y_min, self.stop_y_max)
            in_slow = self.in_box(x, y, self.slow_x_min, self.slow_x_max, self.slow_y_min, self.slow_y_max)

            if cls == "human_like":
                human_cnt += 1
            elif cls == "drum_like":
                drum_cnt += 1
            else:
                unk_cnt += 1

            if zmax is not None:
                if zmax >= self.z_thresh_ped:
                    z_ge += 1
                else:
                    z_lt += 1

            if in_stop or in_slow:
                if in_stop:
                    raw_stop += 1
                if in_slow:
                    raw_slow += 1

            # STOP은 사람(human_like)만 인정
            stop_class_ok = (cls == "human_like")
            slow_class_ok = (cls in ("human_like", "drum_like"))

            if use_size_gate:
                if in_stop and stop_class_ok and sz >= self.min_cluster_size_stop:
                    n_stop += 1
                elif in_slow and slow_class_ok and sz >= self.min_cluster_size_slow:
                    n_slow += 1
            else:
                if in_stop and stop_class_ok:
                    n_stop += 1
                elif in_slow and slow_class_ok:
                    n_slow += 1

        # 3-1) 미션 구간 추정(0.3 기준)
        if (z_ge + z_lt) == 0:
            mission = "UNKNOWN"
        else:
            mission = "DYNAMIC" if z_ge > z_lt else "STATIC"

        # 4) 연속성/쿨다운
        frame_state = "PASS"
        if n_stop > 0:
            self.stop_streak += 1
            self.slow_streak = 0
            if self.cooldown_ctr == 0 and self.stop_streak >= self.min_frames_stop:
                frame_state = "STOP"
        else:
            self.stop_streak = 0
            if n_slow > 0:
                self.slow_streak += 1
                if self.slow_streak >= self.min_frames_slow:
                    frame_state = "SLOW"
            else:
                self.slow_streak = 0

        if self.cooldown_ctr > 0:
            self.cooldown_ctr -= 1
            if frame_state == "STOP":
                frame_state = "SLOW" if n_slow > 0 else "PASS"

        # 5) 다수결
        self.history.append(frame_state)
        counts = {s: self.history.count(s) for s in ("STOP", "SLOW", "PASS")}
        action = max(counts, key=counts.get)

        # STOP 해제 시 쿨다운 개시
        if len(self.history) >= 2 and self.history[-2] == "STOP" and action != "STOP":
            self.cooldown_ctr = max(self.cooldown_ctr, self.stop_cooldown)

        # 6) 퍼블리시
        tgt_kph = self.v_stop if action == "STOP" else (self.v_slow if action == "SLOW" else self.v_normal)
        self.state_pub.publish(action)
        self.stop_pub.publish(Bool(data=(action == "STOP")))
        self.vel_pub.publish(tgt_kph)

        # /moving_stop_cmd: 1(감속=SLOW), 2(정지=STOP), 3(재출발=그 외)
        mov_cmd = 2 if action == "STOP" else (1 if action == "SLOW" else 3)
        self.mov_pub.publish(Int32(data=mov_cmd))

        # 7) 시각화 캐시
        self.latest_pts = pts
        self.latest_action = action
        self.latest_topz = topzs
        self.latest_classes = classes
        self.latest_mission = mission
        self._last_dbg = dict(
            total=len(pts),
            raw_stop=raw_stop, raw_slow=raw_slow,
            n_stop=n_stop, n_slow=n_slow,
            size_field=self.size_field_name if size_field_known
            else ("NONE(auto-bypass)" if not self.enforce_size_filter else "NONE(ENFORCED)"),
            th_stop=self.min_cluster_size_stop, th_slow=self.min_cluster_size_slow,
            streakS=self.stop_streak, streakL=self.slow_streak,
            cd=self.cooldown_ctr,
            use_size_gate=use_size_gate,
            human=human_cnt, drum=drum_cnt, unknown=unk_cnt,
            z_ge=z_ge, z_lt=z_lt
        )

    # ---------- 시각화 타이머 ----------
    def cb_viz_timer(self, _evt):
        now = rospy.Time.now()
        timed_out = (self.last_obj_time is None) or ((now - self.last_obj_time).to_sec() > self.input_timeout)

        if timed_out:
            action = "PASS"
            pts, topzs, classes = [], [], []
            mission = "UNKNOWN"
            self._last_dbg = dict(
                total=0, raw_stop=0, raw_slow=0, n_stop=0, n_slow=0,
                size_field=self._last_dbg.get("size_field", "-"),
                th_stop=self.min_cluster_size_stop, th_slow=self.min_cluster_size_slow,
                streakS=0, streakL=0, cd=self.cooldown_ctr,
                use_size_gate=self._last_dbg.get("use_size_gate", False),
                human=0, drum=0, unknown=0,
                z_ge=0, z_lt=0
            )
        else:
            action = self.latest_action
            pts = self.latest_pts
            topzs = self.latest_topz
            classes = self.latest_classes
            mission = self.latest_mission

        self.publish_markers(pts, action, topzs, classes, mission)

    # ---------- 보조 ----------
    @staticmethod
    def in_box(x, y, x_min, x_max, y_min, y_max):
        return (x_min <= x <= x_max) and (y_min <= y <= y_max)

    # ---------- 시각화 퍼블리셔 ----------
    def publish_markers(self, pts, action, topzs, classes, mission):
        arr = MarkerArray()

        # 박스
        arr.markers.append(make_box_marker_xy(
            self.stop_x_min, self.stop_x_max, self.stop_y_min, self.stop_y_max,
            self.fixed_frame, "stop_box", 1001, (1, 0, 0, 0.85)
        ))
        arr.markers.append(make_box_marker_xy(
            self.slow_x_min, self.slow_x_max, self.slow_y_min, self.slow_y_max,
            self.fixed_frame, "slow_box", 1002, (1, 1, 0, 0.85)
        ))

        # 포인트: 클래스별 색
        if pts:
            pts_h = [p for p, c in zip(pts, classes) if c == "human_like"]
            pts_d = [p for p, c in zip(pts, classes) if c == "drum_like"]
            pts_u = [p for p, c in zip(pts, classes) if c == "unknown"]

            if pts_h:
                arr.markers.append(make_points_marker(pts_h, self.fixed_frame, "obs_human", 2001,
                                                      rgba=(1.0, 0.2, 0.2, 1.0)))  # red
            if pts_d:
                arr.markers.append(make_points_marker(pts_d, self.fixed_frame, "obs_drum", 2002,
                                                      rgba=(0.2, 0.5, 1.0, 1.0)))  # blue
            if pts_u:
                arr.markers.append(make_points_marker(pts_u, self.fixed_frame, "obs_unknown", 2003,
                                                      rgba=(0.7, 0.7, 0.7, 1.0)))  # gray

            # 각 클러스터 위에 zmax 텍스트
            tid = 4000
            for (x, y), zmax in zip(pts, topzs):
                txt = "zmax={:.2f}m".format(zmax) if (zmax is not None) else "zmax=?"
                arr.markers.append(make_text_marker(x, y, txt, self.fixed_frame, "topz_text", tid))
                tid += 1

        # 상태 텍스트
        ctr_s, _ = y_center_half(self.slow_y_min, self.slow_y_max)
        bottom_s = min(self.slow_y_min, self.slow_y_max)
        top_s = max(self.slow_y_min, self.slow_y_max)
        ctr_st, _ = y_center_half(self.stop_y_min, self.stop_y_max)

        st = Marker()
        st.header.frame_id = self.fixed_frame
        st.header.stamp = rospy.Time.now()
        st.ns = "state_text"
        st.id = 3001
        st.type = Marker.TEXT_VIEW_FACING
        st.action = Marker.ADD
        st.pose.position.x = max(self.slow_x_min, 0.5)
        st.pose.position.y = bottom_s - 0.6
        st.scale.z = 0.5
        st.color.a = 1.0
        st.color.r = st.color.g = st.color.b = 1.0
        st.text = "{}".format(self.latest_action)
        arr.markers.append(st)

        # 미션 구간 텍스트 (동적/정적/구분불가)
        ms = Marker()
        ms.header.frame_id = self.fixed_frame
        ms.header.stamp = rospy.Time.now()
        ms.ns = "mission_text"
        ms.id = 3002
        ms.type = Marker.TEXT_VIEW_FACING
        ms.action = Marker.ADD
        ms.pose.position.x = max(self.slow_x_min, 0.5)
        ms.pose.position.y = bottom_s + 10  # 원본 요청값 그대로 유지
        ms.scale.z = 0.5
        ms.color.a = 1.0
        ms.color.r = 0.5
        ms.color.g = 1.0
        ms.color.b = 1.0
        ms.text = "MISSION: {}".format(mission)
        arr.markers.append(ms)

        # 디버그 3줄(+ 클래스/임계 집계)
        dbg = self._last_dbg or {}
        t1 = "STOP n={}/raw{} th≥{} streak={} cd={}".format(
            dbg.get("n_stop", 0), dbg.get("raw_stop", 0), dbg.get("th_stop", "-"), dbg.get("streakS", 0), dbg.get("cd", 0)
        )
        t2 = "SLOW n={}/raw{} th≥{} streak={}".format(
            dbg.get("n_slow", 0), dbg.get("raw_slow", 0), dbg.get("th_slow", "-"), dbg.get("streakL", 0)
        )
        t3 = "size={} gate={} | cls H/D/U={} / {} / {} | z>=0.3 / z<0.3 = {} / {}".format(
            dbg.get("size_field", "-"), dbg.get("use_size_gate", False),
            dbg.get("human", 0), dbg.get("drum", 0), dbg.get("unknown", 0),
            dbg.get("z_ge", 0), dbg.get("z_lt", 0)
        )
        for tid, (x, y, text) in enumerate(
            [
                (0.5 * (self.stop_x_min + self.stop_x_max), ctr_st, t1),
                (0.5 * (self.slow_x_min + self.slow_x_max), top_s + 0.4, t2),
                (self.slow_x_min, bottom_s - 1.6, t3),
            ],
            start=3101,
        ):
            mkr = Marker()
            mkr.header.frame_id = self.fixed_frame
            mkr.header.stamp = rospy.Time.now()
            mkr.ns = "debug_text"
            mkr.id = tid
            mkr.type = Marker.TEXT_VIEW_FACING
            mkr.action = Marker.ADD
            mkr.pose.position.x = x
            mkr.pose.position.y = y
            mkr.pose.position.z = 0.2
            mkr.scale.z = 0.35
            mkr.color.a = 1.0
            mkr.color.r = 1.0
            mkr.color.g = 1.0
            mkr.color.b = 0.0
            mkr.text = text
            arr.markers.append(mkr)

        self.vis_pub.publish(arr)

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    GoStopNode().spin()
