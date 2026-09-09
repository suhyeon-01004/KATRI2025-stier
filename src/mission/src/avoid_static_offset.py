#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import threading
from typing import Optional, List, Tuple

from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point as GPoint
from std_msgs.msg import Float32, Header, ColorRGBA, Int32
from mission.msg import PathInfo
from object_detector.msg import ObjectInfo

# ========================= 공용 유틸 =========================
FIXED_FRAME = "velodyne"
PATH_NUM_POINTS = 20

# ▼▼▼ 기본 오른쪽 오프셋. 장애물로 꺾이는 구간에는 적용하지 않음.
CENTER_RIGHT_OFFSET_D = 0.8  # [m]

def wrap_to_pi(a): return (a + math.pi) % (2.0 * math.pi) - math.pi
def clamp(n, lo, hi): return max(lo, min(hi, n))
def finite(x: float) -> bool: return (x == x) and (abs(x) != float('inf'))
def angle_in_sector(theta, center, half_width):
    theta, center = wrap_to_pi(theta), wrap_to_pi(center)
    return abs(wrap_to_pi(theta - center)) <= half_width

def inside_aabb(px, py, pz, cx, cy, cz, sx, sy, sz) -> bool:
    hx, hy, hz = sx * 0.5, sy * 0.5, sz * 0.5
    return (cx - hx <= px <= cx + hx) and (cy - hy <= py <= cy + hy) and (cz - hz <= pz <= cz + hz)

def hdr() -> Header:
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = FIXED_FRAME
    return h

def del_marker(ns, mid=0, delete_all=False):
    m = Marker(); m.header = hdr(); m.ns = ns; m.id = mid
    m.action = Marker.DELETEALL if delete_all else Marker.DELETE
    return m

def marker_line(pA_xy, pB_xy, color_rgba, ns, mid=0, width=0.08, z=0.0):
    m = Marker(); m.header = hdr(); m.ns = ns; m.id = mid
    m.type = Marker.LINE_STRIP; m.action = Marker.ADD
    m.scale.x = width
    m.color.r, m.color.g, m.color.b, m.color.a = color_rgba
    m.points = [GPoint(x=float(pA_xy[0]), y=float(pA_xy[1]), z=z),
                GPoint(x=float(pB_xy[0]), y=float(pB_xy[1]), z=z)]
    return m

# =============== 공유 저장소들 ===============
class ObstacleStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._boxes = []  # [(minx, maxx, miny, maxy), ...]
    def set_boxes(self, boxes_xy):
        with self._lock: self._boxes = list(boxes_xy)
    def get_boxes(self):
        with self._lock: return list(self._boxes)

OBST_STORE = ObstacleStore()

class LaneStore:
    """((rx1,ry1),(rx2,ry2)), ((lx1,ly1),(lx2,ly2)) in FIXED_FRAME"""
    def __init__(self):
        self._lock = threading.Lock()
        self._right = None
        self._left  = None
    def set_lanes(self, right, left):
        with self._lock:
            self._right, self._left = right, left
    def get_lanes(self):
        with self._lock:
            return self._right, self._left

LANE_STORE = LaneStore()

# ==================== 섹션 관리 ====================
_SECTION_LOCK = threading.Lock()
_CURRENT_SECTION = None
_SECTION_ACTIVE_ID = 9
_SECTION_LISTENERS = []


def _notify_section_listeners(active: bool, value: Optional[int]):
    for cb in list(_SECTION_LISTENERS):
        try:
            cb(active, value)
        except Exception as exc:
            rospy.logwarn("Section listener error: %s", exc)


def register_section_listener(callback):
    if callback not in _SECTION_LISTENERS:
        _SECTION_LISTENERS.append(callback)


def set_current_section(value: Optional[int]):
    global _CURRENT_SECTION
    prev_active = is_section_active()
    with _SECTION_LOCK:
        _CURRENT_SECTION = value
    new_active = is_section_active()
    if prev_active != new_active:
        _notify_section_listeners(new_active, value)


def get_current_section() -> Optional[int]:
    with _SECTION_LOCK:
        return _CURRENT_SECTION


def is_section_active() -> bool:
    current = get_current_section()
    return current == _SECTION_ACTIVE_ID


def section_cb(msg: Int32):
    
    try:
        value = int(getattr(msg, "data", None))
    except (TypeError, ValueError):
        value = None
    set_current_section(value)


# ==================== 0) 2D CV 칼만 필터 ====================
class Kalman2D:
    """ 상태: [x, y, vx, vy]^T / 측정: [x, y] / 모델: CV """
    def __init__(self, q_pos=0.05, q_vel=0.5, r_meas=0.04):
        self.initialized = False
        self.q_pos, self.q_vel, self.r_meas = float(q_pos), float(q_vel), float(r_meas)
        self.x = np.zeros((4,1), dtype=np.float64)
        self.P = np.eye(4, dtype=np.float64) * 1e3
        self.H = np.array([[1,0,0,0],[0,1,0,0]], dtype=np.float64)
        self.R = np.eye(2, dtype=np.float64) * self.r_meas
    def _Q(self, dt: float) -> np.ndarray:
        dt2 = dt*dt
        qx = np.array([[0.25*dt2*dt2, 0, 0.5*dt2, 0],
                       [0, 0.25*dt2*dt2, 0, 0.5*dt2],
                       [0.5*dt2, 0, dt, 0],
                       [0, 0.5*dt2, 0, dt]], dtype=np.float64)
        S = np.diag([self.q_pos, self.q_pos, self.q_vel, self.q_vel])
        return qx @ S @ qx.T
    def predict(self, dt: float):
        F = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]], dtype=np.float64)
        self.x = F @ self.x; self.P = F @ self.P @ F.T + self._Q(dt)
    def update(self, z: np.ndarray):
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
    def process(self, meas_xy: Tuple[float,float], dt: float) -> Tuple[float,float]:
        mx, my = float(meas_xy[0]), float(meas_xy[1])
        if not self.initialized:
            self.x[:] = np.array([[mx],[my],[0.0],[0.0]], dtype=np.float64)
            self.P[:] = np.eye(4, dtype=np.float64)
            self.initialized = True
            return mx, my
        self.predict(max(1e-3, dt))
        self.update(np.array([[mx],[my]], dtype=np.float64))
        return float(self.x[0,0]), float(self.x[1,0])

# ==================== 1) 라이다 선형화 + 차선 ====================
class VelodyneFilterAndLanes:
    def __init__(self):
        self.is_section_active = is_section_active
        gp = rospy.get_param
        # 절대 토픽명
        self.in_topic    = gp("~in_topic",  "/velodyne_points")
        self.out_topic   = gp("~out_topic", "/velodyne_points_filtered")
        self.base_topic  = gp("~base_topic","/velodyne_points_line")
        self.lanes_topic = gp("~lanes_topic","/lanes_markers")
        self.lat_topic   = gp("~lateral_offset_topic","/lateral_offset")

        # ROI / 각도
        self.z_min = float(gp("~z_min", -0.2)); self.z_max = float(gp("~z_max",  0.2))
        self.x_min = float(gp("~x_min", -5.0)); self.x_max = float(gp("~x_max",  5.0))
        self.y_min = float(gp("~y_min", -8.0)); self.y_max = float(gp("~y_max", -3.0))
        center_deg = float(gp("~ang_center_deg", -90.0))
        width_deg  = float(gp("~ang_width_deg",   90.0))
        self.center_rad, self.half_width_rad = math.radians(center_deg), math.radians(width_deg * 0.5)

        # 좌표계 보정
        self.swap_xy  = bool(gp("~swap_xy",  False))
        self.invert_x = bool(gp("~invert_x", False))
        self.invert_y = bool(gp("~invert_y", False))
        self.skip_nans = bool(gp("~skip_nans", True))

        # 차선/표시
        self.min_points_for_line = int(gp("~min_points_for_line", 30))
        self.line_auto_length    = bool(gp("~line_auto_length", True))
        self.line_fixed_half_len = float(gp("~line_fixed_half_len", 10.0))
        self.left_offset_m  = float(gp("~left_offset_m",  6.7)) # 조종해야 하는 파라미터 : 차선 오프셋
        self.right_offset_m = float(gp("~right_offset_m", 3.0)) # 조종해야 하는 파라미터 : 차선 오프셋
        self.extend_x_min = float(gp("~extend_x_min", -5.0))
        self.extend_x_max = float(gp("~extend_x_max",  5.0))

        # 차량 제원(표시/ROI 참고)
        self.veh_length = float(gp('~veh_length', 2.020))
        self.veh_width  = float(gp('~veh_width',  1.160))

        # Pub/Sub
        self.sub        = rospy.Subscriber(self.in_topic, PointCloud2, self.callback, queue_size=1)
        self.pub_pc     = rospy.Publisher(self.out_topic, PointCloud2, queue_size=1)
        self.pub_base   = rospy.Publisher(self.base_topic, Marker, queue_size=10, latch=True)
        self.pub_lanes  = rospy.Publisher(self.lanes_topic, MarkerArray, queue_size=10, latch=True)
        self.pub_lat    = rospy.Publisher(self.lat_topic, Float32, queue_size=10)

        # 출력 필드
        self.out_fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # 초기 삭제
        self.pub_base.publish(del_marker("lane_base", delete_all=True))
        self.pub_lanes.publish(MarkerArray(markers=[del_marker("lanes", 1), del_marker("lanes", 2)]))

    def project_xy(self, x, y):
        if self.invert_x: x = -x
        if self.invert_y: y = -y
        if self.swap_xy:  x, y = y, x
        return x, y
    def unproject_xy(self, xg, yg):
        if self.swap_xy:  xg, yg = yg, xg
        if self.invert_y: yg = -yg
        if self.invert_x: xg = -xg
        return xg, yg

    def _iter_points_xyzI(self, msg):
        try:
            it = pc2.read_points(msg, field_names=("x","y","z","intensity"), skip_nans=self.skip_nans)
            for x,y,z,i in it: yield x,y,z,float(i)
        except Exception:
            it = pc2.read_points(msg, field_names=("x","y","z"), skip_nans=self.skip_nans)
            for x,y,z in it:   yield x,y,z,0.0

    @staticmethod
    def fit_line_pca(points_xy):
        mean = points_xy.mean(axis=0)
        centered = points_xy - mean
        cov = centered.T @ centered / max(len(points_xy)-1, 1)
        vals, vecs = np.linalg.eigh(cov)
        d = vecs[:, np.argmax(vals)]
        return mean, d / (np.linalg.norm(d) + 1e-12)

    def forwardize_direction(self, d_xy, p_ref_xy):
        eps = 1e-3
        a_wx, _ = self.unproject_xy(p_ref_xy[0], p_ref_xy[1])
        b_xy = p_ref_xy + eps * d_xy
        b_wx, _ = self.unproject_xy(b_xy[0], b_xy[1])
        return -d_xy if (b_wx - a_wx) < 0.0 else d_xy

    def endpoints_by_original_x(self, p0_xy, d_xy, x_min_orig, x_max_orig):
        d = d_xy / (np.linalg.norm(d_xy) + 1e-12); p0 = p0_xy
        use_axis = 0 if not self.swap_xy else 1
        ox = (lambda v: -v) if self.invert_x else (lambda v: v)
        val_min, val_max = ox(x_min_orig), ox(x_max_orig)
        comp = d[use_axis]; eps = 1e-8
        if abs(comp) < eps:
            t_min, t_max = -self.line_fixed_half_len, self.line_fixed_half_len
        else:
            t_min = (val_min - p0[use_axis]) / comp
            t_max = (val_max - p0[use_axis]) / comp
            if t_min > t_max: t_min, t_max = t_max, t_min
        return p0 + t_min * d, p0 + t_max * d

    def callback(self, msg: PointCloud2):
        # 게이트 체크: 3일 때만 동작
        if not self.is_section_active():
            return

        zmin, zmax = self.z_min, self.z_max
        xmin, xmax = self.x_min, self.x_max
        ymin, ymax = self.y_min, self.y_max
        c, hw      = self.center_rad, self.half_width_rad

        filtered_points, xy_for_fit = [], []
        for x,y,z,intensity in self._iter_points_xyzI(msg):
            if not (zmin <= z <= zmax): continue
            xg, yg = self.project_xy(x, y)
            if not (xmin <= xg <= xmax and ymin <= yg <= ymax): continue
            if not angle_in_sector(math.atan2(yg, xg), c, hw): continue
            filtered_points.append([x, y, z, intensity]); xy_for_fit.append([xg, yg])

        self.pub_pc.publish(pc2.create_cloud(msg.header, self.out_fields, filtered_points))

        if len(xy_for_fit) < self.min_points_for_line:
            self.pub_base.publish(del_marker("lane_base"))
            self.pub_lanes.publish(MarkerArray(markers=[del_marker("lanes",1), del_marker("lanes",2)]))
            return

        xy = np.asarray(xy_for_fit, dtype=np.float32)
        p0, d = self.fit_line_pca(xy); d = self.forwardize_direction(d, p0)

        # 기준선
        if self.line_auto_length and len(xy) >= 2:
            t_vals = (xy - p0) @ d
            baseA, baseB = p0 + float(np.min(t_vals)) * d, p0 + float(np.max(t_vals)) * d
        else:
            baseA, baseB = p0 - self.line_fixed_half_len * d, p0 + self.line_fixed_half_len * d

        x1,y1 = self.unproject_xy(baseA[0], baseA[1])
        x2,y2 = self.unproject_xy(baseB[0], baseB[1])
        self.pub_base.publish(marker_line((x1,y1), (x2,y2), (1.0,0.0,0.0,1.0), "lane_base", mid=0, width=0.08))

        # 좌/우 차선(+y가 좌측)
        n_left = np.array([-d[1], d[0]], dtype=np.float32); n_left /= (np.linalg.norm(n_left) + 1e-12)
        p0_right = p0 + self.right_offset_m * n_left
        p0_left  = p0 + self.left_offset_m  * n_left
        rA, rB = self.endpoints_by_original_x(p0_right, d, self.extend_x_min, self.extend_x_max)
        lA, lB = self.endpoints_by_original_x(p0_left,  d, self.extend_x_min, self.extend_x_max)
        r1,r2 = self.unproject_xy(rA[0], rA[1]), self.unproject_xy(rB[0], rB[1])
        l1,l2 = self.unproject_xy(lA[0], lA[1]), self.unproject_xy(lB[0], lB[1])

        right_mk = marker_line(r1, r2, (1.0,1.0,1.0,1.0), "lanes", mid=1, width=0.08)
        left_mk  = marker_line(l1, l2, (1.0,1.0,0.0,1.0), "lanes", mid=2, width=0.08)
        self.pub_lanes.publish(MarkerArray(markers=[right_mk, left_mk]))

        # ▶ 차선 공유 (바운딩박스 게이트에서 사용)
        LANE_STORE.set_lanes((r1, r2), (l1, l2))

# ================= 2) ObjectInfo → 바운딩박스 + 장애물 업데이트 ==================
class ObjectInfoBBoxViz:
    def __init__(self):
        self.is_section_active = is_section_active
        gp = rospy.get_param
        self.object_topic = gp('~object_topic', '/object_info')
        self.output_topic = gp('~output_topic', '/cluster_bboxes')
        self.use_wireframe = gp('~use_wireframe', True)
        self.color_rgba = [float(v) for v in gp('~color_rgba', [0.0, 1.0, 0.0, 1.0])]
        self.color = ColorRGBA(*self.color_rgba)
        self.line_width = float(gp('~line_width', 0.03))
        self.lifetime = float(gp('~lifetime', 0.0))
        self.max_markers = int(gp('~max_markers', 500))
        self.clear_timeout = float(gp('~cluster_clear_timeout', 1.5)) # 조종해야 하는 파라미터 : 장애물 인식 시간
        self.timer_period = float(gp('~cluster_monitor_period', 0.1))

        # 내부(차량) ROI
        self.veh_length = float(gp('~veh_length', 2.020))
        self.veh_width  = float(gp('~veh_width',  1.160))
        self.veh_height = float(gp('~veh_height', 0.520))
        self.veh_off_x  = float(gp('~veh_offset_x', -0.6))
        self.veh_off_y  = float(gp('~veh_offset_y', 0.0))
        self.veh_off_z  = float(gp('~veh_offset_z', 0.0))
        self.veh_mar_x  = float(gp('~veh_margin_x', 0.05))
        self.veh_mar_y  = float(gp('~veh_margin_y', 0.05))
        self.veh_mar_z  = float(gp('~veh_margin_z', 0.05))

        # ▶ 차선 게이트 옵션
        self.only_inside_lanes = bool(gp('~only_inside_lanes', True))
        self.lane_gate_margin  = float(gp('~lane_gate_margin', 0.05))  # m

        self.pub = rospy.Publisher(self.output_topic, MarkerArray, queue_size=10, latch=True)
        self.sub = rospy.Subscriber(self.object_topic, ObjectInfo, self.cb, queue_size=5)
        self.sub_human = rospy.Subscriber('/dynamic_human_detected', Int32, self.cb_human, queue_size=1)
        self._last_cb_time = None
        self._has_boxes = False
        self._human_detected = False
        rospy.Timer(rospy.Duration(max(0.02, self.timer_period)), self._cb_timer)

        # 초기 전체 삭제
        arr = MarkerArray()
        m = Marker(); m.header = hdr(); m.ns = 'cluster_bbox'; m.id = 0; m.action = Marker.DELETEALL
        arr.markers = [m]; self.pub.publish(arr)

    @staticmethod
    def _g(seq, idx, default=float('nan')):
        try: return seq[idx]
        except Exception: return default

    def _wire(self, cid, center, size) -> Marker:
        cx, cy, cz = center; dx, dy, dz = size
        hx, hy, hz = max(dx*0.5, 1e-3), max(dy*0.5, 1e-3), max(dz*0.5, 1e-3)
        c = [(cx-hx, cy-hy, cz-hz), (cx+hx, cy-hy, cz-hz), (cx+hx, cy+hy, cz-hz), (cx-hx, cy+hy, cz-hz),
             (cx-hx, cy-hy, cz+hz), (cx+hx, cy-hy, cz+hz), (cx+hx, cy+hy, cz+hz), (cx-hx, cy+hy, cz+hz)]
        e = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
        m = Marker(); m.header = hdr(); m.ns='cluster_bbox'; m.id=cid
        m.type = Marker.LINE_LIST; m.action = Marker.ADD
        m.scale.x = max(self.line_width, 1e-4); m.color = self.color
        m.lifetime = rospy.Duration(self.lifetime)
        for a,b in e:
            m.points.append(GPoint(*c[a])); m.points.append(GPoint(*c[b]))
        return m

    def _solid(self, cid, center, size) -> Marker:
        cx, cy, cz = center; dx, dy, dz = size
        m = Marker(); m.header = hdr(); m.ns='cluster_bbox'; m.id=cid
        m.type = Marker.CUBE; m.action = Marker.ADD
        m.pose.position.x = cx; m.pose.position.y = cy; m.pose.position.z = cz
        m.pose.orientation.w = 1.0
        m.scale.x = max(dx,1e-3); m.scale.y = max(dy,1e-3); m.scale.z = max(dz,1e-3)
        m.color = self.color; m.lifetime = rospy.Duration(self.lifetime)
        return m

    def inside_vehicle_roi(self, x, y, z) -> bool:
        cx, cy, cz = self.veh_off_x, self.veh_off_y, self.veh_off_z
        sx = self.veh_length + 2.0*self.veh_mar_x
        sy = self.veh_width  + 2.0*self.veh_mar_y
        sz = self.veh_height + 2.0*self.veh_mar_z
        return inside_aabb(x, y, z, cx, cy, cz, sx, sy, sz)

    # ===== 차선 게이트용 보조 =====
    @staticmethod
    def _project_point_to_segment(p, a, b):
        ap = np.array([p[0]-a[0], p[1]-a[1]], dtype=np.float32)
        ab = np.array([b[0]-a[0], b[1]-a[1]], dtype=np.float32)
        t = float(np.dot(ap, ab) / (np.dot(ab, ab) + 1e-12))
        t = clamp(t, 0.0, 1.0)
        proj = np.array(a, dtype=np.float32) + t * ab
        return proj, t

    @staticmethod
    def _frenet_from_right(r1, r2):
        v = np.array([r2[0]-r1[0], r2[1]-r1[1]], dtype=np.float32)
        v = v / (np.linalg.norm(v) + 1e-12)
        if v[0] < 0.0: v, r1, r2 = -v, r2, r1
        n = np.array([-v[1], v[0]], dtype=np.float32)
        return v, n, np.array(r1, dtype=np.float32), np.array(r2, dtype=np.float32)

    @staticmethod
    def _to_sd(p, v, n, o):
        dpos = np.array([p[0]-o[0], p[1]-o[1]], dtype=np.float32)
        return float(np.dot(dpos, v)), float(np.dot(dpos, n))

    def _lane_point_at_s_total(self, s_total, p1, p2):
        ab = np.array([p2[0]-p1[0], p2[1]-p1[1]], dtype=np.float32)
        L = float(np.linalg.norm(ab) + 1e-12)
        t = clamp(s_total / L, 0.0, 1.0)
        pt = np.array(p1, dtype=np.float32) + t * ab
        return float(pt[0]), float(pt[1])

    def _lane_width_dL(self, s_total, r1, r2, l1, l2):
        rx, ry = self._lane_point_at_s_total(s_total, r1, r2)
        lx, ly = self._lane_point_at_s_total(s_total, l1, l2)
        return float(np.hypot(lx - rx, ly - ry))

    def _inside_lane_gate(self, cx, cy, right, left) -> bool:
        """0 ≤ d ≤ dL(s) 판정 (여유: lane_gate_margin)"""
        r1 = np.array(right[0], dtype=np.float32); r2 = np.array(right[1], dtype=np.float32)
        l1 = np.array(left[0],  dtype=np.float32); l2 = np.array(left[1],  dtype=np.float32)

        v, n, r1f, r2f = self._frenet_from_right(tuple(r1), tuple(r2))
        ego = np.array([0.0, 0.0], dtype=np.float32)
        o, _ = self._project_point_to_segment(ego, r1f, r2f)

        ab = np.array([r2[0]-r1[0], r2[1]-r1[1]], dtype=np.float32)
        L  = float(np.linalg.norm(ab) + 1e-12)
        ap = np.array([o[0]-r1[0],  o[1]-r1[1]], dtype=np.float32)
        s0 = clamp(float(np.dot(ap, ab) / (np.dot(ab, ab) + 1e-12)), 0.0, 1.0) * L

        s_rel, d = self._to_sd((cx,cy), v, n, o)
        s_total  = s0 + s_rel
        dL = self._lane_width_dL(s_total, r1, r2, l1, l2)
        return (0.0 - self.lane_gate_margin) <= d <= (dL + self.lane_gate_margin)

    # ---------- 콜백 ----------
    def cb(self, msg: ObjectInfo):
        # 게이트 꺼져 있으면 아무것도 하지 않음
        if not self.is_section_active():
            return
        if self._human_detected:
            self._clear_boxes()
            return

        right, left = LANE_STORE.get_lanes()
        have_lanes = (right is not None and left is not None)

        n = clamp(int(getattr(msg, 'objectCounts', 0) or 0), 0, self.max_markers)
        arr = []; boxes_xy = []
        for i in range(n):
            cx = self._g(msg.centerX, i); cy = self._g(msg.centerY, i); cz = self._g(msg.centerZ, i)
            lx = self._g(msg.lengthX, i); ly = self._g(msg.lengthY, i); lz = self._g(msg.lengthZ, i)
            mnx = self._g(msg.minX, i);   mny = self._g(msg.minY, i);   mnz = self._g(msg.minZ, i)
            mxx = self._g(msg.maxX, i);   mxy = self._g(msg.maxY, i);   mxz = self._g(msg.maxZ, i)

            have_minmax = all(finite(v) for v in [mnx,mny,mnz,mxx,mxy,mxz]) and (mxx>mnx) and (mxy>mny) and (mxz>mnz)
            if have_minmax:
                cx, cy, cz = 0.5*(mnx+mxx), 0.5*(mny+mxy), 0.5*(mnz+mxz)
                lx, ly, lz = max(mxx-mnx,1e-4), max(mxy-mny,1e-4), max(mxz-mnz,1e-4)
            else:
                if not all(finite(v) for v in [cx,cy,cz,lx,ly,lz]): continue
                lx, ly, lz = max(lx,1e-4), max(ly,1e-4), max(lz,1e-4)

            if self.inside_vehicle_roi(cx, cy, cz): continue

            # ▶ 차선 안에 있는 물체만 유지
            if self.only_inside_lanes:
                if not have_lanes:
                    continue  # 차선 아직 없음 → 스킵
                if not self._inside_lane_gate(cx, cy, right, left):
                    continue

            boxes_xy.append((cx-0.5*lx, cx+0.5*lx, cy-0.5*ly, cy+0.5*ly))
            center = (cx, cy, cz); size = (lx, ly, lz)
            arr.append(self._wire(i, center, size) if self.use_wireframe else self._solid(i, center, size))

        out = MarkerArray()
        out.markers = [del_marker('cluster_bbox', delete_all=True)] + arr
        self.pub.publish(out)
        OBST_STORE.set_boxes(boxes_xy)
        self._last_cb_time = rospy.Time.now()
        self._has_boxes = len(boxes_xy) > 0

    def cb_human(self, msg: Int32):
        val = int(getattr(msg, 'data', 0) or 0)
        self._human_detected = (val != 0)
        if self._human_detected:
            self._clear_boxes()

    def _clear_boxes(self):
        if not self._has_boxes and OBST_STORE.get_boxes() == []:
            return
        OBST_STORE.set_boxes([])
        self._has_boxes = False
        self._last_cb_time = None
        arr = MarkerArray()
        arr.markers = [del_marker('cluster_bbox', delete_all=True)]
        self.pub.publish(arr)

    def _cb_timer(self, _event):
        if not self.is_section_active():
            return
        if self._human_detected:
            return
        if not self._has_boxes:
            return
        if self._last_cb_time is None:
            return
        if (rospy.Time.now() - self._last_cb_time).to_sec() <= self.clear_timeout:
            return

        self._clear_boxes()

# ================= 3) Frenet 경로 계획 (타원 팽창 & 시각화 & KF) ==================
class SimplePathPlanner:
    """
    - 오른쪽 차선 기반 Frenet(s,d)
    - 금지영역: **타원(ellipse) 단일화**
    - s∈[-back_look_s, lookahead_s] 창에서 팽창 박스 수집, 경로는 s∈[0, lookahead_s]에서 생성
    - /inflated_obstacles : 타원 외곽(라운디드-렉트) 시각화
    - PathInfo(20) 지점별 2D CV 칼만 필터 옵션
    - nav_msgs/Path (/nav_path) 퍼블리시는 제거됨
    - ▶ 추가: 차선 내 장애물 유무를 실시간으로 /static_force_lane(Int32)
        * 장애물 없으면 0
        * 장애물 있으면 1
    """
    def __init__(self):
        self.is_section_active = is_section_active
        gp = rospy.get_param
        # s축
        self.lookahead_s     = float(gp("~lookahead_s",   3.0))
        self.sample_ds       = float(gp("~sample_ds",     0.25))
        self.back_look_s     = float(gp("~back_look_s",   2.0))

        # 차선/밴드
        self.margin_d        = float(gp("~margin_d",      0.25))
        self.relax_margin_d  = float(gp("~relax_margin_d",0.10))
        self.max_relax_tries = int(gp("~max_relax_tries", 2))
        self.post_hold_s     = float(gp("~post_hold_s",   1.5))
        self.bias_when_avoiding    = float(gp("~bias_when_avoiding", 0.40))
        self.bias_when_between_two = float(gp("~bias_when_between_two", 0.10))
        self.alpha_ema_beta        = float(gp("~alpha_ema_beta", 0.2))
        self.alpha_eps_inside_band = float(gp("~alpha_eps_inside_band", 0.02))

        # 팽창 파라미터
        self.veh_length   = float(gp("~veh_length",     2.020))
        self.veh_width    = float(gp("~veh_width",      1.160))
        self.clear_s      = float(gp("~x_clearance",    0.15))
        self.clear_d      = float(gp("~y_clearance",    0.15))
        self.obs_clear_s  = float(gp("~obs_clear_s",    0.00))
        self.obs_clear_d  = float(gp("~obs_clear_d",    0.00))

        # 금지영역 필터 임계
        self.min_overlap_s      = float(gp("~min_overlap_s",     0.30))
        self.min_penetration_d  = float(gp("~min_penetration_d", 0.10))
        self.outside_excl_margin= float(gp("~outside_excl_margin",0.10))
        self.centroid_gate_tol  = float(gp("~centroid_gate_tol",  0.02))
        self.min_corridor_width = float(gp("~min_corridor_width", 0.50))
        self.two_side_extra_margin = float(gp("~two_side_extra_margin", 0.5))
        self.narrow_push_to_center_w = float(gp("~narrow_push_to_center_w", 0.80))
        self.narrow_push_gain       = float(gp("~narrow_push_gain", 0.35))

        # 안정화
        self.mode_lock_frames = int(gp("~mode_lock_frames", 6))
        self.switch_allow_s   = float(gp("~switch_allow_s", 0.6))
        self.dd_step_limit    = float(gp("~dd_step_limit", 0.12))
        self.smooth_window    = int(gp("~smooth_window", 3))

        # KF
        self.use_kf_path   = bool(gp("~use_kf_path", True))
        self.kf_q_pos      = float(gp("~kf_q_pos",   0.05))
        self.kf_q_vel      = float(gp("~kf_q_vel",   0.50))
        self.kf_r_meas     = float(gp("~kf_r_meas",  0.04))
        self.kf_dt_min     = float(gp("~kf_dt_min",  0.02))
        self.kf_dt_max     = float(gp("~kf_dt_max",  0.20))
        self._kf_filters: Optional[List[Kalman2D]] = None
        self._last_pub_time: Optional[rospy.Time] = None

        # 시각화(항상 ellipse 외곽)
        self.viz_inflated  = bool(gp("~viz_inflated", True))
        self.viz_samples   = int(gp("~viz_samples", 24))
        self.viz_width     = float(gp("~viz_width", 0.06))
        self.viz_color_rgba= [float(v) for v in gp("~viz_color_rgba", [1.0, 0.2, 0.2, 0.8])]

        # 상태
        self.right_line = None; self.left_line  = None
        self.prev_mode  = "CENTER"; self.lock_count = 0
        self.alpha      = 0.5;      self.prev_d0    = None
        self._last_boxes_info = None
        self._latest_lidar_deg = None
        self._controller_mode = "LANE"  # LANE | AVOID
        self._section_active = False
        self._avoid_speed_kph = float(gp("~avoid_speed_kph", 4))
        self._ld_default = int(gp("~avoid_ld_value", 3))

        # 토픽
        self.sub_lanes      = rospy.Subscriber("/lanes_markers", MarkerArray, self.cb_lanes, queue_size=5)
        self.pub_path_mk    = rospy.Publisher("/path_markers", Marker, queue_size=10, latch=True)
        self.pub_local_path = rospy.Publisher("/local_path", PathInfo, queue_size=1)
        self.pub_inflated   = rospy.Publisher("/inflated_obstacles", MarkerArray, queue_size=10, latch=True)

        # ▶ static force lane 퍼블리셔 (장애물 0 / 없음 1)
        self.pub_static_force = rospy.Publisher("/static_force_lane", Int32, queue_size=10, latch=True)

        # ▶ Mission 제어 관련 퍼블리셔
        self.pub_mission_lane_ctrl = rospy.Publisher("/mission_lane_control", Int32, queue_size=10, latch=True)
        self.pub_mission_deg = rospy.Publisher("/missionDeg", Float32, queue_size=10)
        self.pub_mission_kph = rospy.Publisher("/missionKPH", Float32, queue_size=10)
        self.pub_ld = rospy.Publisher("/lidar_pp_ld", Int32, queue_size=10)

        # ▶ lidarDeg 전달용
        self.sub_lidar_deg = rospy.Subscriber("/lidarDeg", Float32, self.cb_lidar_deg, queue_size=1)

        register_section_listener(self._on_section_change)
        current_section = get_current_section()
        if current_section is not None:
            self._on_section_change(is_section_active(), current_section)

    # ---------- 섹션 상태 ----------
    def _on_section_change(self, active: bool, value: Optional[int]):
        self._section_active = active
        if not active:
            self._set_controller_mode("LANE")
            OBST_STORE.set_boxes([])
            LANE_STORE.set_lanes(None, None)
            self._last_boxes_info = None
            self._publish_empty_path()
            self.pub_static_force.publish(Int32(data=1))
        else:
            self._set_controller_mode("LANE")

    def _set_controller_mode(self, mode: str):
        mode = "LANE" if mode not in ("LANE", "AVOID") else mode
        if mode == self._controller_mode:
            if mode == "LANE":
                self.pub_mission_lane_ctrl.publish(Int32(data=1))
            return
        self._controller_mode = mode
        if mode == "LANE":
            self.pub_mission_lane_ctrl.publish(Int32(data=1))
        else:
            self.pub_mission_lane_ctrl.publish(Int32(data=0))

    # ---------- lidarDeg 콜백 ----------
    def cb_lidar_deg(self, msg: Float32):
        self._latest_lidar_deg = float(msg.data)

    # ---------- 좌표/프레임 ----------
    @staticmethod
    def _project_point_to_segment(p, a, b):
        ap = np.array([p[0]-a[0], p[1]-a[1]], dtype=np.float32)
        ab = np.array([b[0]-a[0], b[1]-a[1]], dtype=np.float32)
        t = float(np.dot(ap, ab) / (np.dot(ab, ab) + 1e-12))
        t = clamp(t, 0.0, 1.0)
        proj = np.array(a, dtype=np.float32) + t * ab
        return proj, t

    def _frenet_frame_from_right_lane(self, r1, r2):
        v = np.array([r2[0]-r1[0], r2[1]-r1[1]], dtype=np.float32)
        v = v / (np.linalg.norm(v) + 1e-12)
        if v[0] < 0.0: v, r1, r2 = -v, r2, r1
        n = np.array([-v[1], v[0]], dtype=np.float32)
        return v, n, r1, r2

    @staticmethod
    def _to_sd(p, v, n, o):
        dpos = np.array([p[0]-o[0], p[1]-o[1]], dtype=np.float32)
        return float(np.dot(dpos, v)), float(np.dot(dpos, n))

    @staticmethod
    def _to_xy(s, d, v, n, o):
        p = o + v*s + n*d
        return float(p[0]), float(p[1])

    @staticmethod
    def _interp_d_left(s, sL1, dL1, sL2, dL2):
        if abs(sL2 - sL1) < 1e-6: return dL1
        t = clamp((s - sL1) / (sL2 - sL1), 0.0, 1.0)
        return dL1 + t*(dL2 - dL1)

    # ---------- 박스 전처리 & 필터 (타원 단일) ----------
    def _inflate_and_filter_boxes(self, boxes_xy, v, n, o, left_seg):
        a = max(1e-6, 0.5*self.veh_length + self.clear_s + self.obs_clear_s)  # s방향
        b = max(1e-6, 0.5*self.veh_width  + self.clear_d + self.obs_clear_d)  # d방향

        l1, l2 = left_seg
        sL1, dL1 = self._to_sd(l1, v, n, o)
        sL2, dL2 = self._to_sd(l2, v, n, o)

        out = []
        win_min, win_max = -self.back_look_s, self.lookahead_s

        for (minx, maxx, miny, maxy) in boxes_xy:
            cx, cy = 0.5*(minx+maxx), 0.5*(miny+maxy)
            sc, dc = self._to_sd((cx,cy), v, n, o)
            if not (win_min <= sc <= win_max + a):
                continue
            dL_at_c = self._interp_d_left(clamp(sc, 0.0, self.lookahead_s), sL1, dL1, sL2, dL2)
            if not (0.0 + self.centroid_gate_tol <= dc <= dL_at_c - self.centroid_gate_tol):
                continue

            corners = [(minx,miny),(minx,maxy),(maxx,miny),(maxx,maxy)]
            ss = []; dd = []
            for c in corners:
                s, dval = self._to_sd(c, v, n, o)
                ss.append(s); dd.append(dval)
            s_min0, s_max0 = min(ss), max(ss)
            d_min0, d_max0 = min(dd), max(dd)

            s_min, s_max = s_min0 - a, s_max0 + a
            d_min, d_max = d_min0 - b, d_max0 + b

            sa, sb = max(win_min, s_min), min(win_max, s_max)
            if sb - sa < self.min_overlap_s:
                continue

            dL_sa = self._interp_d_left(clamp(sa, 0.0, self.lookahead_s), sL1, dL1, sL2, dL2)
            dL_sb = self._interp_d_left(clamp(sb, 0.0, self.lookahead_s), sL1, dL1, sL2, dL2)
            dL_max = max(dL_sa, dL_sb)
            if d_min >= dL_max + self.outside_excl_margin: continue
            if d_max <= 0.0   - self.outside_excl_margin: continue

            out.append((s_min, s_max, d_min, d_max, a, b))

        return out, (sL1, dL1, sL2, dL2)

    # ---------- 허용밴드 보조 ----------
    @staticmethod
    def _subtract_forbidden_from_allow(allow, forb):
        out = []
        for (a,b) in allow:
            if b <= forb[0] or forb[1] <= a:
                out.append((a,b))
            else:
                if a < forb[0]: out.append((a, max(a,forb[0])))
                if forb[1] < b: out.append((min(b,forb[1]), b))
        return [(x,y) for (x,y) in out if (y-x) > 1e-6]

    # ---------- 모드 결정 ----------
    def _decide_mode(self, boxes_sd, sL1,dL1,sL2,dL2):
        left_near = right_near = None
        for (s_min,s_max,d_min,d_max, a, b) in boxes_sd:
            s_front = max(0.0, s_min)
            mid_d = 0.5*(d_min + d_max)
            dL = self._interp_d_left(s_front, sL1, dL1, sL2, dL2)
            if abs(dL - mid_d) < abs(mid_d - 0.0):
                left_near  = s_front if left_near  is None else min(left_near,  s_front)
            else:
                right_near = s_front if right_near is None else min(right_near, s_front)

        new_mode = ("BETWEEN_TWO" if (left_near is not None and right_near is not None)
                    else "AVOID_RIGHT" if left_near is not None
                    else "AVOID_LEFT"  if right_near is not None
                    else "CENTER")

        if self.lock_count > 0:
            self.lock_count -= 1
            return self.prev_mode

        if self.prev_mode == "AVOID_RIGHT" and new_mode == "AVOID_LEFT":
            if right_near is None or right_near > self.switch_allow_s: return self.prev_mode
        if self.prev_mode == "AVOID_LEFT" and new_mode == "AVOID_RIGHT":
            if left_near is None or left_near > self.switch_allow_s: return self.prev_mode

        if new_mode != self.prev_mode: self.lock_count = self.mode_lock_frames
        return new_mode

    # ---------- 경량 스무딩 ----------
    def _smooth_path_sd(self, path_sd):
        if not path_sd or self.smooth_window <= 1: return path_sd
        w = max(1, int(self.smooth_window));  w += (w % 2 == 0)
        k = w // 2; s_list = [p[0] for p in path_sd]; d_list = [p[1] for p in path_sd]; out=[]
        for i in range(len(path_sd)):
            i0, i1 = max(0, i-k), min(len(path_sd)-1, i+k)
            out.append((s_list[i], float(sum(d_list[i0:i1+1])/(i1-i0+1))))
        return out

    # ---------- 타원 금지 밴드 ----------
    @staticmethod
    def _ellipse_delta_d_at_s(s, s_min, s_max, a, b):
        s_core_min = s_min + a
        s_core_max = s_max - a
        if s_core_min <= s <= s_core_max: return b
        ds = (s_core_min - s) if s < s_core_min else (s - s_core_max)
        if ds >= a: return 0.0
        return b * math.sqrt(max(0.0, 1.0 - (ds/a)*(ds/a)))

    def _forbidden_band_at_s(self, s, box) -> Optional[Tuple[float,float]]:
        s_min, s_max, d_min, d_max, a, b = box
        if not (s_min <= s <= s_max): return None
        d_min0 = d_min + b
        d_max0 = d_max - b
        dd = self._ellipse_delta_d_at_s(s, s_min, s_max, a, b)
        return (d_min0 - dd, d_max0 + dd)

    # ---------- 시각화(항상 ellipse 외곽) ----------
    def _publish_inflated_markers(self, boxes_sd, v, n, o):
        if not self.viz_inflated: return
        arr = MarkerArray(); arr.markers.append(del_marker("inflated_obs", delete_all=True))
        color = ColorRGBA(*self.viz_color_rgba); width = max(1e-4, self.viz_width)
        sid = 0
        for (s_min, s_max, d_min, d_max, a, b) in boxes_sd:
            samples = max(8, int(self.viz_samples))
            ss = np.linspace(s_min, s_max, samples)
            top_xy, bot_xy = [], []
            d_min0 = d_min + b; d_max0 = d_max - b
            for s in ss:
                dd = self._ellipse_delta_d_at_s(s, s_min, s_max, a, b)
                top_xy.append(self._to_xy(s, d_max0 + dd, v, n, o))
                bot_xy.append(self._to_xy(s, d_min0 - dd, v, n, o))
            outline = top_xy + list(reversed(bot_xy)) + [top_xy[0]]
            m = Marker(); m.header = hdr(); m.ns="inflated_obs"; m.id = sid; sid += 1
            m.type = Marker.LINE_STRIP; m.action = Marker.ADD
            m.scale.x = width; m.color = color
            m.points = [GPoint(x=float(x), y=float(y), z=0.0) for (x,y) in outline]
            arr.markers.append(m)
        self.pub_inflated.publish(arr)

    # ---------- 핵심: 경로 생성 ----------
    def _build_path_points(self, right_line, left_line, boxes_xy):
        r1_raw, r2_raw = right_line
        v, n, r1_fixed, r2_fixed = self._frenet_frame_from_right_lane(r1_raw, r2_raw)
        ego = np.array([0.0, 0.0], dtype=np.float32)
        proj_on_r, _ = self._project_point_to_segment(ego, np.array(r1_fixed, dtype=np.float32), np.array(r2_fixed, dtype=np.float32))
        o = proj_on_r
        l1 = np.array(left_line[0], dtype=np.float32)
        l2 = np.array(left_line[1], dtype=np.float32)

        boxes_sd, (sL1, dL1, sL2, dL2) = self._inflate_and_filter_boxes(boxes_xy, v, n, o, (l1,l2))

        mode = self._decide_mode(boxes_sd, sL1,dL1,sL2,dL2)
        if   mode == "CENTER":      alpha_t = 0.5
        elif mode == "AVOID_RIGHT": alpha_t = 0.0 + clamp(self.bias_when_avoiding, 0.0, 0.5)
        elif mode == "AVOID_LEFT":  alpha_t = 1.0 - clamp(self.bias_when_avoiding, 0.0, 0.5)
        else:                       alpha_t = 0.5
        beta = clamp(self.alpha_ema_beta, 0.01, 1.0)
        self.alpha = (1.0 - beta)*self.alpha + beta*alpha_t

        ss = np.arange(0.0, self.lookahead_s + 1e-9, self.sample_ds, dtype=float)
        path_sd, prev_band, prev_d = [], None, self.prev_d0
        used_left = used_right = False
        left_front_s = right_front_s = None

        for s in ss:
            dR, dL = 0.0, self._interp_d_left(s, sL1, dL1, sL2, dL2)
            margin = self.margin_d

            relevant = []
            for box in boxes_sd:
                if not (box[0] <= s <= box[1]): 
                    continue
                dm_dx = self._forbidden_band_at_s(s, box)
                if dm_dx is None: 
                    continue
                dm, dx = dm_dx
                relevant.append((dm, dx, box[1]))

            has_left  = any((abs(dL - 0.5*(dm+dx)) < abs(0.5*(dm+dx) - dR)) for (dm,dx,_) in relevant)
            has_right = any((abs(dL - 0.5*(dm+dx)) > abs(0.5*(dm+dx) - dR)) for (dm,dx,_) in relevant)
            if has_left and has_right: margin = self.margin_d + self.two_side_extra_margin

            allow = [] if (dL - dR <= 2*margin) else [(dR + margin, dL - margin)]
            tries = 0
            while allow:
                for (dm,dx,_) in relevant:
                    new_allow = []
                    for band in allow:
                        new_allow.extend(self._subtract_forbidden_from_allow([band], (dm,dx)))
                    allow = new_allow
                    if not allow: break
                allow = [(a,b) for (a,b) in allow if (b-a) >= self.min_corridor_width]
                if allow: break
                tries += 1
                if tries <= self.max_relax_tries:
                    margin = max(0.0, margin - self.relax_margin_d)
                    if dL - dR > 2*margin: allow = [(dR + margin, dL - margin)]
                else: break

            if not allow:
                if prev_d is not None:
                    d_safe = clamp(prev_d, dR + 0.05, dL - 0.05)
                    path_sd.append((float(s), float(d_safe)))
                    prev_d = d_safe
                continue

            if relevant:
                for (dm,dx,s_max) in relevant:
                    mid = 0.5*(dm+dx)
                    if abs(dL - mid) < abs(mid - dR):
                        used_left = True;  left_front_s  = s_max if left_front_s  is None else max(left_front_s,  s_max)
                    else:
                        used_right = True; right_front_s = s_max if right_front_s is None else max(right_front_s, s_max)

            chosen = None
            if prev_band is not None:
                pa, pb = prev_band; best_overlap, best_dist = -1.0, 1e9
                for (a,b) in allow:
                    overlap = max(0.0, min(pb, b) - max(pa, a))
                    center_dist = abs(0.5*(a+b) - 0.5*(pa+pb))
                    if (overlap > best_overlap) or (abs(overlap - best_overlap) < 1e-6 and center_dist < best_dist):
                        best_overlap, best_dist, chosen = overlap, center_dist, (a,b)
            if chosen is None:
                if prev_d is not None:
                    contain = [ (a,b) for (a,b) in allow if (a <= prev_d <= b) ]
                    if contain:
                        if   mode == "AVOID_RIGHT": chosen = min(contain, key=lambda x: 0.5*(x[0]+x[1]))
                        elif mode == "AVOID_LEFT":  chosen = max(contain, key=lambda x: 0.5*(x[0]+x[1]))
                        else:                       chosen = max(contain, key=lambda x: (x[1]-x[0]))
                    else:
                        chosen = min(allow, key=lambda x: abs(0.5*(x[0]+x[1]) - prev_d))
                else:
                    if   mode == "AVOID_RIGHT": chosen = min(allow, key=lambda x: (0.5*(x[0]+x[1]), -(x[1]-x[0])))
                    elif mode == "AVOID_LEFT":  chosen = max(allow, key=lambda x: (0.5*(x[0]+x[1]),  (x[1]-x[0])))
                    else:                       chosen = max(allow, key=lambda x: (x[1]-x[0]))

            prev_band = chosen
            a_band,b_band = chosen; width = b_band - a_band
            alpha_use = self.alpha
            if width <= self.narrow_push_to_center_w:
                alpha_use = (1.0 - self.narrow_push_gain)*alpha_use + self.narrow_push_gain*0.5

            d_des_base = a_band + clamp(alpha_use, 0.0 + self.alpha_eps_inside_band, 1.0 - self.alpha_eps_inside_band) * (b_band - a_band)

            if CENTER_RIGHT_OFFSET_D != 0.0 and not relevant:
                d_des = d_des_base - CENTER_RIGHT_OFFSET_D
            else:
                d_des = d_des_base

            d_des = clamp(d_des, a_band + 1e-3, b_band - 1e-3)

            if prev_d is not None:
                d_des = clamp(d_des, prev_d - self.dd_step_limit, prev_d + self.dd_step_limit)
            d_des = clamp(d_des, a_band + 1e-3, b_band - 1e-3)

            path_sd.append((float(s), float(d_des)))
            prev_d = d_des

        if path_sd:
            end_s = path_sd[-1][0]
            if used_left and not used_right and (left_front_s is not None):
                hold_s = min(left_front_s + self.post_hold_s, end_s)
                dR, dL = 0.0, self._interp_d_left(hold_s, sL1, dL1, sL2, dL2)
                a0, b0 = dR + self.margin_d, dL - self.margin_d
                d_hold = a0 + clamp(self.bias_when_avoiding, 0.0, 0.5) * (b0 - a0)
                d_hold = clamp(d_hold, a0 + 1e-3, b0 - 1e-3)
                path_sd.append((float(hold_s), float(d_hold)))
            if used_right and not used_left and (right_front_s is not None):
                hold_s = min(right_front_s + self.post_hold_s, end_s)
                dR, dL = 0.0, self._interp_d_left(hold_s, sL1, dL1, sL2, dL2)
                a0, b0 = dR + self.margin_d, dL - self.margin_d
                d_hold = b0 - clamp(self.bias_when_avoiding, 0.0, 0.5) * (b0 - a0)
                d_hold = clamp(d_hold, a0 + 1e-3, b0 - 1e-3)
                path_sd.append((float(hold_s), float(d_hold)))

        path_sd = self._smooth_path_sd(path_sd)
        if path_sd: self.prev_d0 = path_sd[0][1]
        self.prev_mode = mode

        # 시각화용 캐시
        self._last_boxes_info = (boxes_sd, v, n, o)

        return [ self._to_xy(s, d, v, n, o) for (s,d) in path_sd ]

    # ---------- KF 적용 & 퍼블리시 ----------
    def _apply_kf_if_needed(self, pts_xy: List[Tuple[float,float]]) -> List[Tuple[float,float]]:
        if not self.use_kf_path or not pts_xy or len(pts_xy)!=PATH_NUM_POINTS:
            return pts_xy
        if self._kf_filters is None:
            self._kf_filters = [Kalman2D(self.kf_q_pos, self.kf_q_vel, self.kf_r_meas) for _ in range(PATH_NUM_POINTS)]
            self._last_pub_time = rospy.Time.now()
            return pts_xy
        now = rospy.Time.now()
        dt = (now - (self._last_pub_time or now)).to_sec()
        dt = clamp(dt, self.kf_dt_min, self.kf_dt_max)
        self._last_pub_time = now
        return [ self._kf_filters[i].process(p, dt) for i,p in enumerate(pts_xy) ]

    def _publish_empty_path(self):
        msg_empty = PathInfo()
        msg_empty.cnt = 0
        msg_empty.x = [0.0]*PATH_NUM_POINTS
        msg_empty.y = [0.0]*PATH_NUM_POINTS
        self.pub_local_path.publish(msg_empty)

    def _publish_mission_commands(self, has_obstacle: bool):
        if has_obstacle:
            self._set_controller_mode("AVOID")
            if self._latest_lidar_deg is not None:
                self.pub_mission_deg.publish(Float32(data=self._latest_lidar_deg))
            self.pub_mission_kph.publish(Float32(data=self._avoid_speed_kph))
            self.pub_ld.publish(Int32(data=self._ld_default))
        else:
            self._set_controller_mode("LANE")
    def _publish_path(self, pts_xy):
        if not pts_xy:
            msg = PathInfo(); msg.cnt = 0
            msg.x = [0.0]*PATH_NUM_POINTS; msg.y = [0.0]*PATH_NUM_POINTS
            self.pub_local_path.publish(msg); return

        m = Marker(); m.header = hdr(); m.ns = "path"; m.id = 0
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.08; m.color.r,m.color.g,m.color.b,m.color.a = 1.0,1.0,0.0,1.0
        m.points = [GPoint(x=float(x), y=float(y), z=0.0) for (x,y) in pts_xy]
        self.pub_path_mk.publish(m)

        xs = np.asarray([p[0] for p in pts_xy], dtype=np.float64)
        ys = np.asarray([p[1] for p in pts_xy], dtype=np.float64)
        if len(xs) < 2:
            xs = np.array([xs[0], xs[0] + 1e-3]); ys = np.array([ys[0], ys[0]])
        seg = np.hypot(np.diff(xs), np.diff(ys))
        s = np.zeros(len(xs)); s[1:] = np.cumsum(seg)
        total = float(s[-1])
        if total < 1e-6:
            xs_fit = np.linspace(xs[0], xs[-1] + 1e-3, PATH_NUM_POINTS)
            ys_fit = np.full(PATH_NUM_POINTS, ys[0])
        else:
            st = np.linspace(0.0, total, PATH_NUM_POINTS)
            xs_fit = np.interp(st, s, xs); ys_fit = np.interp(st, s, ys)

        pts_resampled = list(zip(xs_fit.tolist(), ys_fit.tolist()))
        pts_smoothed  = self._apply_kf_if_needed(pts_resampled)

        msg = PathInfo(); msg.cnt = PATH_NUM_POINTS
        msg.x = [float(v[0]) for v in pts_smoothed]
        msg.y = [float(v[1]) for v in pts_smoothed]
        self.pub_local_path.publish(msg)

    # ---------- 콜백 ----------
    def cb_lanes(self, msg: MarkerArray):
        # 섹션 비활성 시 빈 경로만 유지
        if not self._section_active:
            rospy.loginfo("[WARN] Section Not Acivated") #DEBUG
            self._publish_empty_path()
            return
        right = left = None
        for mk in msg.markers:
            if mk.ns != "lanes" or mk.type != Marker.LINE_STRIP: continue
            if mk.id == 1 and len(mk.points) >= 2:
                right = ((mk.points[0].x, mk.points[0].y), (mk.points[-1].x, mk.points[-1].y))
            elif mk.id == 2 and len(mk.points) >= 2:
                left  = ((mk.points[0].x, mk.points[0].y), (mk.points[-1].x, mk.points[-1].y))
        if right is None or left is None:
            rospy.loginfo("[WARN] no wall (left or right)")
            self._publish_empty_path()
            return

        self.right_line, self.left_line = right, left

        # ▶ 차선 내 장애물 유무 확인(바운딩박스 노드가 차선 내만 저장)
        boxes = OBST_STORE.get_boxes()
        has_in_lane_obstacle = len(boxes) > 0

        # ▶ /static_force_lane 실시간 퍼블리시: 장애물 있으면 0, 없으면 1
        self.pub_static_force.publish(Int32(data=0 if has_in_lane_obstacle else 1))
        rospy.loginfo("[static_force_lane] %d", 0 if has_in_lane_obstacle else 1)

        # 경로 생성/퍼블리시
        pts_xy = self._build_path_points(self.right_line, self.left_line, boxes)

        # 팽창 외곽 시각화(ellipse)
        if self._last_boxes_info is not None:
            boxes_sd, v, n, o = self._last_boxes_info
            self._publish_inflated_markers(boxes_sd, v, n, o)

        self._publish_path(pts_xy)
        self._publish_mission_commands(has_in_lane_obstacle)

# ========================== main ==========================
def main():
    rospy.init_node('fusion_lidar_lanes_frenet_corridor', anonymous=False)

    # ▶ 섹션 정보 구독(9 섹션에서만 동작)
    rospy.Subscriber("/section", Int32, section_cb, queue_size=5)

    lanes   = VelodyneFilterAndLanes()
    bboxes  = ObjectInfoBBoxViz()
    planner = SimplePathPlanner()

    rospy.loginfo("Fusion node ready (section-gated to %d). Publishes /local_path(PathInfo,N=20) + /path_markers + /inflated_obstacles + /static_force_lane + mission handoff. Fixed frame='%s'",
                  _SECTION_ACTIVE_ID, FIXED_FRAME)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
