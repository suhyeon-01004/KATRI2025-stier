#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped
import itertools
import numpy as np
from object_detector.msg import ObjectInfo
from mission.msg import PathInfo  # PathInfo 사용

MAX_AREA_THRESHOLD = 0.1
MAX_LENGTH_THRESHOLD = 0.25
Z_CENTER_THRESHOLD = -0.1

CAR_LENGTH = 2.02
CAR_WIDTH  = 1.16
CAR_HEIGHT = 0.52
MARGIN_X = 0.2
MARGIN_Y = 0.2
MARGIN_Z = 0.2

LIDAR_OFFSET_X = -0.5
LIDAR_OFFSET_Y = 0.0
LIDAR_OFFSET_Z = 0.0

SHOW_ROI_MARKER = True

CONNECT_DISTANCE_THRESHOLD = 3.5
MIN_CONNECTIONS_THRESHOLD = 2

WEIGHT_RIGHT = 0.6
START_PATH_OFFSET = 0.5

SPLINE_DEGREE = 1

# ===== Path smoothing params (전역) =====
PATH_BLEND_PREV = 0.7
PATH_BLEND_CURR = 0.3
PATH_NUM_POINTS = 20           # ✅ PathInfo의 고정 배열 길이에 맞춤 (20)
RESET_PREV_ON_EMPTY = True

class ObjectMarkerVisualizer:
    def __init__(self):
        rospy.init_node('object_marker_visualizer', anonymous=True)
        rospy.Subscriber('/object_info', ObjectInfo, self.callback)
        self.marker_pub = rospy.Publisher('/object_markers', MarkerArray, queue_size=10)
        self.path_pub = rospy.Publisher('/local_path', PathInfo, queue_size=1)  # ✅ 토픽/타입
        if SHOW_ROI_MARKER:
            self.roi_marker_pub = rospy.Publisher('/roi_marker', Marker, queue_size=1)

        self.prev_path = None  # {'x': np.ndarray, 'y': np.ndarray, 'z': np.ndarray}

    def publish_empty_path(self):
        # ✅ 고정 길이 20으로 채워서 보냄
        path_msg = PathInfo()
        path_msg.cnt = 0
        path_msg.x = [0.0]*PATH_NUM_POINTS
        path_msg.y = [0.0]*PATH_NUM_POINTS
        self.path_pub.publish(path_msg)
        if RESET_PREV_ON_EMPTY:
            self.prev_path = None

    def create_roi_marker(self, x_min, x_max, y_min, y_max, z_min, z_max):
        marker = Marker()
        marker.header.frame_id = "velodyne"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "roi"
        marker.id = 999
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = (x_min + x_max) / 2.0
        marker.pose.position.y = (y_min + y_max) / 2.0
        marker.pose.position.z = (z_min + z_max) / 2.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = abs(x_max - x_min)
        marker.scale.y = abs(y_max - y_min)
        marker.scale.z = abs(z_max - z_min)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.2
        marker.lifetime = rospy.Duration(0)
        return marker

    def generate_zigzag_midpoints(self, left_points, right_points, weight_right=0.7):
        left_points = sorted(left_points, key=lambda p: p.y)
        right_points = sorted(right_points, key=lambda p: p.y)

        midpoints = []
        i, j = 0, 0
        prev = None
        is_left_turn = True

        while i < len(left_points) or j < len(right_points):
            if is_left_turn and i < len(left_points):
                curr = left_points[i]; i += 1
            elif (not is_left_turn) and j < len(right_points):
                curr = right_points[j]; j += 1
            else:
                break

            if prev is not None:
                if is_left_turn:
                    left, right = prev, curr
                else:
                    left, right = curr, prev

                w = weight_right
                mid = Point(
                    x=(1 - w) * left.x + w * right.x,
                    y=(1 - w) * left.y + w * right.y,
                    z=(1 - w) * left.z + w * right.z,
                )
                midpoints.append(mid)

            prev = curr
            is_left_turn = not is_left_turn

        return midpoints

    def connect_close_markers(self, centers):
        N = len(centers)
        if N < 2:
            return [], {}, {}

        parent = list(range(N))
        def find(u):
            while parent[u] != u:
                parent[u] = parent[parent[u]]
                u = parent[u]
            return u
        def union(u, v):
            u_root = find(u)
            v_root = find(v)
            if u_root == v_root:
                return False
            parent[v_root] = u_root
            return True

        edges = []
        for i in range(N):
            for j in range(i + 1, N):
                dx = centers[i].x - centers[j].x
                dy = centers[i].y - centers[j].y
                dz = centers[i].z - centers[j].z
                dist = (dx**2 + dy**2 + dz**2)**0.5
                if dist <= CONNECT_DISTANCE_THRESHOLD:
                    edges.append((dist, i, j))
        edges.sort()

        connections = []
        for dist, i, j in edges:
            if union(i, j):
                connections.append((i, j))
            if len(connections) == N - 1:
                break

        if len(connections) < MIN_CONNECTIONS_THRESHOLD:
            print("⚠️ 연결 간선 수 부족 → 시각화 생략")
            return [], {}, {}

        groups = {}
        for idx in range(N):
            root = find(idx)
            groups.setdefault(root, []).append(idx)

        group_centroids = {
            gid: Point(
                x=sum(centers[i].x for i in idxs) / len(idxs),
                y=sum(centers[i].y for i in idxs) / len(idxs),
                z=sum(centers[i].z for i in idxs) / len(idxs),
            ) for gid, idxs in groups.items()
        }

        while len(groups) > 2:
            merge_candidates = []
            for g1, g2 in itertools.combinations(groups.keys(), 2):
                c1, c2 = group_centroids[g1], group_centroids[g2]
                dist = ((c1.x - c2.x)**2 + (c1.y - c2.y)**2 + (c1.z - c2.z)**2)**0.5
                merge_candidates.append((dist, g1, g2))
            merge_candidates.sort()
            _, g1, g2 = merge_candidates[0]
            new_id = min(g1, g2)
            new_indices = groups[g1] + groups[g2]
            new_center = Point(
                x=sum(centers[i].x for i in new_indices) / len(new_indices),
                y=sum(centers[i].y for i in new_indices) / len(new_indices),
                z=sum(centers[i].z for i in new_indices) / len(new_indices),
            )
            del groups[g1], groups[g2]
            groups[new_id] = new_indices
            group_centroids[new_id] = new_center

        group_colors = {}
        sorted_y = sorted(group_centroids.items(), key=lambda x: x[1].y)
        if len(sorted_y) == 2:
            group_colors[sorted_y[0][0]] = (0.0, 1.0, 0.0)
            group_colors[sorted_y[1][0]] = (0.0, 0.0, 1.0)
        elif len(sorted_y) == 1:
            group_colors[sorted_y[0][0]] = (1.0, 1.0, 0.0)

        marker_list = []
        for group_id, indices in groups.items():
            line_marker = Marker()
            line_marker.header.frame_id = "velodyne"
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = "connections"
            line_marker.id = 10000 + group_id
            line_marker.type = Marker.LINE_LIST
            line_marker.action = Marker.ADD
            line_marker.pose.orientation.w = 1.0
            line_marker.scale.x = 0.03
            r, g, b = group_colors.get(group_id, (0.5, 0.5, 0.5))
            line_marker.color.r = r
            line_marker.color.g = g
            line_marker.color.b = b
            line_marker.color.a = 1.0
            line_marker.lifetime = rospy.Duration(0.1)
            for i, j in connections:
                if find(i) == group_id and find(j) == group_id:
                    line_marker.points.append(centers[i])
                    line_marker.points.append(centers[j])
            marker_list.append(line_marker)

        print(f"🌳 트리 개수: {len(groups)} | 🔗 간선 수: {len(connections)}")
        return marker_list, groups, group_colors

    def draw_spline_curve(self, midpoints):
        if len(midpoints) < 2:
            rospy.logwarn("회귀에 필요한 포인트가 부족합니다.")
            self.publish_empty_path()
            return None

        # 원시 포인트 → 다항 회귀 (선형 기본) → 균일 샘플 PATH_NUM_POINTS
        x = np.array([p.x for p in midpoints], dtype=np.float64)
        y = np.array([p.y for p in midpoints], dtype=np.float64)
        z = np.array([p.z for p in midpoints], dtype=np.float64)

        degree = min(SPLINE_DEGREE, len(midpoints) - 1)
        coeffs = np.polyfit(x, y, deg=degree)

        x_min, x_max = float(np.min(x)), float(np.max(x))
        if x_max - x_min < 1e-6:
            x_fit = np.linspace(x_min, x_min + 1e-3, PATH_NUM_POINTS)
        else:
            x_fit = np.linspace(x_min, x_max, PATH_NUM_POINTS)

        y_fit = np.polyval(coeffs, x_fit)
        z_fit = np.full_like(x_fit, float(np.mean(z)))

        # 이전 path와 혼합 스무딩
        if self.prev_path is not None and \
           len(self.prev_path['x']) == PATH_NUM_POINTS and \
           len(self.prev_path['y']) == PATH_NUM_POINTS and \
           len(self.prev_path['z']) == PATH_NUM_POINTS:
            w_prev = float(PATH_BLEND_PREV)
            w_curr = float(PATH_BLEND_CURR)
            denom = w_prev + w_curr
            if denom <= 1e-9:
                w_prev, w_curr = 0.5, 0.5; denom = 1.0
            x_fit = (w_prev * self.prev_path['x'] + w_curr * x_fit) / denom
            y_fit = (w_prev * self.prev_path['y'] + w_curr * y_fit) / denom
            z_fit = (w_prev * self.prev_path['z'] + w_curr * z_fit) / denom

        # RViz 선 시각화
        trend_marker = Marker()
        trend_marker.header.frame_id = "velodyne"
        trend_marker.header.stamp = rospy.Time.now()
        trend_marker.ns = "spline_path"
        trend_marker.id = 60000
        trend_marker.type = Marker.LINE_STRIP
        trend_marker.action = Marker.ADD
        trend_marker.pose.orientation.w = 1.0
        trend_marker.scale.x = 0.04
        trend_marker.color.r = 1.0
        trend_marker.color.g = 1.0
        trend_marker.color.b = 0.0
        trend_marker.color.a = 1.0
        trend_marker.lifetime = rospy.Duration(0.2)
        for xi, yi, zi in zip(x_fit, y_fit, z_fit):
            trend_marker.points.append(Point(x=float(xi), y=float(yi), z=float(zi)))

        # ✅ PathInfo 형식으로 퍼블리시 (항상 길이 20)
        valid_cnt = PATH_NUM_POINTS  # 여기서는 20개 모두 유효
        x_list = [float(v) for v in x_fit[:PATH_NUM_POINTS]]
        y_list = [float(v) for v in y_fit[:PATH_NUM_POINTS]]
        # 혹시라도 길이가 모자라면 0.0 패딩
        if len(x_list) < PATH_NUM_POINTS:
            x_list += [0.0] * (PATH_NUM_POINTS - len(x_list))
        if len(y_list) < PATH_NUM_POINTS:
            y_list += [0.0] * (PATH_NUM_POINTS - len(y_list))

        path_msg = PathInfo()
        path_msg.cnt = int(valid_cnt)
        path_msg.x = x_list
        path_msg.y = y_list
        self.path_pub.publish(path_msg)

        # 다음 프레임을 위해 저장
        self.prev_path = {
            'x': np.array(x_fit, dtype=np.float64),
            'y': np.array(y_fit, dtype=np.float64),
            'z': np.array(z_fit, dtype=np.float64),
        }

        return trend_marker

    def callback(self, msg):
        marker_array = MarkerArray()
        marker_id = 0
        centers = []
        markers = []

        car_x_min = -(CAR_LENGTH / 2 + MARGIN_X)
        car_x_max =  (CAR_LENGTH / 2 + MARGIN_X)
        car_y_min = -(CAR_WIDTH  / 2 + MARGIN_Y)
        car_y_max =  (CAR_WIDTH  / 2 + MARGIN_Y)
        car_z_min = -(CAR_HEIGHT / 2 + MARGIN_Z)
        car_z_max =  (CAR_HEIGHT / 2 + MARGIN_Z)

        if SHOW_ROI_MARKER:
            roi_marker = self.create_roi_marker(
                car_x_min + LIDAR_OFFSET_X,
                car_x_max + LIDAR_OFFSET_X,
                car_y_min + LIDAR_OFFSET_Y,
                car_y_max + LIDAR_OFFSET_Y,
                car_z_min + LIDAR_OFFSET_Z,
                car_z_max + LIDAR_OFFSET_Z,
            )
            self.roi_marker_pub.publish(roi_marker)

        for i in range(msg.objectCounts):
            lx, ly, lz = msg.lengthX[i], msg.lengthY[i], msg.lengthZ[i]
            area = lx * ly
            if area > MAX_AREA_THRESHOLD or lx > MAX_LENGTH_THRESHOLD or ly > MAX_LENGTH_THRESHOLD:
                continue
            cx = msg.centerX[i] - LIDAR_OFFSET_X
            cy = msg.centerY[i] - LIDAR_OFFSET_Y
            cz = msg.centerZ[i] - LIDAR_OFFSET_Z
            if car_x_min <= cx <= car_x_max and car_y_min <= cy <= car_y_max and car_z_min <= cz <= car_z_max:
                continue
            if msg.centerZ[i] > Z_CENTER_THRESHOLD:
                continue
            marker = Marker()
            marker.header.frame_id = "velodyne"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "object"
            marker.id = marker_id; marker_id += 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = msg.centerX[i]
            marker.pose.position.y = msg.centerY[i]
            marker.pose.position.z = msg.centerZ[i]
            marker.pose.orientation.w = 1.0
            marker.scale.x = lx
            marker.scale.y = ly
            marker.scale.z = lz
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.lifetime = rospy.Duration(0.1)
            marker_array.markers.append(marker)
            centers.append(Point(x=msg.centerX[i], y=msg.centerY[i], z=msg.centerZ[i]))
            markers.append(marker)

        published_path = False

        if len(centers) >= 2:
            line_markers, groups, group_colors = self.connect_close_markers(centers)
            for m in line_markers:
                marker_array.markers.append(m)
            for group_id, indices in groups.items():
                r, g, b = group_colors.get(group_id, (0.5, 0.5, 0.5))
                for idx in indices:
                    markers[idx].color.r = r
                    markers[idx].color.g = g
                    markers[idx].color.b = b
                    markers[idx].color.a = 0.8
            if len(groups) == 2:
                sorted_groups = sorted(groups.items(), key=lambda item: sum(centers[i].y for i in item[1]) / len(item[1]))
                left_points = [centers[i] for i in sorted_groups[0][1]]
                right_points = [centers[i] for i in sorted_groups[1][1]]
                midpoints = self.generate_zigzag_midpoints(left_points, right_points, WEIGHT_RIGHT)
                origin_pt = Point(x=START_PATH_OFFSET, y=0.0, z=0.0)
                midpoints.insert(0, origin_pt)

                if len(midpoints) >= 1:
                    path_marker = Marker()
                    path_marker.header.frame_id = "velodyne"
                    path_marker.header.stamp = rospy.Time.now()
                    path_marker.ns = "zigzag_midpoints"
                    path_marker.id = 50000
                    path_marker.type = Marker.POINTS
                    path_marker.action = Marker.ADD
                    path_marker.pose.orientation.w = 1.0
                    path_marker.scale.x = 0.08
                    path_marker.scale.y = 0.08
                    path_marker.color.r = 1.0
                    path_marker.color.g = 1.0
                    path_marker.color.b = 1.0
                    path_marker.color.a = 1.0
                    path_marker.lifetime = rospy.Duration(0.2)
                    path_marker.points.extend(midpoints)
                    marker_array.markers.append(path_marker)

                spline_marker = self.draw_spline_curve(midpoints)
                if spline_marker:
                    marker_array.markers.append(spline_marker)
                    published_path = True

        if not published_path:
            self.publish_empty_path()

        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        ObjectMarkerVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
