import rospy
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
import copy
import cv2
import os


# region 변수

# 1005 Kcity
# CUT_ROAD_RATIO = 0.4
# IMAGE_WIDTH = 1280
# IMAGE_HEIGHT = 720
# WARP_SIZE = 720
# CAR_OFFSET = 1.5  # BEV 시작점과 라이다와의 거리
# XM_PER_PIXEL = 3.9 / 420
# YM_PER_PIXEL = 10.5 / 720

# 1013 kcity
# CUT_ROAD_RATIO = 0.4
# IMAGE_WIDTH = 1280
# IMAGE_HEIGHT = 720
# WARP_SIZE = 720
# CAR_OFFSET = 3.1  # BEV 시작점과 라이다와의 거리
# XM_PER_PIXEL = 3.9 / 420
# YM_PER_PIXEL = 9.25 / 720

# 1019 kcity
# CUT_ROAD_RATIO = 0.4
# IMAGE_WIDTH = 1280
# IMAGE_HEIGHT = 720
# WARP_SIZE = 720
# CAR_OFFSET = 0.5  # BEV 시작점과 라이다와의 거리
# XM_PER_PIXEL = 3.5 / 420
# YM_PER_PIXEL = 8.8 / 720

# 최종
CUT_ROAD_RATIO = 0.4
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
WARP_SIZE = 720

CAR_OFFSET = 2.5  # BEV 시작점과 라이다와의 거리  # 251019 10도
XM_PER_PIXEL = 4.06 / 420 #차선 폭
YM_PER_PIXEL = 12.9 / 720 #BEV 차선 보이는 길이

# CAR_OFFSET = 1.8  # BEV 시작점과 라이다와의 거리 # 251019 15도
# XM_PER_PIXEL = 4.06 / 420 #차선 폭
# YM_PER_PIXEL = 4.2 / 720 #BEV 차선 보이는 길이

# endregion 변수


# region 함수

def get_camera_bev_parameters():
    # src = np.float32([[481, 149], [108, 429], [731, 151], [1160, 428]])  # 1005 kcity
    #src = np.float32([[470, 120], [41, 424], [768, 120], [1226, 423]])  # 1013 kcity
    #src = np.float32([[473, 15], [30, 427], [767, 15], [1261, 426]])  # 1019 kcity
    #src = np.float32([[448, 15], [38, 426], [793, 15], [1270, 427]])  # 1024 fmtc
    #src = np.float32([[495, 30], [49, 431], [812, 30], [1257, 429]])  # 최종
    #dst = np.float32([[150, 0], [150, 720], [570, 0], [570, 720]])
    
    # src = np.float32([[468, 3], [2, 329], [800, 3], [1278, 328]]) # 251019 10도
    # dst = np.float32([[150, 0], [150, 720], [570, 0], [570, 720]])
    
    # src = np.float32([[334, 1], [1, 233], [949, 1], [1278, 235]]) # 251019 15도
    # dst = np.float32([[200, 0], [200, 720], [520, 0], [520, 720]])
    
    src = np.float32([[535, 1], [1, 290], [748, 1], [1279, 288]])
    dst = np.float32([[200, 0], [200, 720], [520, 0], [520, 720]])
    


    M = cv2.getPerspectiveTransform(src, dst)

    return src, dst, M


# 기능 : 시각화 설정 함수
# R, G, B 값은 0~255 사이의 값으로 전달
def visualization_marker_array(point_list, color=(0,0,0), lenx=0.5, leny=0.5, lenz=0.5, marker_type_input='cube', duration=0.5):
    r = color[0] / 255.0
    g = color[1] / 255.0
    b = color[2] / 255.0

    marker_type = None
    if marker_type_input == 'cube':
        marker_type = Marker.CUBE
    elif marker_type_input == 'sphere':
        marker_type = Marker.SPHERE
    elif marker_type_input == 'arrow':
        marker_type = Marker.ARROW
    elif marker_type_input == 'cylinder':
        marker_type = Marker.CYLINDER
    elif marker_type_input == 'line_strip':
        marker_type = Marker.LINE_STRIP

    array = MarkerArray()
    for i, point in enumerate(point_list):
        marker = Marker()
        marker.header.frame_id = "velodyne"
        marker.header.stamp = rospy.Time()
        marker.id = i
        marker.type = marker_type
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.0
        marker.scale.x = lenx
        marker.scale.y = leny
        marker.scale.z = lenz
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.lifetime = rospy.Duration(duration)
        array.markers.append(marker)

    return array

# 기능 : 시각화 설정 함수
def visualization_marker(point, color=(0,0,0), lenx=0.5, leny=0.5, lenz=0.5, marker_type_input='cube', duration=0.5):
    r = color[0] / 255.0
    g = color[1] / 255.0
    b = color[2] / 255.0

    marker_type = None
    if marker_type_input == 'cube':
        marker_type = Marker.CUBE
    elif marker_type_input == 'sphere':
        marker_type = Marker.SPHERE
    elif marker_type_input == 'arrow':
        marker_type = Marker.ARROW
    elif marker_type_input == 'cylinder':
        marker_type = Marker.CYLINDER
    elif marker_type_input == 'line_strip':
        marker_type = Marker.LINE_STRIP

    marker = Marker()
    marker.header.frame_id = "velodyne"
    marker.header.stamp = rospy.Time()
    marker.id = 0
    marker.type = marker_type
    marker.pose.position.x = point[0]
    marker.pose.position.y = point[1]
    marker.pose.position.z = 0.0
    marker.scale.x = lenx
    marker.scale.y = leny
    marker.scale.z = lenz
    marker.color.a = 1.0
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.lifetime = rospy.Duration(duration)

    return marker

# 기능 : 두 점의 좌표를 입력받아 두 점 사이의 거리를 계산함
def calculate_distance(pt1, pt2):
    distance = ((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2)**0.5
    return distance

# 기능 : 어떤 차선과 동일한 형태의 가상 차선을 생성함
def make_virtual_lane(make_dir, opposite_lane_points, road_width=3.5):
    opposite_x = np.array([pt[0] for pt in opposite_lane_points])
    opposite_y = np.array([pt[1] for pt in opposite_lane_points])
    lane_fit = np.polyfit(opposite_x, opposite_y, 3)
    virtual_lane_x_list = list()
    virtual_lane_y_list = list()

    if make_dir == 'left':
        for i in range(0, len(opposite_x), 10):
            x = opposite_x[i]
            y = opposite_y[i]
            slope = 2*lane_fit[0]*x + lane_fit[1]
            slope = -(1/slope)
            unit = (1 + slope**2) ** 0.5
            unit_x = (1 / unit) * road_width
            unit_y = (slope / unit) * road_width
            if slope < 0:
                unit_x *= -1
                unit_y *= -1
            virtual_x = x + unit_x
            virtual_y = y + unit_y
            virtual_lane_x_list.append(virtual_x)
            virtual_lane_y_list.append(virtual_y)

    else:
        for i in range(0, len(opposite_x), 10):
            x = opposite_x[i]
            y = opposite_y[i]
            slope = 2*lane_fit[0]*x + lane_fit[1]
            slope = -(1/slope)
            unit = (1 + slope**2) ** 0.5
            unit_x = (1 / unit) * road_width
            unit_y = (slope / unit) * road_width
            if slope > 0:
                unit_x *= -1
                unit_y *= -1
            virtual_x = x + unit_x
            virtual_y = y + unit_y
            virtual_lane_x_list.append(virtual_x)
            virtual_lane_y_list.append(virtual_y)

    virtual_lane_fit = np.polyfit(virtual_lane_x_list, virtual_lane_y_list, 3)
    virtual_lane_y = np.polyval(virtual_lane_fit, opposite_x)
    virtual_lane_coords = [pt for pt in zip(opposite_x, virtual_lane_y)]

    return virtual_lane_coords

# 기능 : 현재 차선을 기준으로 현재 차선 내에 있는 장애물들을 계산함
def get_current_obstacles(left_lidar_points, right_lidar_points, obstacles):
    if obstacles is None:
        return

    current_obstacles = list()

    left_lane_fit = fit_polynomial(left_lidar_points, 3)
    right_lane_fit = fit_polynomial(right_lidar_points, 3)
 
    #print('obstacles :', obstacles)
    for pt in obstacles:
        ob_x, ob_y = pt

        left_lane_ob_y = np.polyval(left_lane_fit, ob_x)
        right_lane_ob_y = np.polyval(right_lane_fit, ob_x)

        if right_lane_ob_y <= ob_y <= left_lane_ob_y:
            current_obstacles.append((ob_x, ob_y))

    return current_obstacles


# 기능 : 좌표들의 리스트를 입력받아서 다항식에 적합시킨 다항식 계수들을 계산함
def fit_polynomial(points, degree=3):
    x_list = [pt[0] for pt in points]
    y_list = [pt[1] for pt in points]
    points_fit = np.polyfit(x_list, y_list, degree)

    return points_fit

def get_left_lane_index(data):
    max_below_360 = float('-inf')
    max_below_360_index = -1

    for index, value in enumerate(data):
        if 0 <= value <= 360 and value > max_below_360:
            max_below_360 = value
            max_below_360_index = index

    if max_below_360_index == -1:
        max_below_360_index = None

    return max_below_360_index

def get_right_lane_index(data):
    min_above_360 = float('inf')
    min_above_360_index = -1

    for index, value in enumerate(data):
        if 360 <= value <= 720 and value < min_above_360:
            min_above_360 = value
            min_above_360_index = index

    if min_above_360_index == -1:
        min_above_360_index = None

    return min_above_360_index

# 기능 : 왼쪽 하단이 원점인 BEV 이미지를 라이다 좌표계로 변환
def convert_point_bevcam2lidar(x_points, y_points):
    x_points = np.array(x_points)
    y_points = np.array(y_points)

    x_points = -(x_points - WARP_SIZE/2)
    x_points *= XM_PER_PIXEL
    y_points *= YM_PER_PIXEL
    x_points, y_points = y_points, x_points
    x_points += CAR_OFFSET

    lidar_coords = [pt for pt in zip(x_points, y_points)]

    return lidar_coords

# 기능 : Front View의 이미지 차선 좌표를 받아 BEV 상에서의 현재 왼쪽 차선과 오른쪽 차선들의 좌표들을 반환함
# 매개변수 : lanes_xys - , M - BEV 변환 행렬
# 호출 : lane_control
def get_current_lane_lidar_points(lanes_xys, M, virtual_lane_width=3.5, degree=3):
    ploty = np.linspace(0, WARP_SIZE-1, WARP_SIZE)

    lane_coefficients = list()
    x_intercept_list = list()
    for lane_points in lanes_xys:
        points = np.array(lane_points, dtype=np.float32)
        points = points.reshape(-1, 1, 2)
        transformed_points = cv2.perspectiveTransform(points, M).reshape(-1, 2)
        transformed_points[:, 1] = -(transformed_points[:, 1] - WARP_SIZE)

        x_list = transformed_points[:, 0]
        y_list = transformed_points[:, 1]
        if len(transformed_points) <= 15:
            coefficients = np.polyfit(y_list, x_list, 1)
        else:
            coefficients = np.polyfit(y_list, x_list, degree)
        
        lane_coefficients.append(coefficients)
        x_intercept_list.append(coefficients[-1])

    left_lane_idx = get_left_lane_index(x_intercept_list)
    right_lane_idx = get_right_lane_index(x_intercept_list)

    if left_lane_idx is not None and right_lane_idx is not None:  # 양쪽 차선이 모두 인식된 경우
        left_fit = lane_coefficients[left_lane_idx]
        right_fit = lane_coefficients[right_lane_idx]

        left_plotx = np.polyval(left_fit, ploty)
        right_plotx = np.polyval(right_fit, ploty)

        left_lidar_points = convert_point_bevcam2lidar(left_plotx, ploty)
        right_lidar_points = convert_point_bevcam2lidar(right_plotx, ploty)

    elif left_lane_idx is None and right_lane_idx is not None:  # 오른쪽 차선만 인식된 경우
        right_fit = lane_coefficients[right_lane_idx]
        right_plotx = np.polyval(right_fit, ploty)
        right_lidar_points = convert_point_bevcam2lidar(right_plotx, ploty)

        left_lidar_points = make_virtual_lane('left', right_lidar_points, virtual_lane_width)

    elif left_lane_idx is not None and right_lane_idx is None:  # 왼쪽 차선만 인식된 경우
        left_fit = lane_coefficients[left_lane_idx]
        left_plotx = np.polyval(left_fit, ploty)
        left_lidar_points = convert_point_bevcam2lidar(left_plotx, ploty)

        right_lidar_points = make_virtual_lane('right', left_lidar_points, virtual_lane_width)

    else:
        return None


    return left_lidar_points, right_lidar_points

# 기능 : 3차 함수의 기울기 평균을 계산함
def calculate_slope_avg_on_cubic_function(points, points_fit=None):
    if points_fit is None:
        points_fit = fit_polynomial(points)

    differential_coefficient = [3*points_fit[0], 2*points_fit[1], points_fit[2]]

    x_list = [pt[0] for i, pt in enumerate(points) if i%10==0]
    slope_list = np.polyval(differential_coefficient, x_list)

    slope_avg = sum(slope_list) / len(slope_list)

    return slope_avg

# 기능 : 기울기와 하나의 좌표를 입력받아 해당 기울기를 가지면서 주어진 좌표를 지나는 직선의 기울기(m)와 y절편(b)를 반환함
# 어떤 직선의 기울기와 하나의 좌표를 알고 있는 경우
def calculate_line_parameters(slope, point):
    x, y = point
    intercept = y - slope * x
    return [slope, intercept]

# 기능 : 3차 함수와 1차 함수가 만나는 점들을 구한 후, point와 가장 가까운 하나의 좌표를 계산하여 반환함
def calculate_intersection_point_cubic_linear(cubic_fit, linear_fit, point):
    coefficients = [cubic_fit[0], cubic_fit[1], cubic_fit[2]-linear_fit[0], cubic_fit[3]-linear_fit[1]]
    x_intersections = np.roots(coefficients)
    x_intersections = [x.real for x in x_intersections if np.isreal(x)]
    y_intersections = np.polyval(linear_fit, x_intersections)
    intersection_points = list(zip(x_intersections, y_intersections))

    distance_list = [((pt[0]-point[0])**2 + (pt[1]-point[1])**2)**0.5 for pt in intersection_points]
    distance_min_idx = np.argmin(distance_list)
    result_point = intersection_points[distance_min_idx]

    return result_point

def calculate_perpendicular_slope(slope):
    return -(1/slope)

def compute_coordinate_offset(slope, m_value):
    unit = (1 + slope**2) ** 0.5
    offset_x = (1 / unit) * m_value
    offset_y = (slope / unit) * m_value

    return (offset_x, offset_y)

def calculate_m2idx(m_value, plotx):

    x_list = copy.deepcopy(plotx)
    x_list = np.array(x_list)
    x_list -= CAR_OFFSET
    x_list -= m_value
    x_list = abs(x_list)
    closest_idx = np.argmin(x_list)

    return closest_idx


# 3차 곡선과 이 곡선 위의 점과 m_value 만큼 떨어진 곡선 위의 두 좌표를 구하는 함수
def calculate_points_at_distance(poly_fit, pt, m_value):
    x = pt[0]
    y = pt[1]
    result_points = list()

    # 비슷한거 2개 있는건 + - 차이임
    for i in range(1, 99999):
        current_x = x + i*0.05
        current_y = np.polyval(poly_fit, current_x)
        distance = ((current_x-x)**2 + (current_y-y)**2)**0.5
        if distance > m_value:
            result_points.append((current_x, current_y))
            break

    for i in range(1, 99999):
        current_x = x - i*0.05
        current_y = np.polyval(poly_fit, current_x)
        distance = ((current_x-x)**2 + (current_y-y)**2)**0.5
        if distance > m_value:
            result_points.append((current_x, current_y))
            break

    return result_points

def generate_lane_center_path(left_lidar_points, right_lidar_points, mid_x_meter=4, end_x_meter=7):
    plotx = np.array([pt[0] for pt in left_lidar_points])

    lefty = np.array([pt[1] for pt in left_lidar_points])
    righty = np.array([pt[1] for pt in right_lidar_points])

    mid_x_idx = calculate_m2idx(mid_x_meter, plotx)
    end_x_idx = calculate_m2idx(end_x_meter, plotx)

    waypoint_temp = []
    mid_center_x = plotx[mid_x_idx]
    mid_center_y = (lefty[mid_x_idx] + righty[mid_x_idx]) / 2
    start_x = CAR_OFFSET
    start_y = 0
    temp_fit = np.polyfit([start_x, mid_center_x], [start_y, mid_center_y], 1)
    for i in range(0, end_x_idx+1):
        if i < mid_x_idx:
            #mid_y = temp_fit[0]*plotx[i] + temp_fit[1]
            mid_y = np.polyval(temp_fit, plotx[i])
        else:
            mid_y = (lefty[i] + righty[i]) / 2
        mid_x = plotx[i]
        waypoint_temp.append((mid_x, mid_y))

    waypoint_x = list()
    waypoint_y = list()
    for pt in waypoint_temp:
        waypoint_x.append(pt[0])
        waypoint_y.append(pt[1])
    waypoint_fit = np.polyfit(waypoint_x, waypoint_y, 3)

    waypoint_plotx = np.linspace(waypoint_x[0], waypoint_x[-1], 20)
    waypoint_ploty = np.polyval(waypoint_fit, waypoint_plotx)

    waypoint = list()
    for i in range(0, len(waypoint_plotx)):
        waypoint.append((waypoint_plotx[i], waypoint_ploty[i]))

    return waypoint
