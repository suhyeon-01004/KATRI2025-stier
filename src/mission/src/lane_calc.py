import numpy as np
import copy
import cv2
import time

import sys
sys.path.append('/home/stier/catkin_ws/src/mission/src')
from mission_utils import *

def get_left_road_left_lane_index(lst, target):
    smaller_values = [x for x in lst if x < target]
    
    if not smaller_values:
        return None
    
    largest_smaller_value = max(smaller_values)
    
    return lst.index(largest_smaller_value)

def get_right_road_right_lane_index(lst, target):
    larger_values = [x for x in lst if x > target]
    
    if not larger_values:
        return None
    
    smallest_larger_value = min(larger_values)
    
    return lst.index(smallest_larger_value)


def get_right_lane_x_intercept(lanes_xys, M):
    
    lane_coefficients = list()
    x_intercept_list = list()
    for lane_points in lanes_xys:
        points = np.array(lane_points, dtype=np.float32)
        points = points.reshape(-1, 1, 2)
        transformed_points = cv2.perspectiveTransform(points, M).reshape(-1, 2)
        transformed_points[:, 1] = -(transformed_points[:, 1] - WARP_SIZE)

        x_list = transformed_points[:, 0]
        y_list = transformed_points[:, 1]
        coefficients = np.polyfit(y_list, x_list, 3)
        
        lane_coefficients.append(coefficients)
        x_intercept_list.append(coefficients[-1])

    right_lane_idx = get_right_lane_index(x_intercept_list)

    return x_intercept_list[right_lane_idx]

def get_left_lane_x_intercept(lanes_xys, M):
    
    lane_coefficients = list()
    x_intercept_list = list()
    for lane_points in lanes_xys:
        points = np.array(lane_points, dtype=np.float32)
        points = points.reshape(-1, 1, 2)
        transformed_points = cv2.perspectiveTransform(points, M).reshape(-1, 2)
        transformed_points[:, 1] = -(transformed_points[:, 1] - WARP_SIZE)

        x_list = transformed_points[:, 0]
        y_list = transformed_points[:, 1]
        coefficients = np.polyfit(y_list, x_list, 3)
        
        lane_coefficients.append(coefficients)
        x_intercept_list.append(coefficients[-1])

    left_lane_idx = get_left_lane_index(x_intercept_list)

    return x_intercept_list[left_lane_idx]
    




def calculate_lane(lanes_xys, M):
    left_lidar_points, right_lidar_points = get_current_lane_lidar_points(lanes_xys, M)
    angle, waypoint_lidar = calculate_angle(left_lidar_points, right_lidar_points)

    return angle, waypoint_lidar, left_lidar_points, right_lidar_points  
    

def calculate_angle(left_lidar_points, right_lidar_points):
    plotx = np.array([pt[0] for pt in left_lidar_points])

    lefty = np.array([pt[1] for pt in left_lidar_points])
    righty = np.array([pt[1] for pt in right_lidar_points])
    mid_idx = calculate_m2idx(4, plotx)

    """ 2024-07-19 차선 중간 경로 포인트 생성 """
    waypoint_temp = []
    mid_center_x = plotx[mid_idx]
    mid_center_y = (lefty[mid_idx] + righty[mid_idx]) / 2
    start_x = CAR_OFFSET
    start_y = 0
    temp_fit = np.polyfit([start_x, mid_center_x], [start_y, mid_center_y], 1)
    for i in range(len(lefty)):
        if i < mid_idx:
            mid_y = temp_fit[0]*plotx[i] + temp_fit[1]
        else:
            mid_y = (lefty[i] + righty[i]) / 2
        mid_x = plotx[i]
        waypoint_temp.append((mid_x, mid_y))
    """ 차선 중간 경로 포인트 생성 끝 """

    waypoint_x = list()
    waypoint_y = list()
    for pt in waypoint_temp:
        waypoint_x.append(pt[0])
        waypoint_y.append(pt[1])
    waypoint_fit = np.polyfit(waypoint_x, waypoint_y, 3)
    waypoint_ploty = waypoint_fit[0]*(plotx**3) + waypoint_fit[1]*(plotx**2) + waypoint_fit[2]*plotx + waypoint_fit[3]

    waypoint = list()
    for i in range(len(plotx)):
        waypoint.append((plotx[i], waypoint_ploty[i]))

    """ 2024-07-19 차선 중간 경로 포인트 pure[100%] Linking CXX executable /h_pursuit """
    L = 1.04  # 축간거리 (계산에 사용되는 변수)

    ''' 화면상에서 차량의 위치 (아마 카메라가 달린 위치로 계속 고정되지 않을까 싶습니다...) '''
    car_location_x, car_location_y = 0, 0
    
    ''' Ld에 해당하는 mid_points[] 내의 점 인덱스 '''
    target_index = 432

    ''' Ld에 해당하는 mid_points[] 내의 점과 차량과의 실제 거리 (화면이 실제와 몇대몇 비율인지 알아야 할거 같음) '''
    distance = 8

    dx = waypoint[target_index][0] - car_location_x
    dy = waypoint[target_index][1] - car_location_y
    temp_alpha = np.arctan2(dy, dx)

    radian = np.arctan(2 * L * np.sin(temp_alpha) / (distance))
    angle = radian*(180/np.pi)
    """ 차선 중간 경로 포인트 pure_pursuit 끝 """

    return angle, waypoint




# region Lane Change Codes

def get_right_lane_x_intercept(lanes_xys, M):
    lane_coefficients = list()
    x_intercept_list = list()
    for lane_points in lanes_xys:
        points = np.array(lane_points, dtype=np.float32)
        points = points.reshape(-1, 1, 2)
        transformed_points = cv2.perspectiveTransform(points, M).reshape(-1, 2)
        transformed_points[:, 1] = -(transformed_points[:, 1] - WARP_SIZE)

        x_list = transformed_points[:, 0]
        y_list = transformed_points[:, 1]
        coefficients = np.polyfit(y_list, x_list, 3)
        
        lane_coefficients.append(coefficients)
        x_intercept_list.append(coefficients[-1])

    right_lane_idx = get_right_lane_index(x_intercept_list)

    return x_intercept_list[right_lane_idx]

def get_left_lane_x_intercept(lanes_xys, M):
    lane_coefficients = list()
    x_intercept_list = list()
    for lane_points in lanes_xys:
        points = np.array(lane_points, dtype=np.float32)
        points = points.reshape(-1, 1, 2)
        transformed_points = cv2.perspectiveTransform(points, M).reshape(-1, 2)
        transformed_points[:, 1] = -(transformed_points[:, 1] - WARP_SIZE)

        x_list = transformed_points[:, 0]
        y_list = transformed_points[:, 1]
        coefficients = np.polyfit(y_list, x_list, 3)
        
        lane_coefficients.append(coefficients)
        x_intercept_list.append(coefficients[-1])

    left_lane_idx = get_left_lane_index(x_intercept_list)

    return x_intercept_list[left_lane_idx]

def calculate_right_lane_change(lanes_xys, M, before_right_lane_x_intercept):
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
        coefficients = np.polyfit(y_list, x_list, 3)
        
        lane_coefficients.append(coefficients)
        x_intercept_list.append(coefficients[-1])

    left_lane_idx = get_left_lane_index(x_intercept_list)
    right_lane_idx = get_right_lane_index(x_intercept_list)
    right_road_right_lane_idx = get_right_road_right_lane_index(x_intercept_list, x_intercept_list[right_lane_idx])
    
    left_fit = lane_coefficients[left_lane_idx]
    right_fit = lane_coefficients[right_lane_idx]

    left_plotx = left_fit[0]*(ploty**3) + left_fit[1]*(ploty**2) + left_fit[2]*ploty + left_fit
    right_plotx = right_fit[0]*(ploty**3) + right_fit[1]*(ploty**2) + right_fit[2]*ploty + right_fit

    left_lidar_points = convert_point_bevcam2lidar(left_plotx, ploty)
    right_lidar_points = convert_point_bevcam2lidar(right_plotx, ploty)

    if right_road_right_lane_idx is None:  # 가장 오른쪽 차선으로 차선변경할 때 순간적으로 idx에 None이 들어가는 경우가 있음
        left_plotx = left_fit[0]*(ploty**3) + left_fit[1]*(ploty**2) + left_fit[2]*ploty + left_fit
        right_plotx = right_fit[0]*(ploty**3) + right_fit[1]*(ploty**2) + right_fit[2]*ploty + right_fit

        mid_plotx = (left_plotx + right_plotx) / 2
        goal_point = (int(mid_plotx[150]), int(ploty[150]))

    else:
        right_road_right_fit = lane_coefficients[right_road_right_lane_idx]
        right_road_right_plotx = right_road_right_fit[0]*(ploty**3) + right_road_right_fit[1]*(ploty**2) + right_road_right_fit[2]*ploty + right_road_right_fit[3]

        right_mid_plotx = (right_plotx+right_road_right_plotx)/2
        goal_point = (int(right_mid_plotx[150]), int(ploty[150]))

    start_point = (360, 0)
    lane_change_fit = np.polyfit([start_point[1], goal_point[1]], [start_point[0], goal_point[0]], 1)
    lane_change_plotx = lane_change_fit[0]*ploty[:150] + lane_change_fit[1]

    
    # Calculate Angle
    """ 2024-07-19 차선 중간 경로 포인트 생성 """
    lane_waypoint = []
    for i in range(len(lane_change_plotx)):
        mid_x = lane_change_plotx[i]
        mid_y = ploty[i]
        lane_waypoint.append((mid_x, mid_y))
    
    """ 차선 중간 경로 포인트 생성 끝 """
    waypoint_x = [pt[0] for pt in lane_waypoint]
    waypoint_y = [pt[1] for pt in lane_waypoint]
    waypoint_lidar = convert_point_bevcam2lidar(waypoint_x, waypoint_y)

    """ 2024-07-19 차선 중간 경로 포인트 pure_pursuit """
    L = 1.04  # 축간거리 (계산에 사용되는 변수)

    ''' 화면상에서 차량의 위치 (아마 카메라가 달린 위치로 계속 고정되지 않을까 싶습니다...) '''
    car_location_x, car_location_y = 0, 0
    
    ''' Ld에 해당하는 mid_points[] 내의 점 인덱스 '''
    target_index = 90

    ''' Ld에 해당하는 mid_points[] 내의 점과 차량과의 실제 거리 (화면이 실제와 몇대몇 비율인지 알아야 할거 같음) '''
    distance = 2

    dx = waypoint_lidar[target_index][0] - car_location_x
    dy = waypoint_lidar[target_index][1] - car_location_y
    temp_alpha = np.arctan2(dy, dx)

    radian = np.arctan(2 * L * np.sin(temp_alpha) / (distance))
    angle = radian*(180/np.pi)

    # 차선 변경 완료 여부 확인
    is_lane_change_finish = None
    if np.argmin([(before_right_lane_x_intercept - x)**2 for x in x_intercept_list]) == right_lane_idx:
        is_lane_change_finish = False
    else:
        is_lane_change_finish = True

    return angle, is_lane_change_finish, x_intercept_list[right_lane_idx], waypoint_lidar, left_lidar_points, right_lidar_points


def calculate_left_lane_change(lanes_xys, M, before_left_lane_x_intercept):
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
        coefficients = np.polyfit(y_list, x_list, 3)
        
        lane_coefficients.append(coefficients)
        x_intercept_list.append(coefficients[-1])

    left_lane_idx = get_left_lane_index(x_intercept_list)
    right_lane_idx = get_right_lane_index(x_intercept_list)
    left_road_left_lane_idx = get_left_road_left_lane_index(x_intercept_list, x_intercept_list[left_lane_idx])
    #right_road_right_lane_idx = get_right_road_right_lane_index(x_intercept_list, x_intercept_list[right_lane_idx])
    
    left_fit = lane_coefficients[left_lane_idx]
    right_fit = lane_coefficients[right_lane_idx]

    left_plotx = left_fit[0]*(ploty**3) + left_fit[1]*(ploty**2) + left_fit[2]*ploty + left_fit[3]
    right_plotx = right_fit[0]*(ploty**3) + right_fit[1]*(ploty**2) + right_fit[2]*ploty + right_fit[3]

    left_lidar_points = convert_point_bevcam2lidar(left_plotx, ploty)
    right_lidar_points = convert_point_bevcam2lidar(right_plotx, ploty)

    if left_road_left_lane_idx is None:  # 가장 오른쪽 차선으로 차선변경할 때 순간적으로 idx에 None이 들어가는 경우가 있음
        left_plotx = left_fit[0]*(ploty**3) + left_fit[1]*(ploty**2) + left_fit[2]*ploty + left_fit[3]
        right_plotx = right_fit[0]*(ploty**3) + right_fit[1]*(ploty**2) + right_fit[2]*ploty + right_fit[3]

        mid_plotx = (left_plotx + right_plotx) / 2
        goal_point = (int(mid_plotx[150]), int(ploty[150]))

    else:
        left_road_left_fit = lane_coefficients[left_road_left_lane_idx]
        left_road_left_plotx = left_road_left_fit[0]*(ploty**3) + left_road_left_fit[1]*(ploty**2) + left_road_left_fit[2]*ploty + left_road_left_fit[3]

        left_mid_plotx = (left_plotx+left_road_left_plotx)/2
        goal_point = (int(left_mid_plotx[150]), int(ploty[150]))



    

    start_point = (360, 0)
    lane_change_fit = np.polyfit([start_point[1], goal_point[1]], [start_point[0], goal_point[0]], 1)
    lane_change_plotx = lane_change_fit[0]*ploty[:150] + lane_change_fit[1]


    
    # Calculate Angle

    """ 2024-07-19 차선 중간 경로 포인트 생성 """
    lane_waypoint = []
    for i in range(len(lane_change_plotx)):
        mid_x = lane_change_plotx[i]
        mid_y = ploty[i]
        lane_waypoint.append((mid_x, mid_y))
    
    """ 차선 중간 경로 포인트 생성 끝 """
    waypoint_x = [pt[0] for pt in lane_waypoint]
    waypoint_y = [pt[1] for pt in lane_waypoint]
    waypoint_lidar = convert_point_bevcam2lidar(waypoint_x, waypoint_y)

    """ 2024-07-19 차선 중간 경로 포인트 pure_pursuit """
    L = 1.04  # 축간거리 (계산에 사용되는 변수)
    Ld = 0

    ''' 화면상에서 차량의 위치 (아마 카메라가 달린 위치로 계속 고정되지 않을까 싶습니다...) '''
    car_location_x, car_location_y = 0, 0
    
    ''' Ld에 해당하는 mid_points[] 내의 점 인덱스 '''
    target_index = 90

    ''' Ld에 해당하는 mid_points[] 내의 점과 차량과의 실제 거리 (화면이 실제와 몇대몇 비율인지 알아야 할거 같음) '''
    distance = 2

    dx = waypoint_lidar[target_index][1] - car_location_x
    dy = waypoint_lidar[target_index][0] - car_location_y
    temp_alpha = np.arctan2(dx, dy)

    radian = np.arctan(2 * L * np.sin(temp_alpha) / (distance))
    angle = radian*(180/np.pi)



    # 차선 변경 완료 여부 확인
    is_lane_change_finish = None
    if np.argmin([(before_left_lane_x_intercept - x)**2 for x in x_intercept_list]) == left_lane_idx:
        is_lane_change_finish = False
    else:
        is_lane_change_finish = True

    return angle, is_lane_change_finish, x_intercept_list[left_lane_idx], waypoint_lidar, left_lidar_points, right_lidar_points



# endregion Lane Change Codes
