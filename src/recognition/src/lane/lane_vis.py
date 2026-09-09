#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np
import argparse
import json
import time
import copy
import cv2
import sys

COLORS = [
    (255, 0, 0),
    (0, 255, 0),
    (0, 0, 255),
    (255, 255, 0),
    (0, 255, 255)
]

import warnings
warnings.simplefilter('ignore', np.RankWarning)


def get_left_lane_index(data):
    max_below_360 = float('-inf')
    max_below_360_index = -1

    for index, value in enumerate(data):
        if value < 360 and value > max_below_360:
            max_below_360 = value
            max_below_360_index = index

    return max_below_360_index

def get_right_lane_index(data):
    min_above_360 = float('inf')
    min_above_360_index = -1

    for index, value in enumerate(data):
        if value > 360 and value < min_above_360:
            min_above_360 = value
            min_above_360_index = index

    return min_above_360_index

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

def draw_lane_on_original_image(frame, lanes_xys):
    image = copy.deepcopy(frame)

    for idx, xys in enumerate(lanes_xys):
        for i in range(1, len(xys)):
            cv2.line(image, xys[i - 1], xys[i], COLORS[idx], thickness=3)

    return image

def draw_lane_on_warp_image():
    pass



def cam_callback(image_msg):
    global frame
    frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
    cv2.imwrite('frame.png', frame)


def lane_data_callback(lane_data):
    global lanes_xys
    lanes_xys = json.loads(lane_data.data)



if __name__ == '__main__':
    rospy.init_node('lane_visualizer', anonymous=True)

    frame = None

    path_vis_pub = rospy.Publisher("/path_vis_pub", MarkerArray, queue_size=1)
    bridge = CvBridge()
    image_width, image_height = 1280, 720
    warp_size = 720

    src = np.float32([[495, 30], [49, 431], [812, 30], [1257, 429]])
    src = np.float32([[535, 1], [1, 290], [748, 1], [1279, 288]])
    dst = np.float32([[150, 0], [150, 720], [570, 0], [570, 720]])  # 원근 변환한 이미지의 destination 좌표
    M = cv2.getPerspectiveTransform(src, dst)

    wide_src = np.float32([[495, 30], [49, 431], [812, 30], [1257, 429]])
    wide_src = np.float32([[535, 1], [1, 290], [748, 1], [1279, 288]])
    wide_dst = np.float32([[310, 0], [310, 720], [410, 0], [410, 720]]) # 값 바꾸기
    M_wide = cv2.getPerspectiveTransform(wide_src, wide_dst)

    rospy.Subscriber('lane_data_publisher', String, lane_data_callback)
    #rospy.Subscriber('/usb_cam1/image_raw', Image, cam_callback)
    print('started!!')

    while True:
        try:
            image_msg = rospy.wait_for_message("/usb_cam2/image_raw", Image, 5)
            frame = bridge.imgmsg_to_cv2(image_msg, "bgr8")

            current_lane_points = lanes_xys
            vis_original = draw_lane_on_original_image(frame, current_lane_points)

            lane_image = frame[int(0.4*image_height):, :]
            lane_image_points = [[(x, y - int(0.4 * image_height)) for x, y in sublist] for sublist in current_lane_points]

            warp_img = cv2.warpPerspective(lane_image, M, (warp_size, warp_size), flags=cv2.INTER_LINEAR)
            warp_img_wide = cv2.warpPerspective(lane_image, M_wide, (warp_size, warp_size), flags=cv2.INTER_LINEAR)

            ploty = np.linspace(0, warp_size-1, warp_size)

            lane_coefficients = list()
            x_intercept_list = list()
            bev_image_points_list = list()  # 왼쪽 하단이 원점인 이미지 BEV 좌표
            for lane_points in lane_image_points:
                points = np.array(lane_points, dtype=np.float32)
                points = points.reshape(-1, 1, 2)
                transformed_points = cv2.perspectiveTransform(points, M).reshape(-1, 2)
                transformed_points[:, 1] = -(transformed_points[:, 1] - warp_size)
                bev_image_points_list.append(transformed_points)

                x_list = transformed_points[:, 0]
                y_list = transformed_points[:, 1]
                coefficients = np.polyfit(y_list, x_list, 3)
                
                lane_coefficients.append(coefficients)
                x_intercept_list.append(coefficients[-1])

            lane_coefficients_wide = list()
            bev_wide_image_points_list = list()
            for lane_points in lane_image_points:
                points = np.array(lane_points, dtype=np.float32)
                points = points.reshape(-1, 1, 2)
                transformed_points = cv2.perspectiveTransform(points, M_wide).reshape(-1, 2)
                transformed_points[:, 1] = -(transformed_points[:, 1] - warp_size)
                bev_wide_image_points_list.append(transformed_points)

                x_list = transformed_points[:, 0]
                y_list = transformed_points[:, 1]
                coefficients = np.polyfit(y_list, x_list, 3)
                
                lane_coefficients_wide.append(coefficients)

            left_lane_idx = get_left_lane_index(x_intercept_list)
            right_lane_idx = get_right_lane_index(x_intercept_list)
            left_road_left_lane_idx = get_left_road_left_lane_index(x_intercept_list, x_intercept_list[left_lane_idx])
            right_road_right_lane_idx = get_right_road_right_lane_index(x_intercept_list, x_intercept_list[right_lane_idx])
            
            left_fit = lane_coefficients[left_lane_idx]
            right_fit = lane_coefficients[right_lane_idx]
            left_bev_points = bev_image_points_list[left_lane_idx]
            right_bev_points = bev_image_points_list[right_lane_idx]

            left_plotx = left_fit[0]*(ploty**3) + left_fit[1]*(ploty**2) + left_fit[2]*ploty + left_fit[3]
            right_plotx = right_fit[0]*(ploty**3) + right_fit[1]*(ploty**2) + right_fit[2]*ploty + right_fit[3]

            # 이미지에 그리기
            # Left
            for i in range(1, len(left_bev_points)):
                pt1 = (int(left_bev_points[i-1][0]), int(-(left_bev_points[i-1][1]-720)))
                pt2 = (int(left_bev_points[i][0]), int(-(left_bev_points[i][1]-720)))
                cv2.line(warp_img, pt1, pt2, (255,0,0), 3)

            # Right
            for i in range(1, len(right_bev_points)):
                pt1 = (int(right_bev_points[i-1][0]), int(-(right_bev_points[i-1][1]-720)))
                pt2 = (int(right_bev_points[i][0]), int(-(right_bev_points[i][1]-720)))
                cv2.line(warp_img, pt1, pt2, (0,0,255), 3)

            # Wide Warp Image Lane Visualization
            for i, bev_points in enumerate(bev_wide_image_points_list):

                if i==left_lane_idx or i==right_lane_idx:
                    color = (255,0,0)
                elif i==left_road_left_lane_idx or i==right_road_right_lane_idx:
                    color = (0,255,0)
                else:
                    color=(0,0,255)

                #current_plotx = coefficient[0]*(ploty**3) + coefficient[1]*(ploty**2) + coefficient[2]*ploty + coefficient[3]
                for i in range(1, len(bev_points)):
                    pt1 = (int(bev_points[i-1][0]), int(-(bev_points[i-1][1]-720)))
                    pt2 = (int(bev_points[i][0]), int(-(bev_points[i][1]-720)))
                    cv2.line(warp_img_wide, pt1, pt2, color, 3)
                    
            vis_original = cv2.resize(vis_original, (640, 360))
            warp_img = cv2.resize(warp_img, (360, 360))
            warp_img_wide = cv2.resize(warp_img_wide, (360, 360))

            cv2.imshow('vis_original', vis_original)
            cv2.imshow('warp_img', warp_img)
            cv2.imshow('warp_img_wide', warp_img_wide)
            if cv2.waitKey(25) == ord('q'):
                break

        except:
            continue
        
