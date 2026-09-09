#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32, String, Float32

from ultralytics import YOLO
from pathlib import Path
import cv2
import numpy as np
import sys
import time
import json
import threading

import warnings
warnings.simplefilter('ignore', np.RankWarning)

import sys
sys.path.append('{}/catkin_ws/src/mission/src'.format(Path.home()))
import mission_utils as util
from mission_utils import CUT_ROAD_RATIO, IMAGE_HEIGHT, WARP_SIZE

def callback_speed(data):
    global stop_distance, last_time, current_speed

    current_speed = data.data

    if current_speed <= 0.1:  # 완전 정지해있어도 GPS 속도는 완전히 0이 아님
        current_speed = 0

    if stop_distance is not None and stopline_activate:
        stop_distance -= (current_speed/3.6) * (time.time()-last_time)
    
    last_time = time.time()

def callback_current_idx(data):
    global stopline_activate, stopline_finish, stop_distance, left_lane_lidar_points, right_lane_lidar_points
    
    current_idx = data.data
    #print('current_idx :', current_idx)
    
    '''
    265 ~ 300 : 소형 정적 후 비신호 우회전 정지선
    335 ~ 370 : 첫 번째 좌회전 
    423 ~ 462 : 첫 번째 직진
    655 ~ 697 : 대형 정적 후 정지선
    840 ~ 885 : 배달 후 대형 교차로 정지선
    990 ~ 1020 : 대형 교차로 후 삼거리 좌회전 정지선
    1287 ~ 1327 : 비신호 우회전 정지선
    1496 ~ 1538 : 직진 정지선
    1580 ~ 1619: 마지막 정지선
    '''
    if (265 <= current_idx < 300) or (335 <= current_idx < 370) or (423 <= current_idx < 462) or \
        (655 <= current_idx < 697) or (840 <= current_idx < 885) or (990 <= current_idx < 1020)  \
        or (1287 <= current_idx < 1327) or (1496 <= current_idx < 1538) or (1580 <= current_idx < 1619):

    # if (50 <= current_idx < 93) or (141 <= current_idx < 180) or (635 <= current_idx < 665):
        if stop_distance is None:
            if not stopline_finish:
                stopline_activate = True
        else:
            if stop_distance > 0 and not stopline_finish:
                stopline_activate = True
            else:
                stopline_activate = False
                stopline_finish = True

    else:
        stopline_activate = False
        stopline_finish = False
        stop_distance = None
        left_lane_lidar_points = None
        right_lane_lidar_points = None




def publish_stop():
    global current_speed

    while not rospy.is_shutdown():
        stop_sign_value = 0
        if stop_distance is None:
            stop_sign_value = 0
        else:

            stopline_distance_pub.publish(stop_distance)
            if 8 < stop_distance <= 15:
                stop_sign_value = 2

            elif 0 < stop_distance <= 8:
                if current_speed <= 6:
                    if stop_distance <= 3:
                        stop_sign_value = 1
                    else:
                        stop_sign_value = 2
                else:
                    stop_sign_value = 3

            else:
                stop_sign_value = 0
                
        print('stop_distance :', stop_distance)
        #print('current_speed :', current_speed)
        #print('stop_sign_value :', stop_sign_value)
        #print()

        stop_sign_pub.publish(stop_sign_value)
        time.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('stop_line', anonymous=True)

    # ROS Publisher
    stop_sign_pub = rospy.Publisher('/stopsign', Int32, queue_size=10)
    stopline_distance_pub = rospy.Publisher('/stopline_distance', Float32, queue_size=1)

    # ROS Subscriber
    rospy.Subscriber('/gps_speed', Float32, callback_speed)
    rospy.Subscriber('/current_idx', Int32, callback_current_idx)

    stop_distance = None
    stopline_activate = False
    stopline_finish = False
    left_lane_lidar_points = None
    right_lane_lidar_points = None
    current_speed = 0

    stop_publish_thread = threading.Thread(target=publish_stop).start()

    stop_line_model = YOLO('{}/catkin_ws/ModelFiles/StopLine.pt'.format(Path.home()))
    stop_line_model.predict(np.zeros((720, 1280, 3)), verbose=False)

    src, dst, M = util.get_camera_bev_parameters()
    last_time = time.time()

    bridge = CvBridge()
    while not rospy.is_shutdown():
        #print('stopline_activate :', stopline_activate)
        if stopline_activate:
            lane_data = rospy.wait_for_message('lane_data_publisher', String)
            lanes_xys = json.loads(lane_data.data)
            lanes_xys = [[(x, y - int(CUT_ROAD_RATIO * IMAGE_HEIGHT)) for x, y in lane] for lane in lanes_xys]

            image_msg = rospy.wait_for_message('/usb_cam1/image_raw', Image)
            frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            lane_img = frame[int(CUT_ROAD_RATIO * IMAGE_HEIGHT):, :]

            ret_val = util.get_current_lane_lidar_points(lanes_xys, M, degree=1)
            if ret_val is None:
                if left_lane_lidar_points is None and right_lane_lidar_points is None:
                    continue
            else:
                left_lane_lidar_points, right_lane_lidar_points = ret_val
            left_lane_lidar_fit = util.fit_polynomial(left_lane_lidar_points, 1)
            right_lane_lidar_fit = util.fit_polynomial(right_lane_lidar_points, 1)

            # warp_img
            warp_img = cv2.warpPerspective(lane_img, M, (WARP_SIZE, WARP_SIZE), flags=cv2.INTER_LINEAR)

            # YOLO Detection
            result = stop_line_model.predict(frame, verbose=False)[0]   
            
            if result is not None and len(result) != 0:            # result[0].masks
                if result[0].masks:
                    boxes = result.boxes
                    clss = result.boxes.cls.cpu().tolist()
                    masks = result.masks.xy

                    x_avg_list = []  # BEV 내의 정지선 x최소 라이다 좌표 - 거리 계산 위함
                    for index, (mask, _cls) in enumerate(zip(masks, clss)):

                        # class id 8(정지선) 이고 신뢰도 50프로 이상 
                        if _cls == 8 and boxes[index].conf > 0.5:        #and boxes[index].conf > 0.5
    
                            # 원본 이미지 시각화 + mask 좌표들 BEV변환    
                            for i in range(1, len(mask)):
                                x1, y1 = mask[i-1]
                                x2, y2 = mask[i]
                                cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), color = (0,0,255), thickness=2, lineType=8, shift=0)

                            mask_points = [(x1, y1 - IMAGE_HEIGHT * CUT_ROAD_RATIO) for (x1, y1) in mask]
                            mask_points = np.array(mask_points, dtype=np.float32)

                            # reshape하여 (N, 1, 2) 형태로 변환
                            mask_points = mask_points.reshape(-1, 1, 2)
                            if mask_points.size == 0:
                                continue
                            transformed_points = cv2.perspectiveTransform(mask_points, M).reshape(-1, 2)
                            x_list = transformed_points[:, 0]
                            y_list = -(transformed_points[:, 1] - WARP_SIZE)
                            lidar_points = util.convert_point_bevcam2lidar(x_list, y_list)
                            lidar_points = [pt for pt in lidar_points if np.polyval(right_lane_lidar_fit, pt[0]) <= pt[1] <= np.polyval(left_lane_lidar_fit, pt[0])]
                            
                            x_list = [pt[0] for pt in lidar_points]
                            if len(x_list) > 0:
                                x_avg = (min(x_list) + max(x_list)) / 2
                                x_avg_list.append(x_avg)

                    if len(x_avg_list) > 0:
                        if stop_distance is not None:
                            stop_distance_list = [abs(stop_distance - pt) for pt in x_avg_list]
                            current_stop_distance = x_avg_list[np.argmin(stop_distance_list)]
                            if stop_distance > 6:
                                stop_distance = current_stop_distance
                            else:
                                if abs(stop_distance - current_stop_distance) < 1:
                                    stop_distance = current_stop_distance
                        else:
                            if min(x_avg_list) <= 16:
                                stop_distance = min(x_avg_list)
                        last_time = time.time()
                        #print('stopline!!! : {:7.4f}'.format(stop_distance))
                            


            # frame = cv2.resize(frame, (640, 360))
            # warp_img = cv2.resize(warp_img, (360, 360))
            # cv2.imshow('seg_frame', frame)
            # cv2.imshow('warp_img', warp_img)
            # if cv2.waitKey(25) == ord('q'):
            #     break

        else:
            stop_distance = None
            time.sleep(0.1)