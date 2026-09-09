#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Int32, Float32

import threading
from ultralytics import YOLO
from pathlib import Path
import numpy as np
import time
import cv2

idx_to_class = ['red', 'off', 'yellow', 'green', 'red_left', 'green_left', 'yellow_green', 'yellow_left', 'red_yellow_left']

def callback_current_idx(data):
    global current_idx
    current_idx = data.data

def callback_speed(data):
    global current_speed
    current_speed = data.data  # Km/h

def callback_stopline_distance(data):
    global stopline_distance
    stopline_distance = data.data

def transform_to_publish_format(light_str):
    ret_val = None
    if light_str == 'red':
        ret_val = '10000'
    elif light_str == 'off':
        ret_val = '00000'
    elif light_str == 'yellow':
        ret_val = '01000'
    elif light_str == 'green':
        ret_val = '00010'
    elif light_str == 'red_left':
        ret_val = '10100'
    elif light_str == 'green_left':
        ret_val = '00110'
    elif light_str == 'yellow_green':
        ret_val = '01010'
    elif light_str == 'yellow_left':
        ret_val = '01100'
    elif light_str == 'red_yellow_left':
        ret_val = '11100'

    return ret_val

def check_light_section():
    global current_idx

    if (335 <= current_idx < 370) or (423 <= current_idx < 462) or\
        (655 <= current_idx < 697) or (840 <= current_idx < 885) or (960 <= current_idx < 1020) or \
        (1496 <= current_idx < 1538) or (1580 <= current_idx < 1619):
        light_section = True
    else:
        light_section = False

    return light_section

def check_yellow_dillema_section():
    global current_idx

    if (335 <= current_idx < 370) or (423 <= current_idx < 462) or\
        (655 <= current_idx < 697) or (1496 <= current_idx < 1538) or (1580 <= current_idx < 1619):
        dillema_section = True
    else:
        dillema_section = False

    return dillema_section

def check_current_signal():
    global current_idx

    current_signal = None
    if (335 <= current_idx < 370) or (840 <= current_idx < 885) or (960 <= current_idx < 1020):
        current_signal = 'left'
    elif (423 <= current_idx < 462) or\
        (655 <= current_idx < 697) or (1496 <= current_idx < 1538) or (1580 <= current_idx < 1619):
        current_signal = 'straight'

    return current_signal

def publish_crack():
    signal_start_time = time.time()
    while True:
        time_elapsed = time.time() - signal_start_time
        crack_pub.publish(time_elapsed)
        time.sleep(0.1)

        if time_elapsed > 300:
            break

if __name__ == '__main__':
    rospy.init_node('traffic_light', anonymous=True)

    # ROS Publisher
    light_pub = rospy.Publisher('/sign', String, queue_size=10)
    crack_pub = rospy.Publisher('/crack', Float32, queue_size=1)

    # ROS Subscriber
    rospy.Subscriber('/gps_speed', Float32, callback_speed)
    rospy.Subscriber('/current_idx', Int32, callback_current_idx)
    rospy.Subscriber('/stopline_distance', Float32, callback_stopline_distance)

    crack_pub.publish(300)

    traffic_light_model = YOLO('{}/catkin_ws/ModelFiles/TrafficLight_YOLOv8n.pt'.format(Path.home()))
    traffic_light_model.predict(np.zeros((720, 1280, 3)), verbose=False)

    current_speed = 0
    current_idx = 0
    stopline_distance = None
    yellow_start_time = None
    current_signal = None
    light_started = True
    first_yellow = None
    previous_light_sign = None
    bridge = CvBridge()
    crack_started = False

    while not rospy.is_shutdown():

        if not check_light_section():
            print('not light section')
            light_started = True
            yellow_start_time = None
            light_pub.publish('00000')

            image_msg = rospy.wait_for_message("/usb_cam2/image_raw", Image)
            frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            frame = cv2.resize(frame, (640, 360))
            cv2.imshow('frame', frame)
            if cv2.waitKey(25) == ord('q'):
                break

            continue

        image_msg = rospy.wait_for_message("/usb_cam2/image_raw", Image)
        frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        # YOLO Detection
        result = traffic_light_model.predict(frame, verbose=False)[0]

        boxes = result.boxes.xyxy
        classes = [int(pt) for pt in result.boxes.cls]
        confs = [int(conf*100) for conf in result.boxes.conf]

        width_list = list()
        light_centers = list()
        for i, box in enumerate(boxes):
            if classes[i] == 1:  # Off는 X
                continue

            box = [int(pt) for pt in box]
            start = (box[0], box[1])
            end = (box[2], box[3])

            width = end[0] - start[0]
            height = end[1] - start[1]

            width_list.append(width)
            light_centers.append((start[1] + end[1])/2)

            class_ = idx_to_class[classes[i]]
            color = (255, 255, 255)

            cv2.rectangle(frame, start, end, color=color, thickness=2)
            cv2.putText(frame, class_, (box[0], box[3]+15), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)
            cv2.putText(frame, '({}%)'.format(confs[i]), (box[0], box[3]+40), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)


        if len(light_centers) == 0:
            print('No Light!!')
        else:
            if (655 <= current_idx < 697) or (840 <= current_idx < 885):  # 버스 신호등 인식 방지
                max_idx = np.argmax(width_list)  # 가장 너비가 긴 신호등만 남김
                light_centers = [light_centers[max_idx]]
                boxes = [boxes[max_idx]]
                classes = [classes[max_idx]]
            
            closest_idx = np.argmin(light_centers)  # 가장 위에 있는 신호등 선택
            current_light_sign = idx_to_class[classes[closest_idx]]
            cv2.putText(frame, current_light_sign, (10, 70), cv2.FONT_HERSHEY_DUPLEX, 3, (255,255,255), 2)

            box = [int(pt) for pt in boxes[closest_idx]]
            start = (box[0], box[1])
            end = (box[2], box[3])
            cv2.rectangle(frame, start, end, color=(0, 0, 255), thickness=2)

            light_info = transform_to_publish_format(current_light_sign)

            if light_started:
                light_started = False
                if current_light_sign == 'yellow':
                    first_yellow = True
                else:
                    first_yellow = False

            if current_speed >= 5 and not first_yellow and check_yellow_dillema_section():
                if current_light_sign == 'yellow':
                    if yellow_start_time is None:
                        yellow_start_time = time.time()

                    yellow_remain_time = 3.0 - (time.time() - yellow_start_time)
                    if ((current_speed/3.6) * yellow_remain_time) > (stopline_distance+1):
                        if check_current_signal() == 'left':
                            light_info = '00100'
                        elif check_current_signal() == 'straight':
                            light_info = '00010'

            if (840 <= current_idx < 885):
                if previous_light_sign is None:
                    previous_light_sign = current_light_sign

                if previous_light_sign == 'red_left' and (current_light_sign == 'red' or current_light_sign == 'yellow'):
                    yellow_start_time = time.time()
                if current_light_sign == 'red_left':
                    yellow_start_time = None

                if yellow_start_time is not None:
                    yellow_remain_time = 3.0 - (time.time() - yellow_start_time)
                    if ((current_speed/3.6) * yellow_remain_time) > stopline_distance:
                        light_info = '00100'

                previous_light_sign = current_light_sign


            if (1496 <= current_idx < 1538) and len(light_centers) > 1:  # 직진 연속에서 빨리 켜지는 신호등 인식
                indices = np.argsort(width_list)[-2:][::-1]
                if idx_to_class[classes[indices[0]]] == 'green_left' or idx_to_class[classes[indices[1]]] == 'green_left':
                    light_info = '00010'

            if 960 <= current_idx < 1020:
                if not crack_started and current_light_sign=='red_left':
                    crack_started = True
                    threading.Thread(target=publish_crack).start()




            light_pub.publish(light_info)

        frame = cv2.resize(frame, (640, 360))
        cv2.imshow('frame', frame)
        if cv2.waitKey(25) == ord('q'):
            break