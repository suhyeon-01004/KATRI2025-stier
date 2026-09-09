#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 시작할 때 차선 내부에 장애물이 한개도 인식되지 않으면 바로 finished로 들어가서 코드가 동작하지 않을 수 있음

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import time
import cv2

from ultralytics import YOLO
from pathlib import Path
import sys
sys.path.append('{}/catkin_ws/src/mission/src'.format(Path.home()))
from mission_utils import *


if __name__ == '__main__':
    rospy.init_node('rosbag_yolo', anonymous=True)
    model = YOLO('{}/catkin_ws/ModelFiles/RubberCone_YOLOv10m_1280.pt'.format(Path.home()))
    #model = YOLO('/home/siewoo/best.pt')
    #model = YOLO('{}/catkin_ws/ModelFiles/StopLine.pt')
    
    bridge = CvBridge()
    while True:
        image_msg = rospy.wait_for_message("/usb_cam1/image_raw", Image)
        frame = bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # YOLO Detection
        result = model.predict(frame, verbose=False)[0]

        boxes = result.boxes.xyxy
        classes = [int(pt) for pt in result.boxes.cls]
        confs = [int(conf*100) for conf in result.boxes.conf]

        for i, box in enumerate(boxes):
            box = [int(pt) for pt in box]
            start = (box[0], box[1])
            end = (box[2], box[3])

            class_ = str(classes[i])
            color = (255, 255, 255)

            #if 7 <= classes[i] <= 12 and confs[i]>90:

            cv2.rectangle(frame, start, end, color=color, thickness=2)

            cv2.putText(frame, class_, (box[0], box[3]+15), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)
            cv2.putText(frame, '({}%)'.format(confs[i]), (box[0], box[3]+40), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
