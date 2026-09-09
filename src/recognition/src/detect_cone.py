#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 라바콘 YOLO로 인식해서 박스 정보 Publish하는 코드

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

from ultralytics import YOLO
import numpy as np
import json
import cv2


if __name__ == '__main__':
    rospy.init_node('traffic_light_yolo_publisher', anonymous=True)

    # ROS Publisher
    cone_predict_pub = rospy.Publisher('cone_predict_pub', String, queue_size=10)

    # YOLO Model
    cone_model = YOLO('/home/stier/catkin_ws/ModelFiles/RubberCone_YOLOv10m_1280.pt')

    result_dict = {'boxes':list(), 'classes':list(), 'confs':list(), 'colors':list()}
    bridge = CvBridge()
    while True:
        image_msg = rospy.wait_for_message("/usb_cam1/image_raw", Image)
        frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        # YOLO Detection
        result = cone_model.predict(frame, verbose=True)[0]

        boxes = [[int(pt) for pt in box] for box in result.boxes.xyxy]
        classes = [int(pt) for pt in result.boxes.cls]
        confs = [int(conf*100) for conf in result.boxes.conf]

        color_list = list()
        for i, box in enumerate(boxes):
            cone_pixels = frame[int(0.4 * box[1] + 0.6 * box[3]):int(0.1 * box[1] + 0.9 * box[3]),
                                    int(0.6 * box[0] + 0.4 * box[2]):int(0.4 * box[0] + 0.6 * box[2]), :]
            cone_pixels = cone_pixels.reshape(-1, 3)

            B_avg = sum(cone_pixels[:, 0]) / len(cone_pixels)
            G_avg = sum(cone_pixels[:, 1]) / len(cone_pixels)
            R_avg = sum(cone_pixels[:, 2]) / len(cone_pixels)
            blue_distance = (255 - B_avg) ** 2 + G_avg ** 2 + R_avg ** 2
            yellow_distance = B_avg ** 2 + (255 - G_avg) ** 2 + (255 - R_avg) ** 2

            if yellow_distance > blue_distance:  # Blue Cone
                color_list.append('blue')
            else:
                color_list.append('yellow')


        result_dict['boxes'] = boxes
        result_dict['classes'] = classes
        result_dict['confs'] = confs
        result_dict['colors'] = color_list

        json_result = json.dumps(result_dict)
        cone_predict_pub.publish(json_result)
