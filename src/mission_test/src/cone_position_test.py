#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from visualization_msgs.msg import Marker, MarkerArray
from object_detector.msg import ObjectInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO
import numpy as np
import cv2

from pathlib import Path
import sys
sys.path.append('{}/catkin_ws/src/mission/src'.format(Path.home()))
from mission_utils import *


# 왼쪽 상단이 원점인 BEV 이미지를 라이다 좌표계로 변환
def convert_point_bevcam2lidar(pixel_coord):

    x = pixel_coord[0]
    y = pixel_coord[1]

    y = -(y - 720)
    x = -(x - 360)
    x = x * XM_PER_PIXEL
    y = y * YM_PER_PIXEL
    x, y = y, x
    x += CAR_OFFSET

    return [x, y]

def visualization_set(data, cz, r, g, b, x, y, z):
    array = MarkerArray()
    for i, point in enumerate(data):
        marker = Marker()
        marker.header.frame_id = "velodyne"
        marker.header.stamp = rospy.Time()
        marker.id = i
        marker.type = Marker.CUBE
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = cz[i]
        marker.scale.x = x[i]
        marker.scale.y = y[i]
        marker.scale.z = z[i]
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.lifetime = rospy.Duration(0.1)
        array.markers.append(marker)
    return array

if __name__ == '__main__':
    rospy.init_node('cone_position_test', anonymous=True)

    src = np.float32([[495, 30], [49, 431], [812, 30], [1257, 429]])   # 카메라 위치가 바뀌면 바꿔줘야 함
    dst = np.float32([[150, 0], [150, 720], [570, 0], [570, 720]])
    M = cv2.getPerspectiveTransform(src, dst)

    cone_pos_pub = rospy.Publisher("/cone_pos_pub", MarkerArray, queue_size=1)

    image_width, image_height = 1280, 720
    model = YOLO('/home/stier/catkin_ws/ModelFiles/RubberCone_YOLOv10m_1280.pt')

    bridge = CvBridge()
    while True:
        image_msg = rospy.wait_for_message('/usb_cam1/image_raw', Image)
        frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        result = model.predict(frame)[0]
        boxes = result.boxes.xyxy
        classes = [int(pt) for pt in result.boxes.cls]
        confs = [int(conf * 100) for conf in result.boxes.conf]
        lidar_coord_list = list()
        transformed_points_list = list()

        for i, box in enumerate(boxes):
            box = [int(pt) for pt in box]
            start = (box[0], box[1])
            end = (box[2], box[3])

            cone_pixels = frame[int(0.4 * box[1] + 0.6 * box[3]):int(0.1 * box[1] + 0.9 * box[3]),
                                int(0.6 * box[0] + 0.4 * box[2]):int(0.4 * box[0] + 0.6 * box[2]), :]
            cone_pixels = cone_pixels.reshape(-1, 3)

            B_avg = sum(cone_pixels[:, 0]) / len(cone_pixels)
            G_avg = sum(cone_pixels[:, 1]) / len(cone_pixels)
            R_avg = sum(cone_pixels[:, 2]) / len(cone_pixels)
            blue_distance = (255 - B_avg) ** 2 + G_avg ** 2 + R_avg ** 2
            yellow_distance = B_avg ** 2 + (255 - G_avg) ** 2 + (255 - R_avg) ** 2
            orange_distance = B_avg**2 + G_avg**2 + (255-R_avg)**2

            if B_avg > 100:
                color_type = 'blue'
            else:
                if abs(G_avg - R_avg) > 80:
                    color_type = 'orange'
                else:
                    color_type = 'yellow'

            distance_list = [orange_distance, blue_distance, yellow_distance]

            if color_type == 'blue':  # Blue Cone
                color = (255, 0, 0)
                cv2.rectangle(frame, start, end, color=color, thickness=2)
                cv2.putText(frame, 'Blue', (box[0], box[3] + 15), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)
                cv2.putText(frame, '({}%)'.format(confs[i]), (box[0], box[3] + 40), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)

            elif color_type == 'yellow':
                color = (0, 255, 255)
                cv2.rectangle(frame, start, end, color=color, thickness=2)
                cv2.putText(frame, 'Yellow', (box[0], box[3] + 15), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)
                cv2.putText(frame, '({}%)'.format(confs[i]), (box[0], box[3] + 40), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)

            elif color_type == 'orange':
                color = (0, 165, 255)
                cv2.rectangle(frame, start, end, color=color, thickness=2)
                cv2.putText(frame, 'Orange', (box[0], box[3] + 15), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)
                cv2.putText(frame, '({}%)'.format(confs[i]), (box[0], box[3] + 40), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)

            pixel_coord = [[int((start[0]+end[0])/2), int(end[1]-0.4*image_height)]]
            points = np.array(pixel_coord, dtype=np.float32)
            points = points.reshape(-1, 1, 2)
            transformed_points = cv2.perspectiveTransform(points, M).reshape(-1, 2)[0]
            print('transformed_points :', transformed_points)
            cv2.circle(frame, pixel_coord[0], 5, (0,0,255), 5)
            lidar_coord = convert_point_bevcam2lidar(transformed_points)
            lidar_coord_list.append(lidar_coord)
            cv2.putText(frame, '({:.2f}, {:.2f})'.format(lidar_coord[0], lidar_coord[1]), (box[0], box[1] - 40), cv2.FONT_HERSHEY_DUPLEX, 0.6, color)
            print('lidar_coord :', lidar_coord)
            transformed_points_list.append(transformed_points)
        

        ob_x = [pt[0] for pt in lidar_coord_list]
        ob = visualization_set(lidar_coord_list, [0.1]*len(ob_x), 0.0, 1.0, 0.0, [0.1]*len(ob_x), [0.1]*len(ob_x), [0.1]*len(ob_x))
        cone_pos_pub.publish(ob)

        img = np.zeros((720, 720, 3))
        for pt in transformed_points_list:
            cv2.circle(img, (int(pt[0]), int(pt[1])), 5, (255,255,255), 3)

        lane_image = frame[int(0.4*image_height):, :]
        warp_img = cv2.warpPerspective(lane_image, M, (720, 720), flags=cv2.INTER_LINEAR)

        cv2.imshow('warp_img', warp_img)
        cv2.imshow('frame', frame)
        cv2.imshow('img', img)
        if cv2.waitKey(25) == ord('q'):
            break

            


