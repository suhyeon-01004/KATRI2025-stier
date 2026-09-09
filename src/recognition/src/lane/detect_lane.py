#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

import os
import cv2
import torch.backends.cudnn as cudnn
import argparse
import numpy as np
import time
import sys
import json
sys.path.append('/home/stier/catkin_ws/src/recognition/src/lane')

from clrnet.utils.config import Config
from clrnet.engine.runner import Runner


def calculate_lane():

    args = parse_args()
    
    os.environ["CUDA_VISIBLE_DEVICES"] = '0' # 사용하고자 하는 특정 gpu 
    cfg = Config.fromfile((os.path.dirname(os.path.abspath(__file__))) +\
        "/configs/clrnet/clr_resnet34_tusimple.py") # 모델 아키텍처 지정
    
    cfg.gpus = 1 # gpu 개수 지정

    cfg.load_from = '/home/stier/catkin_ws/src/recognition/src/lane/69.pth' # pt파일 경로
    
    cfg.resume_from = args.resume_from
    cfg.finetune_from = args.finetune_from
    cfg.view = args.view # 시각화
    cfg.seed = args.seed
    
    cfg.work_dirs = args.work_dirs if args.work_dirs else cfg.work_dirs

    cudnn.benchmark = True

    runner = Runner(cfg)

    lane_info_publisher = rospy.Publisher('lane_data_publisher', String, queue_size=5)
    bridge = CvBridge()

    while True:
        image_msg = rospy.wait_for_message('/usb_cam2/image_raw', Image)
        frame = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        lanes_xys = runner.detect(frame)
        lane_points = list()
        for points in lanes_xys:
            if points and len(points)>1:
                lane_points.append(points)

        lane_info_str = json.dumps(lane_points)
        lane_info_publisher.publish(lane_info_str)

        print(lane_info_str, end='\n\n')
        

def parse_args():
    parser = argparse.ArgumentParser(description='Train a detector')

    parser.add_argument('--work_dirs',
                        type=str,
                        default=None,
                        help='work dirs')
    parser.add_argument('--load_from',
                        default=None,
                        help='the checkpoint file to load from')
    parser.add_argument('--resume_from',
            default=None,
            help='the checkpoint file to resume from')
    parser.add_argument('--finetune_from',
            default=None,
            help='the checkpoint file to resume from')
    parser.add_argument('--view', action='store_false', help='whether to view')
    parser.add_argument(
        '--validate',
        action='store_true',
        help='whether to evaluate the checkpoint during training')
    parser.add_argument(
        '--test',
        action='store_true',
        help='whether to test the checkpoint on testing set')
    parser.add_argument('--gpus', nargs='+', type=int, default='0')
    parser.add_argument('--seed', type=int, default=0, help='random seed')
    args = parser.parse_args(rospy.myargv()[1:])

    return args


if __name__ == '__main__':
    rospy.init_node('lane_info_publisher_node', anonymous=True)

    calculate_lane()
