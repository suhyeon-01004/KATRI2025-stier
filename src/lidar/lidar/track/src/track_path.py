#!/usr/bin/env python
# -*- coding: utf-8 -*-

#라바콘 위치, 색깔 정보로 차량 추종점을 생성하고 보간법을 적용하여 Local Path를 생성하는 노드

import rospy
import math as m
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from std_msgs.msg import String
from track.msg import PathInfo_track
from object_detector.msg import ObjectInfo

from scipy.spatial import Delaunay
from scipy.interpolate import splrep, splev, interpolate


# Local Path Waypoint 개수
linspace_num = 20

class Cone:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.color = String()
        self.dis = 0
        self.side_dis = 0
        self.idx = 0


class Left_Right:
    
    def __init__(self):
        self.lidar_ob = []
        self.cones,self.yellow,self.blue,self.gray = self.color_division()
        self.gray_pre = []


    def cal_dis(self,a,b):

        dis = m.sqrt(pow((a.x-b.x),2)+pow((a.y-b.y),2))

        return dis

    # # 장애물 받아오는 함수
    def callback_ob(self, msg):
        lidar_ob = []
        cenx = msg.centerX
        ceny = msg.centerY
        lenx = msg.lengthX
        lenz = msg.lengthZ

        for i in range(msg.objectCounts):
            #cenx, ceny로 ROI를 설정, lenz를 라바콘 사이즈 맞춰서 라바콘만 인식되게 만듬.
            if abs(cenx[i])<10.0 and abs(ceny[i])<4.0:
                if lenz[i]<0.55 and lenz[i]>0.15:
                    cone = Cone(cenx[i],ceny[i])
                else:
                    continue
            else:
                continue
            
            cone.ratio = lenx[i]/lenz[i]
            cone.color = 'gray'

            lidar_ob.append(cone)

        self.lidar_ob = lidar_ob
    # 여기까지는 object_detector에서 받은 정보를 가지고 lidar_ob에 cone 정보를 저장


    # 이후 저장된 콘들을 좌우 구분하는 부분
    # 여기 아래에 새로운 알고리즘을 넣어주면 될듯
    # 새로운 알고리즘 짤 때, 오른쪽이 blue, 좌측이 yellow로 해서 넣어주면 편할듯?

    # def color_division(self):
    #     novision_y = 3.0
    #     novision_x = 2.5
    #
    #     l_ob = self.lidar_ob
    #
    #     left_gray = []
    #     right_gray = []
    #
    #     def process_gray_cones(gray_cones, target_color, target_list):
    #         if target_color == 'blue':
    #             std_p = Cone(0.0,-1.0)
    #         else:
    #             std_p = Cone(0.0,1.0)
    #
    #         if len(gray_cones) != 0:
    #             for g in gray_cones:
    #                 g.side_dis = self.cal_dis(g,std_p)
    #
    #             sorted_gray_cones = sorted(gray_cones, key=lambda x: x.side_dis)
    #             # print([(a.x,a.y)for a in sorted_gray_cones], target_color)
    #             target_idx = sorted_gray_cones[0].idx
    #             l_ob[target_idx].color = target_color
    #             target_list.append(l_ob[target_idx])
    #             # print((sorted_gray_cones[0].x, sorted_gray_cones[0].y))
    #             del sorted_gray_cones[0]
    #         else:
    #             sorted_gray_cones = gray_cones
    #
    #         return sorted_gray_cones,target_list
    #
    #     for i in range(len(l_ob)):
    #         for a in v_ob:
    #             if a.y==480.0:
    #                 continue
    #             else:
    #                 d = self.cal_dis(a,l_ob[i])
    #                 if d<v_to_l_dis:
    #                     l_ob[i].color = a.color
    #                     l_ob[i].idx = i
    #                     if a.color == 'yellow':
    #                         yellow.append(l_ob[i])
    #                     elif a.color == 'blue':
    #                         blue.append(l_ob[i])
    #                 else:
    #                     continue
    #
    #
    #         if l_ob[i].color not in ['blue','yellow'] and abs(l_ob[i].y)<novision_y and l_ob[i].x<novision_x:
    #             l_ob[i].dis = self.cal_dis(zero,l_ob[i])
    #             l_ob[i].idx = i
    #
    #             if l_ob[i].y>0:
    #                 left_gray.append(l_ob[i])
    #             else:
    #                 right_gray.append(l_ob[i])
    #
    #     sorted_right_gray, edit_b= process_gray_cones(right_gray, 'blue', blue)
    #     sorted_left_gray, edit_y = process_gray_cones(left_gray, 'yellow', yellow)
    #
    #     gray = sorted_left_gray+sorted_right_gray
    #     self.gray_pre = gray
    #
    #     eedit_y, eedit_b, edit_gray = self.remove_gray_sm(edit_y,edit_b, gray)
    #
    #     cones = eedit_b+eedit_y+edit_gray
    #
    #     return cones, eedit_y, eedit_b, edit_gray
    #
    #
    # def remove_gray_sm(self,y,b,g):
    #
    #     data = y+b
    #
    #     # print('remove start!')
    #
    #     ee_y = y
    #     ee_b = b
    #     ee_gray = [] + g
    #     cnt = 0
    #
    #     for gp in g:
    #         i = g.index(gp)-cnt
    #         # print('i', i)
    #         gray_dis = [self.cal_dis(gp,data[i]) for i in range(len(data))]
    #         # print(gray_dis)
    #         idx = gray_dis.index(min(gray_dis))
    #
    #         new_cone = gp
    #         new_cone.color = data[idx].color
    #         if new_cone.color == 'yellow':
    #             # print('yellow to gray', (new_cone.x,new_cone.y))
    #             ee_y.append(new_cone)
    #             del ee_gray[i]
    #             # print(len(y),len(ee_y))
    #             cnt = cnt+1
    #         elif new_cone.color == 'blue':
    #             # print('blue to gray', (new_cone.x,new_cone.y))
    #             ee_b.append(new_cone)
    #             del ee_gray[i]
    #             cnt = cnt+1
    #         else:
    #             continue
    #
    #     return ee_y,ee_b,ee_gray
    #
    #
    # def which_color(self,cone):
    #
    #     yellow = []
    #     blue = []
    #     gray = []
    #
    #     for c in cone:
    #         if c.color == "yellow":
    #             yellow.append(c)
    #         elif c.color == "blue":
    #             blue.append(c)
    #         else:
    #             gray.append(c)
    #
    #     return yellow, blue, gray
    #
    # def publish_set(data):
    #
    #     loc = []
    #
    #     for i in range(len(data)):
    #         ob = Cone(data[i].x,data[i].y)
    #         ob.color = data[i].color
    #         loc.append(ob)
    #
    #     obs = Cone_loc()
    #     obs = loc
    #
    #     return obs


class drawing_path():

    def __init__(self,yellow, blue, lidar):

        self.obstacle = lidar
        self.right_data = blue
        self.left_data = yellow

        self.center = []

        self.local_path = PathInfo_track()

        self.path_x = []
        self.path_y = []     


    def triangulation(self, ob):

        coordinate = []

        for a in ob:
            coordinate.append((a.x,a.y))

        points = np.array(coordinate)
        center_x = []
        center_y = []

        tri = Delaunay(points)
        idx = tri.simplices

        center_x = []
        center_y = []

        for i in idx :
            for j in range(3):
                if ob[i[j]].color != ob[i[j-1]].color :
                    center_x.append((ob[i[j]].x+ob[i[j-1]].x)/2)
                    center_y.append((ob[i[j]].y+ob[i[j-1]].y)/2)
        
        return center_x, center_y, idx
    

    def making_path(self):

        def to_track_msg(x_list,y_list):

            path_msg = PathInfo_track()

            path_msg.cnt = len(y_list)
            path_msg.x = x_list
            path_msg.y = y_list

            return path_msg
        
        cone_data = self.right_data+self.left_data

        local_path = PathInfo_track()

        if len(cone_data) > 2: 

            center_x, center_y, idx = self.triangulation(cone_data)
            self.center = list(zip(center_x, center_y))
 
            if len(center_x) * len(center_y) != 0 :

                sorted_indices = np.argsort(center_x)
                s_cx = np.array(center_x)[sorted_indices]
                s_cy = np.array(center_y)[sorted_indices]

                sorted_cx = s_cx.tolist()
                sorted_cy = s_cy.tolist()

                cnt = 0
                for i in range(len(sorted_cx)):
                    if i != 0:
                        idx = i=cnt
                        if sorted_cx[idx-1]==sorted_cx[idx]:
                            del sorted_cx[i]
                            del sorted_cy[i]
                            cnt = cnt+1
                center_x = sorted_cx
                center_y = sorted_cy
                
            else :
                center_x = [0.0]
                center_y = [0.0]

        elif len(cone_data) == 2 and cone_data[0].color!=cone_data[1].color:
            a = cone_data[0]
            b = cone_data[1]
            center_x = [(a.x+b.x)/2]
            center_y = [(a.y+b.y)/2]     
        
        else:
            center_x = [0.0]
            center_y = [0.0]

        x_new = [0.0 for _ in range(linspace_num)]
        y_new = [0.0 for _ in range(linspace_num)]
        print(len(center_x))
        for j in range(len(center_x)):
            if j>19:
                break
            x_new[j] = center_x[j]
            y_new[j] = center_y[j]

        local_path = to_track_msg(x_new,y_new)

        return local_path, x_new, y_new, center_x, center_y


###########for visualize
def vis_set(data,cz,r,g,b,x,y,z):

    array = MarkerArray()
    i = 0

    for pose in data:
        marker = Marker()
        marker.header.frame_id = "velodyne"
        marker.header.stamp = rospy.Time()
        marker.id = i
        if type(pose) is not tuple:
            marker.type = 1
            marker.pose.position.x = pose.x
            marker.pose.position.y = pose.y
            marker.pose.position.z = cz
        elif type(pose) is tuple:
            marker.type = 2
            marker.pose.position.x = pose[0]
            marker.pose.position.y = pose[1]
            marker.pose.position.z = 0
        marker.scale.x = x
        marker.scale.y = y
        marker.scale.z = z
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.lifetime = rospy.Duration(0.2)
        array.markers.append(marker)
        i = i+1
        
    return array

if __name__ == '__main__':
            
    lr = Left_Right()

    rospy.init_node('track_path')

    rospy.Subscriber('/object_info', ObjectInfo, lr.callback_ob)

    # rviz 상에서 확인하기 위해 pub하는 부분
    gray_vis_pub = rospy.Publisher("/gray_marker", MarkerArray, queue_size=1)
    blue_vis_pub = rospy.Publisher("/blue_marker", MarkerArray, queue_size=1)
    yellow_vis_pub = rospy.Publisher("/yellow_marker", MarkerArray, queue_size=1)
    vision_vis_pub = rospy.Publisher("/vision_marker", MarkerArray, queue_size=1)
    center_vis_pub = rospy.Publisher("/center_marker", MarkerArray, queue_size=10)
    path_vis_pub = rospy.Publisher("/path_marker", MarkerArray, queue_size=10)

    # /local_path 노드에서 PathInfo_track 메시지를 발행, pure_pursuit_track 노드에서 subscribe
    path_pub = rospy.Publisher('/local_path', PathInfo_track, queue_size = 10)

    zero = Cone(0.0,0.0)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        lidar, yellow, blue, gray = lr.color_division()
        
        vision_vis = vis_set(lr.vision_ob, 0.0, 1.0, 0.0, 0.0, 0.4,0.4,0.1)
        vision_vis_pub.publish(vision_vis)

        yellow_vis = vis_set(yellow, 0.0, 1.0, 1.0, 0.0, 0.2,0.2,0.2)
        blue_vis = vis_set(blue, 0.0, 0.0, 0.0, 1.0, 0.2,0.2,0.2)
        gray_vis = vis_set(gray, 0.0, 0.5, 0.5, 0.5, 0.2,0.2,0.2)
        yellow_vis_pub.publish(yellow_vis)
        blue_vis_pub.publish(blue_vis)
        gray_vis_pub.publish(gray_vis)

        dp = drawing_path(yellow,blue,lidar)

        if len(lidar) != 0:
            local_path,x,y,center_x, center_y = dp.making_path()

            center_vis = vis_set(list(zip(center_x,center_y)),0.0,0.0,1.0,0.5,0.2,0.2,0.2)
            center_vis_pub.publish(center_vis)

            waypoint = list(zip(x,y))
            path = vis_set(waypoint,0.0, 0.5,0.0,1.0,0.4,0.4,0.1)
            path_vis_pub.publish(path)

            path_pub.publish(local_path)

        rate.sleep()
