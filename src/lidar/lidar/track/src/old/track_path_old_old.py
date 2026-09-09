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
from vision.msg import Cone_loc
from object_detector.msg import ObjectInfo

from scipy.spatial import Delaunay
from scipy.interpolate import splrep, splev


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
        self.vision_ob = []    

        self.cones,self.yellow,self.blue,self.gray = self.color_division()


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
            if abs(cenx[i])<10.0 and abs(ceny[i])<4.0:
                if lenz[i]<0.55 and lenz[i]>0.15:
                    # if lenx[i]/lenz[i]<0.6:
                    #     cone = Cone(cenx[i],ceny[i])
                    # else:
                    #     continue
                    cone = Cone(cenx[i],ceny[i])
                else:
                    continue
            else:
                continue
            
            cone.ratio = lenx[i]/lenz[i]
            cone.color = 'gray'

            lidar_ob.append(cone)

        self.lidar_ob = lidar_ob

    
    def callback_vision_ob(self,msg):

        cones = msg.location
        vision_ob = []

        x_cell = 640 ## 버드뷰 가로 픽셀 수
        y_cell = 480 ## 버드뷰 세로 픽셀 수
        zero_gap = 2.6 ## 0m~2.6m 간 픽셀(예상)
        x_onemeter = 85 ##라이다 x축 (버드뷰 세로) 1m 몇픽셀?
        y_onemeter = 54 ##라이다 y축 (버드뷰 가로) 1m 몇픽셀?

        for i in range(len(cones)):

            cone = Cone((-cones[i].y+y_cell)/x_onemeter+zero_gap,((-cones[i].x+x_cell/2)/y_onemeter)) ###좌표변환 결과
            cone.color = cones[i].color
            vision_ob.append(cone)
        
        self.vision_ob = vision_ob


    def color_division(self):

        v_to_l_dis = 0.4
        novision_y = 3.0
        novision_x = 2.5

        v_ob = self.vision_ob
        l_ob = self.lidar_ob

        left_gray = []
        right_gray = []

        yellow = []
        blue = []

        def process_gray_cones(gray_cones, target_color, target_list):
            if len(gray_cones) != 0:
                sorted_gray_cones = sorted(gray_cones, key=lambda x: x.dis)
                # print([(a.x,a.y)for a in sorted_gray_cones], target_color)
                target_idx = sorted_gray_cones[0].idx
                l_ob[target_idx].color = target_color
                target_list.append(l_ob[target_idx])
                # print((sorted_gray_cones[0].x, sorted_gray_cones[0].y))
                del sorted_gray_cones[0]
            else:
                sorted_gray_cones = gray_cones

            return sorted_gray_cones,target_list

        def remove_gray(data, gray):

            min_dis_gray = 0.5

            def cal_line_dis(dt,dt_list,gr):

                cnt_g = 0
                for g in range(len(gr)):
                    for i in range(len(dt)):
                        idx = g-cnt_g
                        if i !=0 and ((dt[i-1].y<gr[idx].y<dt[i].y or dt[i].y<gr[idx].y<dt[i-1].y) and (dt[i-1].x<gr[idx].x<dt[i].x or dt[i].x<gr[idx].x<dt[i-1].x)):
                            line_dis = dis_line(dt[i-1],dt[i],gr[idx])
                            if line_dis<1.2:
                                gr[idx].color = dt[i-1].color
                                dt_list.append(gr[idx])
                                del gr[idx]
                                cnt_g = cnt_g+1
                                break

                print('cnt:',cnt_g)
            
                return dt_list, gr
            
            data_list = []

            if len(data)>1:

                sorted(data, key=lambda x: x.dis)
                cnt = 0
                for i in range(len(data)):
                    if i !=0:
                        side_dis = self.cal_dis(data[i-1-cnt], data[i-cnt])
                        if side_dis<4.0:
                            data[i].side_dis = side_dis
                            data_list.append(data[i])
                        else:
                            del data[i]
                            cnt = cnt+1
                            continue

                edit_data, edit_gray = cal_line_dis(data_list,data, gray)

            elif len(data) == 1 and len(gray) != 0:

                dis = []

                for g in gray:
                    gray_dis = self.cal_dis(data[0],g)
                    if gray_dis < min_dis_gray:
                        dis.append(gray_dis)
                    else:
                        dis.append(min_dis_gray)

                if min(dis) != min_dis_gray:
                    idx = dis.index(min(dis))
                    cone = gray[idx]
                    cone.color = data[0].color
                    data.append(cone)
                    del gray[idx]

                edit_data, edit_gray = cal_line_dis(data, gray)
        
            else:
                edit_data = data
                edit_gray = gray

            return edit_data, edit_gray

        def dis_line(a,b,g):

            s = (b.y-a.y)/(b.x-a.x)
            d = abs(s*g.x-g.y-s*a.x+a.y)/m.sqrt(pow(s,2)+1)

            return d

        for i in range(len(l_ob)):
            for a in v_ob:
                if a.y==480.0:
                    continue
                else:
                    d = self.cal_dis(a,l_ob[i])
                    if d<v_to_l_dis:
                        l_ob[i].color = a.color
                        l_ob[i].idx = i
                        if a.color == 'yellow':
                            yellow.append(l_ob[i])
                        elif a.color == 'blue':
                            blue.append(l_ob[i])
                    else:
                        continue

                
            if l_ob[i].color == 'gray' and abs(l_ob[i].y)<novision_y and l_ob[i].x<novision_x:
                l_ob[i].dis = self.cal_dis(zero,l_ob[i])
                l_ob[i].idx = i

                if l_ob[i].y>0:
                    left_gray.append(l_ob[i])
                else:
                    right_gray.append(l_ob[i])

        print(0,len(left_gray),len(right_gray))

        sorted_right_gray, edit_b= process_gray_cones(right_gray, 'blue', blue)
        sorted_left_gray, edit_y = process_gray_cones(left_gray, 'yellow', yellow)

        gray = sorted_left_gray+sorted_right_gray 

        print(1,len(edit_y),len(edit_b),len(gray))

        eedit_y, gray_y = remove_gray(edit_y, gray)
        eedit_b, gray_b = remove_gray(edit_b, gray)

        edit_gray = gray_y+gray_b

        cones = eedit_b+eedit_y+edit_gray

        print(len(cones),len(eedit_y),len(eedit_b),len(edit_gray))

        return cones, eedit_y, eedit_b, edit_gray

        
    def which_color(self,cone):

        yellow = []
        blue = []
        gray = []
    
        for c in cone:
            if c.color == "yellow":
                yellow.append(c)
            elif c.color == "blue":
                blue.append(c)
            else:
                gray.append(c)
    
        return yellow, blue, gray

    def publish_set(data):

        loc = []

        for i in range(len(data)):
            ob = Cone(data[i].x,data[i].y)
            ob.color = data[i].color
            loc.append(ob)

        obs = Cone_loc()
        obs = loc
           
        return obs


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
                    if ob[i[j]].color == 'gray' or ob[i[j-1]].color == 'gray':
                        continue
                    else:
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

        local_path = PathInfo_track()

        if len(self.obstacle) > 2: 

            center_x, center_y, idx = self.triangulation(self.obstacle)
            self.center = list(zip(center_x, center_y))
 
            if len(center_x) * len(center_y) != 0 :

                sorted_indices = np.argsort(center_x)
                sorted_cx = np.array(center_x)[sorted_indices]
                sorted_cy = np.array(center_y)[sorted_indices]

                try:
                    tck = splrep(sorted_cx, sorted_cy, s=1)
                    x = np.linspace(min(sorted_cx), max(sorted_cx), linspace_num)
                    y = splev(x, tck, der=0)
                except TypeError:
                    if len(sorted_cx) >1:
                        tck = splrep(sorted_cx, sorted_cy, k=1, s=2)

                        x = np.linspace(min(sorted_cx), max(sorted_cx), linspace_num)
                        y = splev(x, tck, der=0)
                    else:
                        x = sorted_cx
                        y = sorted_cy

                x_new = x
                y_new = y   
                
            else :
                x_new = [0.0 for _ in range(linspace_num)]
                y_new = [0.0 for _ in range(linspace_num)]  

        elif len(self.obstacle) != 0:
            x_new = [0.0 for _ in range(linspace_num)]
            y_new = [0.0 for _ in range(linspace_num)]
            for i in range(len(self.obstacle)):
                x_new[i] = self.obstacle[i].x
                y_new[i] = self.obstacle[i].y
        
        else:
            pass
              
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
    rospy.Subscriber('/Cone', Cone_loc, lr.callback_vision_ob)

    gray_vis_pub = rospy.Publisher("/gray_marker", MarkerArray, queue_size=1)
    blue_vis_pub = rospy.Publisher("/blue_marker", MarkerArray, queue_size=1)
    yellow_vis_pub = rospy.Publisher("/yellow_marker", MarkerArray, queue_size=1)
    vision_vis_pub = rospy.Publisher("/vision_marker", MarkerArray, queue_size=1)
    center_vis_pub = rospy.Publisher("/center_marker", MarkerArray, queue_size=10)
    path_vis_pub = rospy.Publisher("/path_marker", MarkerArray, queue_size=10)
    
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
