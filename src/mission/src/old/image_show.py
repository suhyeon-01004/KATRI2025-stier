import rospy

from sensor_msgs.msg import PointCloud2, Image

from std_msgs.msg import Int32MultiArray, Float32MultiArray

import sensor_msgs.point_cloud2 as pc2

import numpy as np

import cv2

from ultralytics import YOLO

from cv_bridge import CvBridge

from scipy.spatial import distance

from object_detector.msg import ObjectInfo

  

# Initialize the CvBridge

bridge = CvBridge()

frame = None

pixel_coords = None

center_coords = None

  

# Publishers

pub_pointcloud = None

pub_pixel_coords = None

pub_cones_center = None

  

# Camera and LIDAR parameters

rvec = np.array([[-0.05578533, -0.99827818, -0.01812908],
 [ 0.14790644,  0.00969451, -0.98895384],
 [ 0.9874268,  -0.05785053,  0.14711096]]
)

tvec = np.array([[ 0.01902152],
 [ 0.04891112],
 [-0.01215745]])

cameraMatrix = np.array([[759.63868072,   0.,         633.25037   ],
 [  0.,         763.35336903, 367.11064456],
 [  0.,           0.,           1.        ]]
)

  
  

class Cone:

    def __init__(self, x, y):

        self.x = x

        self.y = y

        self.ratio = 0.0

        self.color = 'gray'

  
  

def pixel_coords_callback(data):

    global pixel_coords

    pixel_coords = data.data

  
  

def center_coords_callback(data):

    global center_coords

    center_coords = data.data

  
  

def callback_gray(msg):

    global gray_cones_center, frame

    gray_cones_center = []

  

    cenx = msg.centerX

    ceny = msg.centerY

    lenx = msg.lengthX

    lenz = msg.lengthZ

  

    for i in range(msg.objectCounts):

        # 크기 조건을 만족하고 카메라 뷰 밖에 있는 경우에만 추가

        if abs(cenx[i]) < 10.0 and abs(ceny[i]) < 4.0 and 0.15 < lenz[i] < 0.55:

            cone = Cone(cenx[i], ceny[i])

            cone.ratio = lenx[i] / lenz[i]

  

            # 카메라 프레임 밖에 있는지 확인

            projected_points, _ = cv2.projectPoints(

                np.array([[cone.x, cone.y, 0]]), rvec, tvec, cameraMatrix, np.zeros(4)

            )

            px, py = projected_points[0][0]

  

            if px < 0 or px >= frame.shape[1] or py < 0 or py >= frame.shape[0]:

                gray_cones_center.append(cone.x)

                gray_cones_center.append(cone.y)

  
  

def listener():

    global pub_pointcloud, pub_pixel_coords, pub_cones_center, gray_cones_center, frame

  

    rospy.init_node('pointcloud_image_listener', anonymous=True)

    rospy.Subscriber('/pixel_coords', Int32MultiArray, pixel_coords_callback)

    rospy.Subscriber('/center_coords', Float32MultiArray, center_coords_callback)

    rospy.Subscriber('/object_info', ObjectInfo, callback_gray)  # 실제 LiDAR 메시지 타입으로 수정

  

    pub_pointcloud = rospy.Publisher('/processed_pointcloud', PointCloud2, queue_size=1)

    pub_cones_center = rospy.Publisher('/cones_center', Float32MultiArray, queue_size=1)

  

    gray_cones_center = []  # 회색 콘 좌표 리스트 초기화

  

    while not rospy.is_shutdown():

        pointcloud_msg = rospy.wait_for_message('/velodyne_points', PointCloud2)

        image_msg = rospy.wait_for_message("/usb_cam3/image_raw", Image)

        frame = bridge.imgmsg_to_cv2(image_msg, "bgr8")

  

        #pub_pointcloud.publish(pointcloud_msg)

  

        objPoints = np.array([[x, y, z] for x, y, z in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True) if x > 0])

        img_points, _ = cv2.projectPoints(

            objPoints, rvec, tvec, cameraMatrix,

            np.array([0, 0, 0, 0], dtype=float))

  

        result = model.predict(frame)[0]

        boxes = result.boxes.xyxy

        classes = [int(pt) for pt in result.boxes.cls]

        confs = [int(conf * 100) for conf in result.boxes.conf]

  

        # 모든 PointCloud 표시

        for i in range(len(img_points)):

            try:

                x, y = int(img_points[i][0][0]), int(img_points[i][0][1])

                if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]:  # 좌표가 이미지 범위 내에 있는지 확인

                    cv2.circle(frame, (x, y), 1, (255, 0, 255), 1)

            except Exception as e:

                rospy.logerr(f"Error drawing circle at img_points[{i}]: {img_points[i]}, Error: {e}")

  

        blue_cones_pixel = list()

        yellow_cones_pixel = list()

  

        for i, box in enumerate(boxes):

            box = [int(pt) for pt in box]

            start = (box[0], box[1])

            end = (box[2], box[3])

            center_x = (start[0] + end[0]) / 2

            center_y = (start[1] + end[1]) / 2

  

            cone_pixels = frame[int(0.4 * box[1] + 0.6 * box[3]):int(0.1 * box[1] + 0.9 * box[3]),

                                int(0.6 * box[0] + 0.4 * box[2]):int(0.4 * box[0] + 0.6 * box[2]), :]

            cone_pixels = cone_pixels.reshape(-1, 3)

            B_avg = sum(cone_pixels[:, 0]) / len(cone_pixels)

            G_avg = sum(cone_pixels[:, 1]) / len(cone_pixels)

            R_avg = sum(cone_pixels[:, 2]) / len(cone_pixels)

            blue_distance = (255 - B_avg) ** 2 + G_avg ** 2 + R_avg ** 2

            yellow_distance = B_avg ** 2 + (255 - G_avg) ** 2 + (255 - R_avg) ** 2

  

            if yellow_distance > blue_distance:  # Blue Cone

                blue_cones_pixel.append((center_x, center_y, start, end))

                color = (255, 0, 0)

                cv2.rectangle(frame, start, end, color=color, thickness=2)

                cv2.putText(frame, 'Blue', (box[0], box[3] + 15), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)

                cv2.putText(frame, '({}%)'.format(confs[i]), (box[0], box[3] + 40), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)

            else:  # Yellow Cone

                yellow_cones_pixel.append((center_x, center_y, start, end))

                color = (0, 255, 255)

                cv2.rectangle(frame, start, end, color=color, thickness=2)

                cv2.putText(frame, 'Yellow', (box[0], box[3] + 15), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)

                cv2.putText(frame, '({}%)'.format(confs[i]), (box[0], box[3] + 40), cv2.FONT_HERSHEY_DUPLEX, 0.7, color)

  

        global pixel_coords

  

        # 1. Pixel Coordinates 이용하여 빨간 원 그리기

        blue_cones_center = []  # blue 콘들을 저장하기 위해 리스트 초기화

        yellow_cones_center = []  # yellow 콘들을 저장하기 위해 리스트 초기화

  

        if pixel_coords is not None:

            pixel_coords_to_remove = []

  

            for i in range(0, len(pixel_coords), 2):

                x = pixel_coords[i]

                y = pixel_coords[i + 1]

                in_blue_box = False

                in_yellow_box = False

  

                for center_x, center_y, start, end in blue_cones_pixel:

                    if start[0] <= x <= end[0] and start[1] <= y <= end[1]:

                        in_blue_box = True

                        if center_coords is not None and i < len(center_coords) - 1:

                            pt = [center_coords[i], center_coords[i + 1], 0]

                            blue_cones_center.append(pt[0])

                            blue_cones_center.append(pt[1])

                        break

  

                for center_x, center_y, start, end in yellow_cones_pixel:

                    if start[0] <= x <= end[0] and start[1] <= y <= end[1]:

                        in_yellow_box = True

                        if center_coords is not None and i < len(center_coords) - 1:

                            pt = [center_coords[i], center_coords[i + 1], 0]

                            yellow_cones_center.append(pt[0])

                            yellow_cones_center.append(pt[1])

                        break

  

                if not in_blue_box and not in_yellow_box:

                    pixel_coords_to_remove.append(i)

                    pixel_coords_to_remove.append(i + 1)

                else:

                    cv2.circle(frame, (int(x), int(y)), 1, (0, 0, 255), 20)

  

            # Remove the pixel coordinates that are not inside any box

            pixel_coords = [coord for j, coord in enumerate(pixel_coords) if j not in pixel_coords_to_remove]

  

        # blue, yellow, gray 콘과의 수가 다를 경우 처리

        blue_cone_count = len(blue_cones_center) // 2

        yellow_cone_count = len(yellow_cones_center) // 2

        gray_cone_count = len(gray_cones_center) // 2

  

        max_cone_count = max(blue_cone_count, yellow_cone_count, gray_cone_count)

  

        # 콘을 동일한 수로 맞추기 위해 -100으로 채움

        for _ in range(blue_cone_count, max_cone_count):

            blue_cones_center.append(-100)

            blue_cones_center.append(-100)

  

        for _ in range(yellow_cone_count, max_cone_count):

            yellow_cones_center.append(-100)

            yellow_cones_center.append(-100)

  

        for _ in range(gray_cone_count, max_cone_count):

            gray_cones_center.append(-100)

            gray_cones_center.append(-100)

  

        print('Blue cones center:')

        for i in range(0, len(blue_cones_center), 2):

            print(f'  blue_x: {blue_cones_center[i]}, blue_y: {blue_cones_center[i + 1]}')

  

        print('Yellow cones center:')

        for i in range(0, len(yellow_cones_center), 2):

            print(f'  yellow_x: {yellow_cones_center[i]}, yellow_y: {yellow_cones_center[i + 1]}')

  

        print('Gray cones center:')

        for i in range(0, len(gray_cones_center), 2):

            print(f'  gray_x: {gray_cones_center[i]}, gray_y: {gray_cones_center[i + 1]}')

  

        # 세 리스트를 합쳐서 6개 요소의 리스트로 만듦

                # 세 리스트를 합쳐서 6개 요소의 리스트로 만듦
        cones_center = []
        for i in range(max_cone_count):
            cones_center.append([
                blue_cones_center[2 * i], blue_cones_center[2 * i + 1],
                yellow_cones_center[2 * i], yellow_cones_center[2 * i + 1],
                gray_cones_center[2 * i], gray_cones_center[2 * i + 1]
            ])

        # cones_center 리스트 출력
        print('Cones center:')
        for cone in cones_center:
            print(f'  Blue: ({cone[0]}, {cone[1]}), Yellow: ({cone[2]}, {cone[3]}), Gray: ({cone[4]}, {cone[5]})')

  

        # 중복된 LIDAR center point 제거

        def remove_duplicate_points(cones_pixel, cones_center):

            updated_cones_center = []

            for center_x, center_y, start, end in cones_pixel:

                box_center = np.array([center_x, center_y])

                points_in_box = [np.array([cones_center[j], cones_center[j + 1]]) for j in range(0, len(cones_center), 2) if start[0] <= cones_center[j] <= end[0] and start[1] <= cones_center[j + 1] <= end[1]]

  

                if points_in_box:

                    if len(points_in_box) > 1:

                        distances = [np.linalg.norm(box_center - pt) for pt in points_in_box]

                        closest_point = points_in_box[np.argmin(distances)]

                    else:

                        closest_point = points_in_box[0]

                    updated_cones_center.extend(closest_point)

  

            return updated_cones_center

  

        blue_cones_center = remove_duplicate_points(blue_cones_pixel, blue_cones_center)

        yellow_cones_center = remove_duplicate_points(yellow_cones_pixel, yellow_cones_center)

        gray_cones_center = remove_duplicate_points([], gray_cones_center)  # gray_cones_pixel 없으므로 빈 리스트 전달

  

        # Flatten the list for publishing

        flattened_cones_center = [val for sublist in cones_center for val in sublist]

  

        cones_center_msg = Float32MultiArray(data=np.array(flattened_cones_center).flatten())

        pub_cones_center.publish(cones_center_msg)

  

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):

            exit()

  
  

if __name__ == '__main__':

    model = YOLO('/home/stier/catkin_ws/ModelFiles/RubberCone_YOLOv10m_1280.pt')

    listener()
