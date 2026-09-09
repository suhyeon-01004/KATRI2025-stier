// include ROS parts
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "object_detector/objectDetectorConfig.h"
#include <sensor_msgs/PointCloud2.h>
// include PCL parts
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include "../include/object_detector/dbscan.h"
// include visualization parts
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// include STL parts
#include <vector>
// include MSG
#include <object_detector/ObjectInfo.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include <cmath>
#include <std_msgs/Int32.h>
using namespace std;
// pcl point type
typedef pcl::PointXYZ PointT;
// cluster point type
typedef pcl::PointXYZI clusterPointT;

// ROI parameter
double zMin, zMax, xMin, xMax, yMin, yMax;
// DBScan parameter
int minPoints;
double epsilon, minClusterSize, maxClusterSize;
// VoxelGrid parameter
float leafSize;

int section = 0;

// publisher
ros::Publisher pubROI;
ros::Publisher pubCluster;
ros::Publisher pubCluster_lane;
ros::Publisher pubObjectInfo;
ros::Publisher pubLane;
ros::Publisher pubPixelCoords;
ros::Publisher pubCenterCoords;
//MSG
object_detector::ObjectInfo objectMsg = object_detector::ObjectInfo();
object_detector::ObjectInfo laneMsg = object_detector::ObjectInfo();

void cfgCallback(object_detector::objectDetectorConfig &config, int32_t level) {
    xMin = config.xMin;
    xMax = config.xMax;
    yMin = config.yMin;
    yMax = config.yMax;
    zMin = config.zMin;
    zMax = config.zMax;

    minPoints = config.minPoints;
    epsilon = config.epsilon;
    minClusterSize = config.minClusterSize;
    maxClusterSize = config.maxClusterSize;

    leafSize  = config.leafSize;
}

void sectionCallback(const std_msgs::Int32::ConstPtr& sec_msgs) {
    section = sec_msgs->data;
    ROS_INFO("Received section: %d", section);

    // section 값에 따라 ROI 파라미터 업데이트
    if (section == 1000) {
        xMin = 0.0; xMax = 20.0;
        yMin = -5.0; yMax = 5.0;
        zMin = -1.0; zMax = 2.0;
        minPoints = 10;
        epsilon = 0.44;
        minClusterSize = 10;
        maxClusterSize = 5000;
    }
    //유턴
    else if (section == 8) {
        xMin = 0.0; xMax = 10.0;
        yMin = -5.0; yMax = 5.0;
        zMin = -0.4; zMax = 1.0;
        minPoints = 5;
        epsilon = 0.44;
        minClusterSize = 10;
        maxClusterSize = 2000;
    } 
    // 터널
    else if (section == 9) {
        xMin = 0.0; xMax = 15.0;
        yMin = -4.0; yMax = 4.0;
        zMin = -0.4; zMax = 1.0;
        minPoints = 10;
        epsilon = 0.44;
        minClusterSize = 10;
        maxClusterSize = 5000;
    } 
    // 톨게이트 
    else if (section == 10) {
        xMin = 0.0; xMax = 15.0;
        yMin = -4.0; yMax = 4.0;
        zMin = -0.3; zMax = 1.0;
        minPoints = 10;
        epsilon = 0.44;
        minClusterSize = 10;
        maxClusterSize = 2000;
    }
    // 소형
    else if (section == 11) {
        xMin = 0.0; xMax = 15.0;
        yMin = -4.0; yMax = 4.0;
        zMin = -0.3; zMax = 1.0;
        minPoints = 5;
        epsilon = 0.44;
        minClusterSize = 5;
        maxClusterSize = 1000;
    } 
    // 배달
    else if (section == 20 || section == 21) { 
        xMin = 0.0; xMax = 15.0;
        yMin = -4.0; yMax = 4.0;
        zMin = -0.2; zMax = 5.0;
        minPoints = 5;
        epsilon = 0.44;
        minClusterSize = 5;
        maxClusterSize = 3000;
    }
    // 주차
    else if (section == 15) {
        xMin = 0.0; xMax = 15.0;
        yMin = -4.0; yMax = 4.0;
        zMin = -0.4; zMax = 1.0;
        minPoints = 5;
        epsilon = 0.44;
        minClusterSize = 5;
        maxClusterSize = 1000;
    }
    //사선주차
    else if (section == 3) {
        zMin = -0.4; zMax = 1.2;
        xMin = 0.0;  xMax = 3.0;   // 여유 3 m (부채꼴에서 다시 r≤2로 제한)
        yMin = -3.5; yMax = 3.5;   // ±3.5 m (부채꼴에서 다시 |θ|≤57°로 제한)
        minPoints      = 5;        // 핵심 포인트 수
        epsilon        = 0.44;     // 포인트 간 거리 임계(0.35~0.50 튜닝 권장)
        minClusterSize = 5;        // 최소 클러스터 크기
        maxClusterSize = 2000;     // 환경에 맞게 상한
}
    //Defalut
    else {
        xMin = 0.0; xMax = 8.0;
        yMin = -5.0; yMax = 5.0;
        zMin = -0.4; zMax = 1.0;
        minPoints = 10;
        epsilon = 0.44;
        minClusterSize = 10;
        maxClusterSize = 1000;
        cout << "Defalut ROI" << endl;
    }
}

pcl::PointCloud<PointT>::Ptr ROI (const sensor_msgs::PointCloud2ConstPtr& input) {
    // ... do data processing
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    pcl::fromROSMsg(*input, *cloud); // sensor_msgs -> PointCloud 형변환

    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    // std::cout << "Loaded : " << cloud->width * cloud->height << '\n';

    // 오브젝트 생성 
    // Z축 ROI
    pcl::PassThrough<PointT> filter;
    filter.setInputCloud(cloud);                //입력 
    filter.setFilterFieldName("z");             //적용할 좌표 축 (eg. Z축)
    filter.setFilterLimits(zMin, zMax);          //적용할 값 (최소, 최대 값)
    filter.filter(*cloud_filtered);             //필터 적용 

    // X축 ROI
    filter.setInputCloud(cloud_filtered);                //입력 
    filter.setFilterFieldName("x");             //적용할 좌표 축 (eg. X축)
    filter.setFilterLimits(xMin, xMax);          //적용할 값 (최소, 최대 값)
    filter.filter(*cloud_filtered);             //필터 적용 

    // Y축 ROI
    filter.setInputCloud(cloud_filtered);                //입력 
    filter.setFilterFieldName("y");             //적용할 좌표 축 (eg. Y축)
    filter.setFilterLimits(yMin, yMax);          //적용할 값 (최소, 최대 값)
    filter.filter(*cloud_filtered);             //필터 적용 

    // 포인트수 출력
//    std::cout << "ROI Filtered :" << cloud_filtered->width * cloud_filtered->height  << '\n'; 

    sensor_msgs::PointCloud2 roi_raw;
    pcl::toROSMsg(*cloud_filtered, roi_raw);
    
    pubROI.publish(roi_raw);

    return cloud_filtered;
}

pcl::PointCloud<PointT>::Ptr ROI_LANE (const sensor_msgs::PointCloud2ConstPtr& input) {
    // ... do data processing
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    pcl::fromROSMsg(*input, *cloud); // sensor_msgs -> PointCloud 형변환

    //std::cout << "Original lane points count: " << cloud->points.size() << std::endl;

    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    //std::cout << "Lane Loaded : " << cloud->width * cloud->height << '\n';

    // 오브젝트 생성 
    // Z축 ROI
    pcl::PassThrough<PointT> filter;
    filter.setInputCloud(cloud);                //입력 
    filter.setFilterFieldName("z");             //적용할 좌표 축 (eg. Z축)
    filter.setFilterLimits(-1, 1);          //적용할 값 (최소, 최대 값)
    filter.filter(*cloud_filtered);             //필터 적용 

    // X축 ROI
    filter.setInputCloud(cloud_filtered);                //입력 
    filter.setFilterFieldName("x");             //적용할 좌표 축 (eg. X축)
    filter.setFilterLimits(0, 10);          //적용할 값 (최소, 최대 값)
    filter.filter(*cloud_filtered);             //필터 적용 

    // Y축 ROI
    filter.setInputCloud(cloud_filtered);                //입력 
    filter.setFilterFieldName("y");             //적용할 좌표 축 (eg. Y축)
    filter.setFilterLimits(-4, 4);          //적용할 값 (최소, 최대 값)
    filter.filter(*cloud_filtered);             //필터 적용 

    sensor_msgs::PointCloud2 roi_raw;
    pcl::toROSMsg(*cloud_filtered, roi_raw);
    
    pubROI.publish(roi_raw);

    return cloud_filtered;
}

pcl::PointCloud<PointT>::Ptr voxelGrid(pcl::PointCloud<PointT>::Ptr input) {
    //Voxel Grid를 이용한 DownSampling
    pcl::VoxelGrid<PointT> vg;    // VoxelGrid 선언
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>); //Filtering 된 Data를 담을 PointCloud 선언
    vg.setInputCloud(input);             // Raw Data 입력
    vg.setLeafSize(leafSize, leafSize, leafSize); // 사이즈를 너무 작게 하면 샘플링 에러 발생
    vg.filter(*cloud_filtered);          // Filtering 된 Data를 cloud PointCloud에 삽입
    
    return cloud_filtered;
}

void cluster(pcl::PointCloud<PointT>::Ptr input) {
    if (input->empty())
        return;

    //KD-Tree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::PointCloud<clusterPointT>::Ptr clusterPtr(new pcl::PointCloud<clusterPointT>);
    tree->setInputCloud(input);

    //Segmentation
    std::vector<pcl::PointIndices> cluster_indices;

    //DBSCAN with Kdtree for accelerating
    DBSCANKdtreeCluster<PointT> dc;
    dc.setCorePointMinPts(minPoints);   //Set minimum number of neighbor points
    dc.setClusterTolerance(epsilon);    //Set Epsilon 
    dc.setMinClusterSize(minClusterSize);
    dc.setMaxClusterSize(maxClusterSize);
    dc.setSearchMethod(tree);
    dc.setInputCloud(input);
    dc.extract(cluster_indices);

    pcl::PointCloud<clusterPointT> totalcloud_clustered;
    int cluster_id = 0;
    std_msgs::Int32MultiArray pixel_coords;
    std_msgs::Float32MultiArray center_coords;

    // 카메라, 라이다 캘리브레이션 매트릭스
    Eigen::Matrix3f rvec;
    rvec <<  8.21233417e-04, -9.99991248e-01,  4.10230056e-03,
            -2.14963416e-01, -4.18293240e-03, -9.76613144e-01,
             9.76621756e-01, -7.98171942e-05, -2.14964970e-01;
    Eigen::Vector3f tvec( -0.01055664, 0.60616676, 1.38181219);

    Eigen::Matrix3f cameraMatrix;
    cameraMatrix << 940.25656824,   0.,         639.25133894,
                    0.,         934.4284002, 401.49278998,
                    0.,           0.,           1.           ;

    //각 Cluster 접근
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++, cluster_id++) {

        pcl::PointCloud<clusterPointT> eachcloud_clustered;
        float cluster_counts = cluster_indices.size();
        
        float max_x = -std::numeric_limits<float>::max();
        float max_y = 0.0;

        float min_x = std::numeric_limits<float>::max();
        float min_y = 0.0;

        //각 Cluster내 각 Point 접근
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {

            clusterPointT tmp;
            tmp.x = input->points[*pit].x; 
            tmp.y = input->points[*pit].y;
            tmp.z = input->points[*pit].z;
            tmp.intensity = cluster_id % 100; // 상수 : 예상 가능한 cluster 총 개수
            eachcloud_clustered.push_back(tmp);
            totalcloud_clustered.push_back(tmp);

            if (tmp.x > max_x) {
                max_x = tmp.x;
                max_y = tmp.y;
            }
            if (tmp.x < min_x) {
                min_x = tmp.x;
                min_y = tmp.y;
            }
        }

        //minPoint와 maxPoint 받아오기
        clusterPointT minPoint, maxPoint;
        pcl::getMinMax3D(eachcloud_clustered, minPoint, maxPoint);

        if (abs(minPoint.y - min_y) < 1) {
            min_y = minPoint.y+0.6;
        }

        if (abs(maxPoint.y - max_y) < 1) {
            max_y = maxPoint.y-0.6;
        }

        objectMsg.lengthX[cluster_id] = maxPoint.x-minPoint.x; // 
        objectMsg.lengthY[cluster_id] = maxPoint.y-minPoint.y; 
        objectMsg.lengthZ[cluster_id] = maxPoint.z-minPoint.z; // 
        objectMsg.centerX[cluster_id] = (minPoint.x + maxPoint.x)/2; //직육면체 중심 x 좌표
        objectMsg.centerY[cluster_id] = (minPoint.y + maxPoint.y)/2; //직육면체 중심 y 좌표
        objectMsg.centerZ[cluster_id] = (minPoint.z + maxPoint.z)/2; //직육면체 중심 z 좌표
        objectMsg.minX[cluster_id] = min_x;  
        objectMsg.minY[cluster_id] = min_y; 
        objectMsg.minZ[cluster_id] = minPoint.z;  
        objectMsg.maxX[cluster_id] = max_x; 
        objectMsg.maxY[cluster_id] = max_y; 
        objectMsg.maxZ[cluster_id] = maxPoint.z;

        Eigen::Vector3f obj_center(objectMsg.centerX[cluster_id], objectMsg.centerY[cluster_id], objectMsg.centerZ[cluster_id]);
        Eigen::Vector3f img_point = cameraMatrix * (rvec * obj_center + tvec);
        img_point /= img_point.z();

        objectMsg.pixelX[cluster_id] = static_cast<int>(img_point.x());
        objectMsg.pixelY[cluster_id] = static_cast<int>(img_point.y());
    
        // pixel_coords.data.push_back(static_cast<int>(img_point.x()));
        // pixel_coords.data.push_back(static_cast<int>(img_point.y()));

        // center_coords.data.push_back(objectMsg.centerX[cluster_id]);  // Add center X
        // center_coords.data.push_back(objectMsg.centerY[cluster_id]);  // Add center Y
        
      	cout << "Object Cluster " << cluster_id << " - centerX: " << objectMsg.centerX[cluster_id] << ", centerY: " << objectMsg.centerY[cluster_id] << ", centerZ: " << objectMsg.centerZ[cluster_id] << "\n";
        //cout << "Pixel coordinates: [" << static_cast<int>(img_point.x()) << ", " << static_cast<int>(img_point.y()) << "]\n";
      	//cout << cluster_indices.size() << "\n";
    }
    
    objectMsg.objectCounts = cluster_id;
    pubObjectInfo.publish(objectMsg);

    //새로 추가
    // pubPixelCoords.publish(pixel_coords);
    // pubCenterCoords.publish(center_coords);  // Publish center coordinates


    sensor_msgs::PointCloud2 cluster_point;
    pcl::toROSMsg(totalcloud_clustered, cluster_point);
    cluster_point.header.frame_id = "velodyne";
    pubCluster.publish(cluster_point);
}

void cluster_lane(pcl::PointCloud<PointT>::Ptr input) {
    if (input->empty())
        return;

    //KD-Tree
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::PointCloud<clusterPointT>::Ptr clusterPtr(new pcl::PointCloud<clusterPointT>);
    tree->setInputCloud(input);

    //Segmentation
    std::vector<pcl::PointIndices> cluster_indices;

    //DBSCAN with Kdtree for accelerating
    DBSCANKdtreeCluster<PointT> dc;

    dc.setCorePointMinPts(0);   //Set minimum number of neighbor points //감소 : 클러스터 개수 증가, 기본값 5
    dc.setClusterTolerance(10);    //Set Epsilon //10으로 하면 차선 싹 하나의 cluster로 봄

    // // 본선
    if(section==3) dc.setMinClusterSize(5); // cluster 형성하기 위한 최소 점의 수, 감소 : 더 작고 많은 클러스터도 형성
    else if(section==7) dc.setMinClusterSize(70);
    else dc.setMinClusterSize(0);

    // 예선
    // dc.setMinClusterSize(5);

    dc.setMaxClusterSize(maxClusterSize);
    dc.setSearchMethod(tree);
    dc.setInputCloud(input);
    dc.extract(cluster_indices);

    pcl::PointCloud<clusterPointT> totalcloud_clustered;
    int cluster_id = 0;

    //각 Cluster 접근
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++, cluster_id++) {

        pcl::PointCloud<clusterPointT> eachcloud_clustered;
        float cluster_counts = cluster_indices.size();
        
        float max_x = -std::numeric_limits<float>::max();
        float max_y = 0.0;

        float min_x = std::numeric_limits<float>::max();
        float min_y = 0.0;

        //각 Cluster내 각 Point 접근
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {

            clusterPointT tmp;
            tmp.x = input->points[*pit].x; 
            tmp.y = input->points[*pit].y;
            tmp.z = input->points[*pit].z;
            tmp.intensity = cluster_id % 100; // 상수 : 예상 가능한 cluster 총 개수
            eachcloud_clustered.push_back(tmp);
            totalcloud_clustered.push_back(tmp);

            if (tmp.x > max_x) {
                max_x = tmp.x;
                max_y = tmp.y;
            }
            if (tmp.x < min_x) {
                min_x = tmp.x;
                min_y = tmp.y;
            }
        }

        //minPoint와 maxPoint 받아오기
        clusterPointT minPoint, maxPoint;
        pcl::getMinMax3D(eachcloud_clustered, minPoint, maxPoint);

        if (abs(minPoint.y - min_y) < 1) {
            min_y = minPoint.y+0.6;
        }

        if (abs(maxPoint.y - max_y) < 1) {
            max_y = maxPoint.y-0.6;
        }

        laneMsg.lengthX[cluster_id] = maxPoint.x-minPoint.x; // 
        laneMsg.lengthY[cluster_id] = maxPoint.y-minPoint.y; 
        laneMsg.lengthZ[cluster_id] = maxPoint.z-minPoint.z; // 
        laneMsg.centerX[cluster_id] = (minPoint.x + maxPoint.x)/2; //직육면체 중심 x 좌표
        laneMsg.centerY[cluster_id] = (minPoint.y + maxPoint.y)/2; //직육면체 중심 y 좌표
        laneMsg.centerZ[cluster_id] = (minPoint.z + maxPoint.z)/2; //직육면체 중심 z 좌표
        laneMsg.minX[cluster_id] = min_x;  
        laneMsg.minY[cluster_id] = min_y; 
        laneMsg.minZ[cluster_id] = minPoint.z;  
        laneMsg.maxX[cluster_id] = max_x; 
        laneMsg.maxY[cluster_id] = max_y; 
        laneMsg.maxZ[cluster_id] = maxPoint.z;
    }
    
    laneMsg.objectCounts = cluster_id;
    pubLane.publish(laneMsg);

    sensor_msgs::PointCloud2 cluster_lane;
    pcl::toROSMsg(totalcloud_clustered, cluster_lane);
    cluster_lane.header.frame_id = "velodyne";
    pubCluster_lane.publish(cluster_lane);
}

void mainCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    pcl::PointCloud<PointT>::Ptr cloudPtr;

    // main process method
    cloudPtr = ROI(input);
    // cloudPtr = voxelGrid(cloudPtr);
    cluster(cloudPtr);

    objectMsg = object_detector::ObjectInfo();
}

void laneCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
    pcl::PointCloud<PointT>::Ptr lanePtr;

    lanePtr = ROI_LANE(input);

    cluster_lane(lanePtr);

    laneMsg = object_detector::ObjectInfo();
}

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init (argc, argv, "object_detector");
    ros::NodeHandle nh;
    
    dynamic_reconfigure::Server<object_detector::objectDetectorConfig> server;
    dynamic_reconfigure::Server<object_detector::objectDetectorConfig>::CallbackType f;

    f = boost::bind(&cfgCallback, _1, _2);
    server.setCallback(f);
    
    // Create ROS subscribers
    ros::Subscriber sub = nh.subscribe("velodyne_points", 1, mainCallback);
    ros::Subscriber lane_sub = nh.subscribe ("/lane_output", 1, laneCallback);
    ros::Subscriber section_sub = nh.subscribe("/section", 1, sectionCallback); // section 토픽 구독

    // Create ROS publishers
    pubROI = nh.advertise<sensor_msgs::PointCloud2>("roi_raw", 1);
    pubCluster = nh.advertise<sensor_msgs::PointCloud2>("cluster", 1);
    pubCluster_lane = nh.advertise<sensor_msgs::PointCloud2>("cluster_lane", 1);
    pubObjectInfo = nh.advertise<object_detector::ObjectInfo>("object_info", 1);
    pubLane = nh.advertise<object_detector::ObjectInfo>("lane_info", 1);
    // pubPixelCoords = nh.advertise<std_msgs::Int32MultiArray>("pixel_coords", 1);
    // pubCenterCoords = nh.advertise<std_msgs::Float32MultiArray>("center_coords", 1);

    // Spin
    ros::spin();
}