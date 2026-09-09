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

#include<cmath>

#include<std_msgs/Int32.h>
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

int section=0;

// publisher
ros::Publisher pubROI;
ros::Publisher pubCluster;
ros::Publisher pubCluster_lane;
ros::Publisher pubObjectInfo;
ros::Publisher pubLane;

//MSG
object_detector::ObjectInfo objectMsg=object_detector::ObjectInfo();
object_detector::ObjectInfo laneMsg=object_detector::ObjectInfo();

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
    //filter.setFilterLimitsNegative (true);     //적용할 값 외 
    filter.filter(*cloud_filtered);             //필터 적용 

    // X축 ROI
    // pcl::PassThrough<PointT> filter;
    filter.setInputCloud(cloud_filtered);                //입력 
    filter.setFilterFieldName("x");             //적용할 좌표 축 (eg. X축)
    filter.setFilterLimits(xMin, xMax);          //적용할 값 (최소, 최대 값)
    //filter.setFilterLimitsNegative (true);     //적용할 값 외 
    filter.filter(*cloud_filtered);             //필터 적용 

    // Y축 ROI
    // pcl::PassThrough<PointT> filter;
    filter.setInputCloud(cloud_filtered);                //입력 
    filter.setFilterFieldName("y");             //적용할 좌표 축 (eg. Y축)
    filter.setFilterLimits(yMin, yMax);          //적용할 값 (최소, 최대 값)
    //filter.setFilterLimitsNegative (true);     //적용할 값 외 
    filter.filter(*cloud_filtered);             //필터 적용 

    // 포인트수 출력
    std::cout << "ROI Filtered :" << cloud_filtered->width * cloud_filtered->height  << '\n'; 

    sensor_msgs::PointCloud2 roi_raw;
    pcl::toROSMsg(*cloud_filtered, roi_raw);
    
    pubROI.publish(roi_raw);

    return cloud_filtered;
}

pcl::PointCloud<PointT>::Ptr ROI_LANE (const sensor_msgs::PointCloud2ConstPtr& input) {
    // ... do data processing
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    pcl::fromROSMsg(*input, *cloud); // sensor_msgs -> PointCloud 형변환

    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    std::cout << "Lane Loaded : " << cloud->width * cloud->height << '\n';

    // 오브젝트 생성 
    // // Z축 ROI
    // pcl::PassThrough<PointT> filter;
    // filter.setInputCloud(cloud);                //입력 
    // filter.setFilterFieldName("z");             //적용할 좌표 축 (eg. Z축)
    // filter.setFilterLimits(zMin, zMax);          //적용할 값 (최소, 최대 값)
    // //filter.setFilterLimitsNegative (true);     //적용할 값 외 
    // filter.filter(*cloud_filtered);             //필터 적용 

    // X축 ROI
    pcl::PassThrough<PointT> filter;
    filter.setInputCloud(cloud_filtered);                //입력 
    filter.setFilterFieldName("x");             //적용할 좌표 축 (eg. X축)
    filter.setFilterLimits(-2.0, -0.5);          //적용할 값 (최소, 최대 값)
    //filter.setFilterLimitsNegative (true);     //적용할 값 외 
    filter.filter(*cloud_filtered);             //필터 적용 

    // // Y축 ROI
    // // pcl::PassThrough<PointT> filter;
    // filter.setInputCloud(cloud_filtered);                //입력 
    // filter.setFilterFieldName("y");             //적용할 좌표 축 (eg. Y축)
    // filter.setFilterLimits(yMin, yMax);          //적용할 값 (최소, 최대 값)
    // //filter.setFilterLimitsNegative (true);     //적용할 값 외 
    // filter.filter(*cloud_filtered);             //필터 적용 

    // 포인트수 출력
    std::cout << "ROI Lane Filtered :" << cloud_filtered->width * cloud_filtered->height  << '\n'; 

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
    
    std::cout << "After Voxel Filtered :" << cloud_filtered->width * cloud_filtered->height  << '\n'; 

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

    }
    
    objectMsg.objectCounts = cluster_id;
    pubObjectInfo.publish(objectMsg);

    sensor_msgs::PointCloud2 cluster_point;
    pcl::toROSMsg(totalcloud_clustered, cluster_point);
    cluster_point.header.frame_id = "velodyne";
    pubCluster.publish(cluster_point);
}

void sectionCallback(const std_msgs::Int32::ConstPtr& sec_msgs) {
    section = sec_msgs->data;
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

    dc.setCorePointMinPts(5);   //Set minimum number of neighbor points
    dc.setClusterTolerance(0.4);    //Set Epsilon 

    // // 본선
    if(section==3) dc.setMinClusterSize(5);
    else if(section==7) dc.setMinClusterSize(70);
    else dc.setMinClusterSize(5);

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

    lanePtr = ROI(input);

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
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("velodyne_points", 1, mainCallback);
    ros::Subscriber lane_sub = nh.subscribe ("/lane_output", 1, laneCallback);

    // Create a ROS publisher for the output point cloud
    pubROI = nh.advertise<sensor_msgs::PointCloud2> ("roi_raw", 1);
    pubCluster = nh.advertise<sensor_msgs::PointCloud2>("cluster", 1);
    pubCluster_lane = nh.advertise<sensor_msgs::PointCloud2>("cluster_lane", 1);
    pubObjectInfo = nh.advertise<object_detector::ObjectInfo>("object_info", 1);
    pubLane = nh.advertise<object_detector::ObjectInfo>("lane_info",1);

    // Spin
    ros::spin();
}
