#ifndef GPS_IMU_FUSION_H
#define GPS_IMU_FUSION_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ublox_msgs/NavPVT.h>
#include <ublox_msgs/NavSTATUS.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include "utm_lla_converter.h"   // ULConverter 포함
#include <nav_msgs/Path.h>

class GPSIMUFusion {
public:
    GPSIMUFusion();

private:
    // ROS 통신
    ros::Subscriber sub_imu_, sub_gps_, sub_status_;
    ros::Publisher pub_odom_;
    tf::TransformBroadcaster tf_broadcaster_;

    // 상태 벡터 및 공분산
    Eigen::VectorXd state_; // [x, y, vx, vy, yaw]
    Eigen::MatrixXd P_;

    // 상태 플래그
    bool gps_ready_;
    bool use_gps_;
    double ref_lat_, ref_lon_;
    int fixStat_;
    int flags2_;
    ros::Time last_time_;

    // UTM 변환기
    ULConverter converter_;

    ros::Publisher path_pub_;
    nav_msgs::Path path_msg_;

    // 콜백 함수
    void statusCallback(const ublox_msgs::NavSTATUS::ConstPtr& msg);
    void gpsCallback(const ublox_msgs::NavPVT::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

    // EKF 함수
    void predict(double ax, double ay, double yaw, double dt);
    void update(const Eigen::Vector2d &z);

    // 출력
    void publishOdom(const ros::Time& stamp);

    // 위도/경도 → UTM
    void latLonToUTM(double lat, double lon, double &x, double &y);
};

#endif // GPS_IMU_FUSION_H
