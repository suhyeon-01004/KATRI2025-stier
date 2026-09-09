#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <ublox_msgs/NavPVT.h>
#include <ublox_msgs/NavSTATUS.h>
#include <erp42_msgs/SerialFeedBack.h>
#include <Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <cmath>

class FusionNode {
public:
    FusionNode() {
        nh_ = ros::NodeHandle("~");

        // 초기 상태 [x, y, heading]
        x_ = Eigen::Vector3d::Zero();
        P_ = Eigen::Matrix3d::Identity() * 1.0;

        utm_sub_ = nh_.subscribe("/utm", 1, &FusionNode::utmCallback, this);
        imu_sub_ = nh_.subscribe("/imu/data", 1, &FusionNode::imuCallback, this);
        speed_sub_ = nh_.subscribe("/erp42_serial/feedback", 1, &FusionNode::speedCallback, this);
        navpvt_sub_ = nh_.subscribe("/ublox_position_receiver/navpvt", 1, &FusionNode::navpvtCallback, this);
        status_sub_ = nh_.subscribe("/ublox_position_receiver/navstatus", 1, &FusionNode::statusCallback, this);

        utm_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/utm_final", 10);
        heading_pub_ = nh_.advertise<ublox_msgs::NavPVT>("/ublox_position_receiver/navpvt_final", 10);

        gps_available_ = false;
        last_time_ = ros::Time::now();
        heading_imu_ = 0.0;
        speed_ = 0.0;
    }

    void spin() {
        ros::Rate rate(50.0);
        while (ros::ok()) {
            ros::spinOnce();
            update();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber utm_sub_, imu_sub_, speed_sub_, navpvt_sub_, status_sub_;
    ros::Publisher utm_pub_, heading_pub_;

    Eigen::Vector3d x_;          // [x, y, heading]
    Eigen::Matrix3d P_;          // 공분산 행렬

    ros::Time last_time_;
    geometry_msgs::PoseStamped latest_utm_;
    bool gps_available_;
    double heading_imu_;
    double speed_;
    int navstatus_flags2_ = 0;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        tf::Quaternion q;
        tf::quaternionMsgToTF(msg->orientation, q);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        heading_imu_ = yaw;
    }

    void speedCallback(const erp42_msgs::SerialFeedBack::ConstPtr& msg) {
        speed_ = msg->speed / 36.0; // km/h → m/s
    }

    void navpvtCallback(const ublox_msgs::NavPVT::ConstPtr& msg) {
        // 원본 heading 그대로 보내는 용도
        if (!gps_available_) return;

        ublox_msgs::NavPVT msg_out = *msg;
        msg_out.heading = static_cast<int32_t>(x_(2) * 180.0 * 1e5 / M_PI); // rad → deg * 1e5
        heading_pub_.publish(msg_out);
    }

    void statusCallback(const ublox_msgs::NavSTATUS::ConstPtr& msg) {
        navstatus_flags2_ = msg->flags2;
        gps_available_ = (msg->flags2 >= 72);
    }

    void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        latest_utm_ = *msg;

        if (gps_available_) {
            // snap 방식으로 보정
            x_(0) = msg->pose.position.x;
            x_(1) = msg->pose.position.y;
            // heading은 IMU 기준 유지 (GPS heading은 불확실성 있음)
            x_(2) = heading_imu_;

            // 공분산 리셋
            P_ = Eigen::Matrix3d::Identity() * 0.5;
        }
    }

    void update() {
        ros::Time now = ros::Time::now();
        double dt = (now - last_time_).toSec();
        last_time_ = now;
        if (dt <= 0.0 || dt > 1.0) return;

        // prediction
        Eigen::Vector3d u;
        u << speed_, 0.0, 0.0;

        double theta = x_(2);
        Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 3, 1> B;
        B << cos(theta) * dt, sin(theta) * dt, 0.0;

        x_ = x_ + B * speed_;
        x_(2) = heading_imu_;  // heading은 항상 IMU 기준 사용

        Eigen::Matrix3d Q = Eigen::Matrix3d::Identity() * 0.05;
        P_ = A * P_ * A.transpose() + Q;

        // publish
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = now;
        pose.header.frame_id = "gps";
        pose.pose.position.x = x_(0);
        pose.pose.position.y = x_(1);
        pose.pose.position.z = 0.0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(x_(2));
        utm_pub_.publish(pose);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fusion_node");
    FusionNode node;
    node.spin();
    return 0;
}
