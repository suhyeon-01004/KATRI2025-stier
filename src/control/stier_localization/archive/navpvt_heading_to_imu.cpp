#include <ros/ros.h>
#include <ublox_msgs/NavPVT.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <cmath>

class NavPVTHeadingToImu
{
public:
  NavPVTHeadingToImu()
  {
    ros::NodeHandle nh, pnh("~");

    pnh.param<std::string>("frame_id", frame_id_, std::string("gps"));
    pnh.param<double>("min_speed", min_speed_, 0.5); // m/s 이하일 때 yaw 신뢰도 낮음
    pnh.param<double>("max_yaw_var", max_yaw_var_, 1e6); // 신뢰 불가일 때 covariance

    sub_ = nh.subscribe("navpvt", 10, &NavPVTHeadingToImu::callback, this);
    pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 10); // navsat에서 바로 구독하도록 이름 지정
  }

private:
  void callback(const ublox_msgs::NavPVT::ConstPtr& msg)
  {
    sensor_msgs::Imu imu;
    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = frame_id_;

    // gSpeed 단위: mm/s → m/s
    double speed = msg->gSpeed * 1e-3;

    if (speed >= min_speed_)
    {
      // heading 단위: 1e-5 deg
      double heading_deg = msg->heading * 1e-5;
      double heading_rad = heading_deg * M_PI / 180.0;

      // ROS ENU 기준 yaw 변환: 동=0 rad, CCW+
      double yaw_enu = M_PI/2.0 - heading_rad;

      geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(yaw_enu);
      imu.orientation = q;

      // headAcc 단위: 1e-5 deg → rad 표준편차
      double sigma_yaw = (msg->headAcc * 1e-5) * M_PI / 180.0;
      double var_yaw = sigma_yaw * sigma_yaw;

      // roll, pitch는 모르므로 큰 분산, yaw는 headAcc 기반
      imu.orientation_covariance[0] = max_yaw_var_;
      imu.orientation_covariance[4] = max_yaw_var_;
      imu.orientation_covariance[8] = std::max(var_yaw, 1e-4); // 최소 바닥값 줌
    }
    else
    {
      // 저속/정지: yaw 신뢰 불가
      imu.orientation_covariance[0] = max_yaw_var_;
      imu.orientation_covariance[4] = max_yaw_var_;
      imu.orientation_covariance[8] = max_yaw_var_;
    }

    pub_.publish(imu);
  }

  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string frame_id_;
  double min_speed_;
  double max_yaw_var_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navpvt_heading_to_imu");
  NavPVTHeadingToImu node;
  ros::spin();
  return 0;
}
