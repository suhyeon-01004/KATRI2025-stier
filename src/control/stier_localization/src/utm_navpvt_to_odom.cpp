#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ublox_msgs/NavPVT.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <mutex>

class UTMNavPVTToOdom
{
public:
  UTMNavPVTToOdom()
  {
    ros::NodeHandle nh, pnh("~");

    // params
    pnh.param<std::string>("world_frame", world_frame_, std::string("utm"));     // header.frame_id
    pnh.param<std::string>("child_frame", child_frame_, std::string("base_link"));// child_frame_id
    pnh.param<double>("min_speed", min_speed_, 0.5);          // [m/s] 미만이면 yaw 신뢰 낮음
    pnh.param<double>("max_yaw_var", max_yaw_var_, 1e6);      // 신뢰 불가시 yaw 분산
    pnh.param<double>("min_yaw_var", min_yaw_var_, 1e-4);     // 너무 작은 분산 방지
    pnh.param<double>("pose_var_xy", pose_var_xy_, 0.0);      // 위치 분산(원하면 설정), 0이면 미지정
    pnh.param<double>("pose_var_z", pose_var_z_, 0.0);        // z 분산
    pnh.param<bool>("zero_altitude", zero_altitude_, false);  // z=0으로 고정할지

    // subscribers (토픽 이름은 remap로 맞추면 됨: /utm, /navpvt)
    sub_utm_ = nh.subscribe("utm", 50, &UTMNavPVTToOdom::utmCallback, this);
    sub_navpvt_ = nh.subscribe("navpvt", 100, &UTMNavPVTToOdom::navpvtCallback, this);

    // publisher
    pub_odom_ = nh.advertise<nav_msgs::Odometry>("odometry/gps", 50);

    ROS_INFO("[utm_navpvt_to_odom] world_frame=%s child_frame=%s min_speed=%.3f",
             world_frame_.c_str(), child_frame_.c_str(), min_speed_);
  }

private:
  // 최신 NavPVT 상태
  struct HeadingState {
    ros::Time stamp;
    double yaw_enu = 0.0;      // ENU yaw [rad]
    double speed = 0.0;        // [m/s]
    double var_yaw = 1e6;      // yaw 분산
    bool valid = false;
  };

  void navpvtCallback(const ublox_msgs::NavPVT::ConstPtr& msg)
  {
    HeadingState hs;

    // stamp: navpvt 메시지에는 header가 없으니 수신 시각 사용
    hs.stamp = ros::Time::now();

    // 속도: gSpeed 단위 mm/s -> m/s
    hs.speed = msg->gSpeed * 1e-3;

    if (hs.speed >= min_speed_)
    {
      // heading 단위: 1e-5 deg -> rad
      const double heading_deg = msg->heading * 1e-5;
      const double heading_rad = heading_deg * M_PI / 180.0;

      // ROS ENU yaw: 동=0, CCW+
      hs.yaw_enu = M_PI / 2.0 - heading_rad;

      // headAcc: 1e-5 deg -> rad 표준편차
      const double sigma_yaw = (msg->headAcc * 1e-5) * M_PI / 180.0;
      hs.var_yaw = std::max(sigma_yaw * sigma_yaw, min_yaw_var_);

      hs.valid = true;
    }
    else
    {
      // 저속/정지: yaw 신뢰 불가
      hs.yaw_enu = 0.0;
      hs.var_yaw = max_yaw_var_;
      hs.valid = false; // 유효 yaw로 취급하지 않음
    }

    {
      std::lock_guard<std::mutex> lk(mutex_);
      last_heading_ = hs;
    }
  }

  void utmCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    // 최신 heading 상태를 읽어서 Odom 작성
    HeadingState hs;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      hs = last_heading_;
    }

    nav_msgs::Odometry odom;
    odom.header.stamp = msg->header.stamp; // 위치 시각을 준수
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = child_frame_;

    // 위치: 입력 UTM 좌표 그대로 사용
    odom.pose.pose.position.x = msg->pose.position.x;
    odom.pose.pose.position.y = msg->pose.position.y;
    odom.pose.pose.position.z = zero_altitude_ ? 0.0 : msg->pose.position.z;

    // 자세(orientation): NavPVT.heading 기반 yaw 사용
    if (hs.valid)
    {
      odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(hs.yaw_enu);
      // pose covariance: xy/z는 파라미터 기반(옵션), yaw는 heading 분산
      fillPoseCovariance(odom, hs.var_yaw);
    }
    else
    {
      // yaw 신뢰 없으면 단위 쿼터니언 + 큰 yaw 분산
      odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
      fillPoseCovariance(odom, max_yaw_var_);
    }

    // twist (선택): NavPVT 속도에서 ENU 성분 추정
    if (hs.valid)
    {
      const double vx_e = hs.speed * std::cos(hs.yaw_enu); // East
      const double vy_n = hs.speed * std::sin(hs.yaw_enu); // North
      odom.twist.twist.linear.x = vx_e;
      odom.twist.twist.linear.y = vy_n;
      odom.twist.twist.linear.z = 0.0;

      // 보수적으로 작은 분산을 줄 수도 있지만, 여기서는 0으로 둠(원하면 파라미터화)
    }
    else
    {
      odom.twist.twist.linear.x = 0.0;
      odom.twist.twist.linear.y = 0.0;
      odom.twist.twist.linear.z = 0.0;
    }

    pub_odom_.publish(odom);
  }

  void fillPoseCovariance(nav_msgs::Odometry& odom, double yaw_var)
  {
    // nav_msgs/Odometry.pose.covariance는 6x6(행우선). 여기서는 간단히 diag만.
    // [x y z roll pitch yaw] 순서로 (x=0, y=7, z=14, roll=21, pitch=28, yaw=35)
    for (int i = 0; i < 36; ++i) odom.pose.covariance[i] = 0.0;

    odom.pose.covariance[0]  = pose_var_xy_; // var(x)
    odom.pose.covariance[7]  = pose_var_xy_; // var(y)
    odom.pose.covariance[14] = pose_var_z_;  // var(z)
    odom.pose.covariance[21] = 1e6;          // var(roll) 큰 값
    odom.pose.covariance[28] = 1e6;          // var(pitch) 큰 값
    odom.pose.covariance[35] = yaw_var;      // var(yaw)
  }

private:
  // params
  std::string world_frame_;
  std::string child_frame_;
  double min_speed_;
  double max_yaw_var_;
  double min_yaw_var_;
  double pose_var_xy_;
  double pose_var_z_;
  bool zero_altitude_;

  // state
  ros::Subscriber sub_utm_;
  ros::Subscriber sub_navpvt_;
  ros::Publisher  pub_odom_;
  std::mutex mutex_;
  HeadingState last_heading_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "utm_navpvt_to_odom");
  UTMNavPVTToOdom node;
  ros::spin();
  return 0;
}
