#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

struct Pose2D {
  double x{0}, y{0}, yaw{0};
  ros::Time stamp;
  bool valid{false};
};

class PoseArbiter {
public:
  PoseArbiter(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:
  void gpsPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void drOdomCb(const nav_msgs::Odometry::ConstPtr& msg);
  void gpsOkCb(const std_msgs::Bool::ConstPtr& msg);
  void onTimer(const ros::TimerEvent&);

  static double yawFromQuat(const geometry_msgs::Quaternion& q);
  static geometry_msgs::Quaternion quatFromYaw(double yaw);
  static double wrap(double a);

  void enterDR();
  void enterGPS();
  void computeDROffset();
  Pose2D applyDROffset(const Pose2D& dr_in) const;
  Pose2D blend(const Pose2D& a, const Pose2D& b, double alpha) const;

  // state
  bool gps_ok_{false};
  bool in_dr_{false};
  bool blending_{false};
  ros::Time blend_start_;
  double blend_duration_{0.5};
  double dx_{0}, dy_{0}, dyaw_{0};

  Pose2D gps_pose_, dr_pose_, last_out_;
  ros::Duration gps_timeout_, dr_timeout_;
  ros::Time last_gps_stamp_, last_dr_stamp_;

  ros::Subscriber sub_gps_pose_, sub_dr_odom_, sub_gps_ok_;
  ros::Publisher  pub_out_pose_, pub_source_;
  ros::Timer timer_;
};
