#include "control_pose_bridge/pose_arbiter.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <cmath>

static double normAng(double a){ while(a>M_PI) a-=2*M_PI; while(a<-M_PI) a+=2*M_PI; return a; }

double PoseArbiter::yawFromQuat(const geometry_msgs::Quaternion& qmsg){
  tf2::Quaternion q; tf2::fromMsg(qmsg, q);
  double r,p,y; tf2::Matrix3x3(q).getRPY(r,p,y);
  return y;
}
geometry_msgs::Quaternion PoseArbiter::quatFromYaw(double yaw){
  tf2::Quaternion q; q.setRPY(0,0,yaw);
  return tf2::toMsg(q);
}
double PoseArbiter::wrap(double a){ return normAng(a); }

PoseArbiter::PoseArbiter(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  double gps_to = pnh.param("gps_timeout", 0.5);
  double dr_to  = pnh.param("dr_timeout",  0.5);
  blend_duration_ = pnh.param("blend_duration", 0.5);

  gps_timeout_ = ros::Duration(gps_to);
  dr_timeout_  = ros::Duration(dr_to);

  sub_gps_pose_ = nh.subscribe("/gps_utm_pose", 10, &PoseArbiter::gpsPoseCb, this);
  sub_dr_odom_  = nh.subscribe("/odom_dr",     50, &PoseArbiter::drOdomCb, this);
  sub_gps_ok_   = nh.subscribe("/gps_ok",      10, &PoseArbiter::gpsOkCb, this);

  pub_out_pose_ = nh.advertise<geometry_msgs::PoseStamped>("/pose_for_pp", 10);
  pub_source_   = nh.advertise<std_msgs::String>("/pose_source", 10, true);

  timer_ = nh.createTimer(ros::Duration(1.0/50.0), &PoseArbiter::onTimer, this);
}

void PoseArbiter::gpsPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  gps_pose_.x = msg->pose.position.x;
  gps_pose_.y = msg->pose.position.y;
  gps_pose_.yaw = yawFromQuat(msg->pose.orientation);
  gps_pose_.stamp = msg->header.stamp;
  gps_pose_.valid = true;
  last_gps_stamp_ = msg->header.stamp;
}

void PoseArbiter::drOdomCb(const nav_msgs::Odometry::ConstPtr& msg){
  dr_pose_.x = msg->pose.pose.position.x;
  dr_pose_.y = msg->pose.pose.position.y;
  dr_pose_.yaw = yawFromQuat(msg->pose.pose.orientation);
  dr_pose_.stamp = msg->header.stamp;
  dr_pose_.valid = true;
  last_dr_stamp_ = msg->header.stamp;
}

void PoseArbiter::gpsOkCb(const std_msgs::Bool::ConstPtr& msg){
  bool new_ok = msg->data;
  if(new_ok == gps_ok_) return;
  gps_ok_ = new_ok;
  if(gps_ok_) enterGPS();
  else        enterDR();
}

void PoseArbiter::enterDR(){
  in_dr_ = true;
  blending_ = false;
  computeDROffset();
  std_msgs::String s; s.data = "DR"; pub_source_.publish(s);
}

void PoseArbiter::enterGPS(){
  in_dr_ = false;
  blending_ = true;
  blend_start_ = ros::Time::now();
  std_msgs::String s; s.data = "GPS"; pub_source_.publish(s);
}

void PoseArbiter::computeDROffset(){
  Pose2D ref = gps_pose_.valid ? gps_pose_ : last_out_;
  Pose2D cur = dr_pose_;
  dyaw_ = wrap(ref.yaw - cur.yaw);
  double c = cos(dyaw_), s = sin(dyaw_);
  double xr = c*cur.x - s*cur.y;
  double yr = s*cur.x + c*cur.y;
  dx_ = ref.x - xr;
  dy_ = ref.y - yr;
}

Pose2D PoseArbiter::applyDROffset(const Pose2D& d) const{
  Pose2D o = d;
  double c = cos(dyaw_), s = sin(dyaw_);
  double xr = c*d.x - s*d.y;
  double yr = s*d.x + c*d.y;
  o.x = xr + dx_;
  o.y = yr + dy_;
  o.yaw = wrap(d.yaw + dyaw_);
  o.valid = d.valid;
  o.stamp = d.stamp;
  return o;
}

Pose2D PoseArbiter::blend(const Pose2D& a, const Pose2D& b, double alpha) const{
  Pose2D o;
  o.x = (1-alpha)*a.x + alpha*b.x;
  o.y = (1-alpha)*a.y + alpha*b.y;
  double dy = wrap(b.yaw - a.yaw);
  o.yaw = wrap(a.yaw + alpha*dy);
  o.stamp = ros::Time::now();
  o.valid = a.valid && b.valid;
  return o;
}

void PoseArbiter::onTimer(const ros::TimerEvent&){
  const ros::Time now = ros::Time::now();
  if(gps_pose_.valid && (now - last_gps_stamp_) > gps_timeout_) gps_pose_.valid = false;
  if(dr_pose_.valid  && (now - last_dr_stamp_)  > dr_timeout_)  dr_pose_.valid  = false;

  Pose2D out;
  if(gps_ok_ && gps_pose_.valid){
    if(blending_ && dr_pose_.valid){
      double t = (now - blend_start_).toSec();
      double a = std::min(1.0, t / blend_duration_);
      Pose2D dr_al = applyDROffset(dr_pose_);
      out = blend(dr_al, gps_pose_, a);
      if(a >= 1.0) blending_ = false;
    }else{
      out = gps_pose_;
    }
  }else{
    if(dr_pose_.valid) out = applyDROffset(dr_pose_);
    else { out = last_out_; out.stamp = now; }
  }

  geometry_msgs::PoseStamped msg;
  msg.header.stamp = out.stamp.isZero() ? now : out.stamp;
  msg.header.frame_id = "utm";
  msg.pose.position.x = out.x;
  msg.pose.position.y = out.y;
  msg.pose.position.z = 0.0;
  msg.pose.orientation = quatFromYaw(out.yaw);
  pub_out_pose_.publish(msg);
  last_out_ = out;
  // ... onTimer 내부 마지막 부분 직전에 추가
  bool has_any = gps_pose_.valid || dr_pose_.valid || last_out_.valid;
  if(!has_any){
    return; // 아무 것도 유효하지 않으면 퍼블리시하지 않음 (RViz NaN 방지)
  }
}
