#include <ros/ros.h>
#include "control_pose_bridge/pose_arbiter.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_arbiter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  PoseArbiter node(nh, pnh);
  ros::spin();
  return 0;
}
