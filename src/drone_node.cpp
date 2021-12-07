#include <ros/ros.h>
#include <vfh_rover/OctomapProcessing.h>
#include <vfh_rover/Vehicle.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

using namespace octomap;

int main(int argc, char ** argv) {
  ros::init(argc, argv, "vfh_rover_node");
  ros::NodeHandle nh;


  Vehicle v(-5,0,1, 1,1,1, 0.4, M_PI/8,-M_PI/8, false);
  OctomapProcessing op (0.09, v, 5, nh);
  ros::Subscriber sub1 = nh.subscribe("/octomap_full", 3, &OctomapProcessing::octomapCallback, &op);
  ros::Subscriber sub2 = nh.subscribe("/goal", 3, &OctomapProcessing::goalCallback, &op);
  ros::Subscriber sub3 = nh.subscribe("/vehiclePose", 3, &OctomapProcessing::poseCallback, &op);
  ros::spin();
  return 0;
}
