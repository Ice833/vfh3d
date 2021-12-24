#include <ros/ros.h>
#include <vfh_rover/OctomapProcessing.h>
#include <vfh_rover/Vehicle.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include<string>

using namespace octomap;
using std::string;

string topicOcto, topicGoal, topicPose;

int main(int argc, char ** argv) {
  ros::init(argc, argv, "drone_node");
  ros::NodeHandle nh;

    float x, y, z, h, w, d, safety_radius; // meters
    float maxIncline, minIncline; // radians
    bool prevSet;

    nh.getParam("/init_x", x); nh.getParam("/init_y", y); nh.getParam("/init_z", z);
    nh.getParam("/height", h); nh.getParam("/width", w); nh.getParam("/depth", d);
    nh.getParam("/safety_radius", safety_radius); nh.getParam("/maxIncline", maxIncline); nh.getParam("/minIncline", minIncline);
    nh.getParam("/prevSet", prevSet);

    nh.getParam("/topic_octomap", topicOcto); nh.getParam("/topic_goal", topicGoal); nh.getParam("/topic_pose", topicPose);

    //   Vehicle v(-5,0,1, 1,1,1, 0.4, M_PI/8,-M_PI/8, false);
    Vehicle v(x,y,z, h,w,d, safety_radius, maxIncline, minIncline, prevSet);
    OctomapProcessing op (0.09, v, 5, nh); // alpha = 0.09, max_range = 5;
    ros::Subscriber sub1 = nh.subscribe(topicOcto, 3, &OctomapProcessing::octomapCallback, &op);
    ros::Subscriber sub2 = nh.subscribe(topicGoal, 3, &OctomapProcessing::goalCallback, &op);
    ros::Subscriber sub3 = nh.subscribe(topicPose, 3, &OctomapProcessing::poseCallback, &op);
    ros::spin();
    return 0;
}
