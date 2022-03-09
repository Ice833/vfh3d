#include <vfh_rover/OctomapProcessing.h>
#include <vfh_rover/VFHistogram.h>
#include <vfh_rover/PathParams.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <unistd.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

bool HAS_PROC = false; // temp var, only process once. For debugging
static int COUNT = 0;

OctomapProcessing::OctomapProcessing(float alpha, Vehicle &v,
    float maxRange, ros::NodeHandle n) :
  vehicle(v), maxRange(maxRange), gotGoal(false), gotOcto(false), alpha(alpha)
{
  histogram_pub = n.advertise<sensor_msgs::PointCloud2>("/histogram", 2);
  pose_pub = n.advertise<geometry_msgs::PoseArray>("/open_poses", 2);
  next_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/next_direction", 2);
  path_pub = n.advertise<nav_msgs::Path>("/vehicle_path",10);
  begin = ros::Time::now();
}

void OctomapProcessing::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  vehicle.setPose(msg->pose);
}


void OctomapProcessing::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  goal = msg->pose;
  gotGoal = true;
  if (not HAS_PROC && gotOcto) {
    process();
  }

}

void OctomapProcessing::octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
  octomap::AbstractOcTree* atree = octomap_msgs::msgToMap(*msg);
  if(atree->size() !=0){
    tree = boost::make_shared<octomap::OcTree>(*(octomap::OcTree *)atree);
    // std::cout << "The size of the current map is: "<< tree->size() << std::endl;
  }  
  gotOcto = true;
  if (not HAS_PROC && gotGoal) {
    process();
  }
  delete(atree);
}

/*void OctomapProcessing::simulate() {
  HAS_PROC = true;
  for (float x=-5; x<15; x+=0.5) {
    //vehicle.x = x;
    process();
    usleep(1000*1000*1000*1000*1000);
  }
}*/

void OctomapProcessing::process() {
  // Build
  VFHistogram h (tree, vehicle, maxRange, alpha);
  //std::cout << h.displayString() << std::endl;
  h.binarize(1);

  COUNT++;
  /*if(COUNT == 20)
    exit(0);
  // Disp string
  // Disp histogram point cloud
 */
  RGBPointCloud::Ptr pc = h.displayCloud(1);
  sensor_msgs::PointCloud2 pc_msg;
  pcl::toROSMsg(*pc, pc_msg);
  pc_msg.header.frame_id = "map";
  histogram_pub.publish(pc_msg);

  // Disp found free positions
  geometry_msgs::PoseArray pa;
  //pa.poses = h.findPaths(5, 4);
  pa.poses = h.findPaths(int(ceil((vehicle.safety_radius+vehicle.w)/alpha)),
                         int(ceil((vehicle.safety_radius+vehicle.h)/alpha)));
  pa.header.frame_id = "map";
  pose_pub.publish(pa);

  // Disp next position
  geometry_msgs::PoseStamped p;
  PathParams params;
  params.goalWeight = 10;
  params.prevWeight = 5;
  params.headingWeight = 0;
  params.elevationWeight = 7;
  // params.goal_radius = 5;
  params.goal_radius = 0.5;
  geometry_msgs::Pose* next_pose = h.optimalPath(vehicle, goal, params, &pa.poses);
  if (next_pose == NULL)
    return;
  p.pose = *next_pose;
  vehicle.prevHeading = next_pose;
  vehicle.prevSet = true;
  p.header.stamp = ros::Time::now();
  p.header.frame_id = "map";
  next_pose_pub.publish(p);

  //Disp path
  path.header.stamp = p.header.stamp;
  path.header.frame_id = "map";
  path.poses.push_back(p);
  path_pub.publish(path);
  //delete(&h);
}
