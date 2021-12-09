#pragma once
#define PCL_NO_PRECOMPILE

#include <cstddef>
#include<cmath>
#include <cmath>
#include <mutex>
#include <queue>
#include <vector>
#include <deque>

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pcl/impl/point_types.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <eigen3/Eigen/Dense>

#include <octomap/octomap.h>
#include <octomap/math/Vector3.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

using PointType = pcl::PointXYZI;
using namespace octomap;

//Unit in m.
constexpr double RESOLUTION = 0.5;

ros::NodeHandle nh;

ros::Subscriber subCropLaserCloudMap;
ros::Publisher pubOcto;

OcTree tree (RESOLUTION);  // create empty tree with resolution 0.5

pcl::PointCloud<PointType>::Ptr laserCloudCrop{new pcl::PointCloud<PointType>};