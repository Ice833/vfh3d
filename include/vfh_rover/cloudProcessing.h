#include <cstddef>
#include<cmath>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud_occupancy.h>

using namespace pcl::octree;

//Unit in m.
constexpr double RESOLUTION = 0.5;
constexpr double HIST_RESOLUTION = 2.0;
constexpr double DRONE_SIZE = 2.0;
constexpr double SAFETY_DIST = 5.0;
constexpr double BOX_LENGTH = 5.0;