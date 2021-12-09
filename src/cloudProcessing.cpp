#include "include/vfh_rover/cloudProcessing.h"

void laserCloudCropHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCropRes)
{
    pcl::fromROSMsg(*laserCloudCropRes, *laserCloudCrop);
}

//Save input pcd into octree.
void cloudProcess(const pcl::PointCloud<PointType>::Ptr& cloud_in)
{
    for(int i = 0; i < cloud_in->size(); i++){
        point3d point(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z);
        tree.updateNode(point, true);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "cloudProcessNode");

    ROS_INFO("[cloudProcessNode]: Starting cloud process.");

    subCropLaserCloudMap = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_map_bounded", 100, laserCloudCropHandler);

    pubOcto = nh.advertise<octomap_msgs::Octomap>("octomapCropped", 100);
    
    ros::spin();
    return 0;
}