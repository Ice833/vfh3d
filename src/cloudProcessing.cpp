#include "include/vfh_rover/cloudProcessing.h"



struct voxel_hist{
    float azimuth;
    float elevation;
    float dist;
    bool isReached;
    voxel_hist(float a, float e):azimuth(a), elevation(e),dist(0.f){}
};

using PolarHist = voxel_hist*;
class local_planner
{
private:

    ros::NodeHandle nh;

    ros::Subscriber subOdomAftMapped, subCropLaserCloudMap;
    ros::Publisher pubPathAvoidence, pubOctoGrid;

    OctreePointCloudSearch<PointType>::Ptr octomap{new OctreePointCloudSearch<PointType>(RESOLUTION)};
    OctreePointCloudOccupancy<PointType>::Ptr occupancy{new OctreePointCloudOccupancy<PointType>(RESOLUTION)};

    std::vector<PolarHist> hist;

    pcl::PointCloud<PointType>::Ptr laserCloudCrop{new pcl::PointCloud<PointType>};

    double vcs_x = 0.0f, vcs_y = 0.0f, vcs_z = 0.0f;

public:

    local_planner(){
        //Read raw pointcloud map
        subOdomAftMapped = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 100, &local_planner::odomAftMappedHandler, this);
        subCropLaserCloudMap = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_map_bounded", 100, &local_planner::laserCloudCropHandler, this);
    }
    local_planner(double leaf_size_){
        local_planner();
        octomap->setResolution(leaf_size_);
    }
    ~local_planner() = default;

    void laserCloudCropHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCropRes){
        pcl::fromROSMsg(*laserCloudCropRes, *laserCloudCrop);
    }

    void odomAftMappedHandler(const nav_msgs::OdometryConstPtr &odomAftMappedRes){
        vcs_x = odomAftMappedRes->pose.pose.position.x;
        vcs_y = odomAftMappedRes->pose.pose.position.y;
        vcs_z = odomAftMappedRes->pose.pose.position.z;
    }

    //Save input pcd into octree.
    void set_cloud_octomap(const pcl::PointCloud<PointType>::Ptr& cloud_in)
    {
        octomap->setInputCloud(cloud_in);
        octomap->addPointsFromInputCloud();
    }

    void set_vcs(const Eigen::Vector3f& pos){
        vcs_x = pos[0]; vcs_y = pos[1]; vcs_z = pos[2];
    }
    //Update pointcloud in octomap.
    void update_cloud_octomap(const pcl::PointCloud<PointType>::Ptr& update_cloud)
    {
        set_cloud_octomap(update_cloud);
    }

    //Return a result pointcloud ptr. 
    pcl::PointCloud<PointType>::Ptr octreeRadiusSearch(float search_radius){

        std::vector<int>pointIdxRadiusSearch;
        std::vector<float>pointRadiusSquaredDist;

        PointType searchPoint;
        pcl::PointCloud<PointType> cloud_box = *octomap->getInputCloud();
        pcl::PointCloud<PointType> cloud_sphere;

        if(octomap->radiusSearch(searchPoint, (double)search_radius,pointIdxRadiusSearch, pointRadiusSquaredDist) > 0){
            for(std::size_t i = 0; i < pointIdxRadiusSearch.size(); i++){
                cloud_sphere.push_back(cloud_box[i]);
            }
            std::cout << "[local_planner]: Radius search completed!" << std::endl;
        }

        return cloud_sphere.makeShared();
    }

    void printVCS(){
        ROS_INFO("[local planner]: vcs_x(%.2f m), vcs_y(%.2f m), vcs_z(%.2f m)", vcs_x, vcs_y, vcs_z);
    }

    float calculateAzimuth(float x_i, float y_i){
        return floor(atan(x_i-vcs_x/y_i-vcs_y)/HIST_RESOLUTION);
    }

    float calculateElevation(float x_i, float y_i, float z_i){
        return floor(atan(z_i-vcs_z/calNorm2(x_i-vcs_x, y_i-vcs_y))/HIST_RESOLUTION);
    }

    float calculateDist(float x_i, float y_i, float z_i){
        return floor(calNorm2(x_i-vcs_x, y_i-vcs_y, z_i-vcs_z));
    }
    
    void buildOccupancy(){
        occupancy->setInputCloud(octomap->getInputCloud());
    }

    void buildPolarHistgram(){
        //Note: Read VCS from topic:aftmapped_odom.
        pcl::PointCloud<PointType> temp = *octomap->getInputCloud();
        int cloudSize  = temp.size();

        for(int i = 0; i < cloudSize; i++){
            PolarHist i_hist = new voxel_hist(calculateAzimuth(temp[i].x, temp[i].y), calculateElevation(temp[i].x, temp[i].y,temp[i].z));
            i_hist->dist = calculateDist(temp[i].x, temp[i].y,temp[i].z);
             hist.push_back(i_hist);
        }

    }

    void addAuxiliaryInfo(){

    }

    void processPointcloud(const pcl::PointCloud<PointType>::Ptr& laser_map_bounded){
        set_cloud_octomap(laser_map_bounded);
        update_cloud_octomap(octreeRadiusSearch(BOX_LENGTH/2));
    }

    void process(){

        //Stage.1: Get octomap from raw pointcloud.
        processPointcloud(laserCloudCrop);

        //Stage.2: Build 2D histogram.
        buildPolarHistgram();
        buildOccupancy();

        //Stage.3: Add new characteristics.
        addAuxiliaryInfo();

        //Stage.4: 2D Binary Histogram


        //Stage.5:Path searching and selection.
        

        
    }



};

//Debug:
int main(int argc, char** argv){
    ros::init(argc, argv, "localPlanner");
    local_planner test;
    ros::Rate rate(100);

    ROS_INFO("[localPlanner]: Starting local planner.");

    // while(ros::ok()){
    //     test.process();
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    return 0;
}