#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>

std::string save_file_path;

ros::Subscriber pc_sub;

#define debug(x) std::cout << #x << " : " << x << std::endl
#define DEBUG
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pc_sub.shutdown();

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);

    debug(cloud.width);
    debug(cloud.height);

#ifdef DEBUG
    pcl::visualization::CloudViewer viewer("viewer");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));
    viewer.showCloud(cloud_ptr);
    while (!viewer.wasStopped());
#endif

    debug(cloud.points.size());
#ifdef DEBUG
    for(int i = 0; i < cloud.points.size();i++){
        debug(cloud.points[i].x);
        debug(cloud.points[i].y);
        debug(cloud.points[i].z);
    }
#endif

    if(save_file_path != std::string("")){
        ROS_INFO("save pc data to : %s", save_file_path.c_str());
        try {
            pcl::io::savePCDFile(save_file_path, cloud);
        }catch (std::exception& e){
            ROS_INFO("%s",e.what());
        }
    }


}

//main method, entrance to ROS node
int main(int argc, char **argv) {
    //Initialize ROS, name node track_tag
    ros::init(argc, argv, "pc_view");
    //create the NodeHandle
    ros::NodeHandle nh("~");

    nh.param("save_file_path",save_file_path,std::string(""));

    pc_sub = nh.subscribe("/camera1/points", 1, cloud_cb);


    //call callbacks until ROS shutdown
    try {
        ros::spin();
    } catch (...) {
        ROS_ERROR("--- ERROR IN spin(), shutting down! ---");
        ros::shutdown();
    }
    return 0;
}
