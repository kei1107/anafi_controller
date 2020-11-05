////////////////////////////////////////////////////////////////////////////////
/// @file           way_point_to_pc.cpp
/// @brief          Convert way points to point cloud
/// @author         Keisuke KIMURA
/// @date           2020/06/01
/// $Version:       1.0.0
/// @note           Prerequisite for use with ROS node.
/// @par            History
///                 2020/06/01 : Support for doxygen
///
/// Copyright (c) 2020 Keisuke KIMURA. All Rights reserved.
/// This software is released under the MIT License.
/// http://opensource.org/licenses/mit-license.php
///
////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int16.h>
#include <fstream>
#include <pcl_ros/point_cloud.h>

const double pi = acos(-1.0);   //!< Definition of PI

std::string wpFrame; //!< Origin Frame name
std::vector<std::vector<double>> wp_info; //!< way point information

/**
 * @brief convert degree to radian
 * @param deg degree
 * @return radian
 */
double degree2radian(double deg){
    return deg * pi / 180.0;
}
/**
 * @brief Read way point information
 * @param way_point_file Way point file path
 * @return read success[true] or failure[false]
 */
bool read_way_point(const std::string &way_point_file) {
    std::ifstream ifs(way_point_file.c_str());
    if (!ifs) {
        ROS_FATAL("Don't open file : %s", way_point_file.c_str());
        return false;
    }
    int wpnum,infonum;
    ifs >> wpnum >> infonum;
    wp_info.assign(wpnum,std::vector<double>(infonum,0.0));
    for(int i = 0; i < wpnum; i++){
        for(int j = 0; j < infonum; j++) {
            ifs >> wp_info[i][j];
        }
        if(infonum >= 6){
            std::swap(wp_info[i][4],wp_info[i][5]);
            wp_info[i][4] =  -1.0*wp_info[i][4] + pi/2.0;
        }
        for(int j = 3; j < infonum; j++){
            // wp_info[i][j] = degree2radian(wp_info[i][j]);
        }
    }
    ROS_INFO("Read %d way point",wpnum);
    return true;
}

int main(int argc, char **argv) {
    //Initialize ROS, name node track_tag
    ros::init(argc, argv, "way_point_to_pc");
    //create the NodeHandle
    ros::NodeHandle nh("~");

    // Read parameters
    nh.param("wpFrame",wpFrame,std::string("wpFrame"));
    std::string way_point_file;
    nh.param("way_point_file", way_point_file, std::string(""));
    // Read mask rules
    if (way_point_file == std::string("")) {
        ROS_FATAL("Parameter not set : way_point_file\nStop program.");
        return 0;
    } else {
        bool ok = read_way_point(way_point_file);
        if(!ok){
            ROS_FATAL("Stop program.");
            return 0;
        }
    }

    ros::Publisher pc_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("wp_pc",10);

    ros::Rate rate(10);
    while(ros::ok()){
        pcl::PointCloud<pcl::PointXYZRGB> cloud;

        for(auto wp:wp_info){
            pcl::PointXYZRGB ps;
            ps.x = wp[0];
            ps.y = wp[1];
            ps.z = wp[2];

            ps.r = 255;
            ps.g = 0;
            ps.b = 0;

            cloud.points.emplace_back(ps);
        }
        auto msg = cloud.makeShared();
        msg->header.frame_id = wpFrame;
        pcl_conversions::toPCL(ros::Time::now(),msg->header.stamp);

        ROS_INFO("publish");
        pc_pub.publish(msg);
        ROS_INFO("published");

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}