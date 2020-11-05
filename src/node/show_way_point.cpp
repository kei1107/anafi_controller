////////////////////////////////////////////////////////////////////////////////
/// @file           show_way_point.cpp
/// @brief          Outputs waypoints as a TF.
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
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int16.h>
#include <fstream>


const double pi = acos(-1.0);   //!< Definition of PI

int target_num; //!< way points num
int now_target; //!< Output waypoint number
std::string wpFrame; //!< Origin Frame name
std::string prefix_; //!< TF prefix
bool all_pub_flag; //!< Flag on TF output method
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
    target_num = wpnum;
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

/**
 * @brief Callback function to change the output waypoint number.
 * @param msg target number
 */
void tn_cb(const std_msgs::Int16ConstPtr& msg){
    ROS_INFO("Receive target : %d",msg->data);
    if(msg->data >= 0 && msg->data < target_num) now_target = msg->data;
}
int main(int argc, char **argv) {
    //Initialize ROS, name node track_tag
    ros::init(argc, argv, "show_way_point");
    //create the NodeHandle
    ros::NodeHandle nh("~");

    ros::Timer timer_pub_wp;
    ros::Subscriber int_sub;

    // Read parameters
    now_target = 0;
    nh.param("all_pub",all_pub_flag,true);
    nh.param("prefix",prefix_,std::string("way_point"));
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
    int_sub = nh.subscribe<std_msgs::Int16>("target_num",10,tn_cb);

    ros::Rate rate(10);
    tf::TransformBroadcaster tf_broadcaster;
    while(ros::ok()){
        if(all_pub_flag) {
            for (now_target = 0; now_target < target_num; now_target++) {
                tf::Quaternion tQ;
                if (wp_info[0].size() <= 3) {
                    tQ = tf::createQuaternionFromRPY(0, 0, 0);
                } else {
                    tQ = tf::createQuaternionFromRPY(wp_info[now_target][3], wp_info[now_target][4],
                                                     wp_info[now_target][5]);
                }
                tf::Transform t(tQ, tf::Vector3(wp_info[now_target][0], wp_info[now_target][1], wp_info[now_target][2]));
                std::string TFName = prefix_ + std::to_string(now_target + 1);
                ros::Time nowt = ros::Time::now();
                tf::StampedTransform wpTF(t, nowt, wpFrame, TFName);
                tf_broadcaster.sendTransform(wpTF);
            }
        }else{
            tf::Quaternion tQ;
            if (wp_info[0].size() <= 3) {
                tQ = tf::createQuaternionFromRPY(0, 0, 0);
            } else {
                tQ = tf::createQuaternionFromRPY(wp_info[now_target][3], wp_info[now_target][4],
                                                 wp_info[now_target][5]);
            }
            tf::Transform t(tQ, tf::Vector3(wp_info[now_target][0], wp_info[now_target][1], wp_info[now_target][2]));
            std::string TFName = prefix_ + std::to_string(now_target + 1);
            ros::Time nowt = ros::Time::now();
            // ROS_INFO("%s => %s",wpFrame.c_str(),TFName.c_str());
            tf::StampedTransform wpTF(t, nowt, wpFrame, TFName);
            tf_broadcaster.sendTransform(wpTF);
        }


        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}