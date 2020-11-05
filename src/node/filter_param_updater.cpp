////////////////////////////////////////////////////////////////////////////////
/// @file           filter_param_updater.cpp
/// @brief          specific_color_to_multi_point_cloud.cpp's filter parameter updater
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
#include <std_msgs/UInt8MultiArray.h>
#include <fstream>

/**
 * @brief Read mask rules
 * @param mask_file_path mask file path
 * @return Update Format
 */
std_msgs::UInt8MultiArray read_mask_rules(const std::string &mask_file_path) {
    std_msgs::UInt8MultiArray ret;

    std::ifstream ifs(mask_file_path.c_str());
    if (!ifs) {
        ROS_INFO("Don't open file : %s", mask_file_path.c_str());
        return ret;
    }
    int CNUM;
    ifs >> CNUM;

    ret.data.emplace_back(CNUM);
    ROS_INFO(" ===================================== ");
    int CLASS, H_MIN, S_MIN, V_MIN, H_MAX, S_MAX, V_MAX;
    while (ifs >> CLASS >> H_MIN >> S_MIN >> V_MIN >> H_MAX >> S_MAX >> V_MAX) {
        ROS_INFO("CLASS %d read HSV mask rule (H_MIN,S_MIN,V_MIN) (H_MAX,S_MAX,V_MAX) : (%d,%d,%d) (%d,%d,%d)",
                     CLASS, H_MIN, S_MIN, V_MIN, H_MAX, S_MAX, V_MAX);
        ret.data.emplace_back(CLASS);
        ret.data.emplace_back(H_MIN);
        ret.data.emplace_back(S_MIN);
        ret.data.emplace_back(V_MIN);
        ret.data.emplace_back(H_MAX);
        ret.data.emplace_back(S_MAX);
        ret.data.emplace_back(V_MAX);
    }
    ROS_INFO(" ===================================== ");
    return ret;
}
int main(int argc, char **argv) {
    //Initialize ROS, name node track_tag
    ros::init(argc, argv, "filter_param_updater");
    //create the NodeHandle
    ros::NodeHandle nh("~");

    std::string mask_file_path;
    nh.param("mask_file_path", mask_file_path, std::string(""));

    // Read mask rules
    std_msgs::UInt8MultiArray msg;
    if (mask_file_path == std::string("")) {
        ROS_WARN("Parameter not set : mask_file_path");
    } else {
        msg = read_mask_rules(mask_file_path);
    }

    if(msg.data.empty()) return 0;

    ros::Publisher filter_param_pub = nh.advertise<std_msgs::UInt8MultiArray>("/filter_param",1);
    while(!filter_param_pub.getNumSubscribers()){
        sleep(1);
    }
    ROS_INFO("Publish filter param");
    ROS_INFO("publish to %s",filter_param_pub.getTopic().c_str());
    filter_param_pub.publish(msg);
    return 0;
}