#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <geometry_msgs/Pose.h>
#include <stdio.h>
#include <queue>
#include <cmath>



/*
 * x-y-z : camera coordinate
 *
 * tello-base
 *
 * x -> right(+)/left(-)
 * y -> down(+)/up(-)
 * z -> forward(+)/backward(-)
 *
 *
 * */

//vector3 struct for ease of organization
typedef struct vec {
    vec(double X = 0, double Y = 0, double Z = -2.0) {
        x = X;
        y = Y;
        z = Z;
    }

    double x, y, z;
} vec;


class Tello {
public:
    double MAX_SPEED;
    double DEADZONE;

    tf::TransformListener listener_;
    std::string base_frame_id;
    std::string frame_id;
    double lastupdate;
    bool update;

    // cmd_vel publisher
    ros::Publisher cmd_vel;
    // ar_pose subscriber
    ros::Subscriber ar_pose;

    // cmd_vel value
    geometry_msgs::Twist move;
    geometry_msgs::PoseStamped marker_pose;

    ar_track_alvar_msgs::AlvarMarker marker;

    vec offset;

    Tello() : frame_id(""), lastupdate(0), update(false) {
        MAX_SPEED = 0.25;
        DEADZONE = MAX_SPEED/2.0;
    }

    void set_offset(vec val){
        offset = val;
    }

    void set_base_frame_id(const std::string &base_frame_name) {
        // TODO
    }

    void set_frame_id(const std::string &frame_name) {
        // TODO
    }

    bool set_transform() {
        try {
            listener_.waitForTransform(base_frame_id, frame_id, ros::Time(), ros::Duration(1.0));
            tf::StampedTransform transform;
            listener_.lookupTransform(base_frame_id, frame_id, ros::Time(), transform);

            return true;
        } catch (tf::TransformException &ex) {
            std::cout << "Failure at " << ros::Time::now() << std::endl;
            std::cout << "Exception thrown:" << ex.what() << std::endl;
            std::cout << "The current list of frames is:" << std::endl;
            std::cout << listener_.allFramesAsString() << std::endl;
            return false;
        }
    }

    void poseCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg) {
        if (msg->markers.empty() || msg->markers.at(0).id != 0) {
            //ROS_INFO("%s : NO_MESSAGE!",frame_id.c_str());
            update = false;
        } else {
            marker = msg->markers.at(0);
            update = true;
            sendCmdVel();
            /*
            try {
                marker.pose.header = marker.header;
                listener_.waitForTransform(marker.header.frame_id, base_frame_id, ros::Time(0), ros::Duration(1.0));
                listener_.transformPose(base_frame_id, marker.pose, marker_pose);
                update = true;
                lastupdate = ros::Time::now().toSec();
            } catch (tf::TransformException &ex) {
                //ROS_INFO("%s : TF_ERROR!", frame_id.c_str());
                update = false;
            }
            */
        }
    }

    //ar_pose_marker callback function - update and publish move message
    void sendCmdVel() {
        if(!update){
            sendCmdHover();
            return;
        }

        double posex = marker.pose.pose.position.x;
        double posey = marker.pose.pose.position.y;
        double posez = marker.pose.pose.position.z;

        double outforward = offset.z + posez;
        double outleft = -(offset.x + posex);
        double outup = -(offset.y + posey);

        double sp = 1.0;
        double p = 1.5;

        outforward = outforward > 0 ? sp * pow(fabs(outforward), p) : -sp * pow(fabs(outforward), p);
        outleft = outleft > 0 ? sp * pow(fabs(outleft), p) : -sp * pow(fabs(outleft), p);
        outup = outup > 0 ? 1.5 * sp * pow(fabs(outup), p) : -1.5 * sp * pow(fabs(outup), p);


        //apply clamp and thresh
        clamp(outforward, -MAX_SPEED, MAX_SPEED);
        clamp(outleft, -MAX_SPEED, MAX_SPEED);
        clamp(outup, -MAX_SPEED, MAX_SPEED);
        dead(outforward, DEADZONE);
        dead(outleft, DEADZONE);
        dead(outup, DEADZONE);

        // publishing velocity movements to "cmd_vel" to make the bebop move


        // move.linear.x -> forward(+)/backward(-)
        // move.linear.y -> left(+)/right(-)
        // move.linear.z -> up(+)/down(-)
        // move.angular.z -> counter-clock(+)/clock(-)

        move.linear.x = outforward;
        move.linear.y = outleft;
        move.linear.z = outup;
        move.angular.z = 0;
        cmd_vel.publish(move);
    }

    void sendCmdHover() {
        move.linear.x = 0;
        move.linear.y = 0;
        move.linear.z = 0;

        move.angular.x = 0;
        move.angular.y = 0;
        move.angular.z = 0;

        cmd_vel.publish(move);
        ROS_INFO("HOVER");
    }


    //clamp toClamp between min and max - toClamp is passed by reference so no need to return anything
    void clamp(double &toClamp, double min, double max) {
        if (toClamp > max) toClamp = max;
        else if (toClamp < min) toClamp = min;
    }

    //create a deadzone for toDead - if it is within deadThresh of 0, clamp it to 0
    void dead(double &toDead, double deadThresh) {
        if (toDead < deadThresh && toDead > -deadThresh) toDead = 0.0;
    }
};

//main method, entrance to ROS node
int main(int argc, char **argv) {
    //Initialize ROS, name node track_tag
    ros::init(argc, argv, "tag_info_fusion");
    //create the NodeHandle
    ros::NodeHandle nh("~");

    std::string target_name = "#####";
    nh.getParam("target_name", target_name);
    if(target_name == "#####"){
        ROS_INFO("No target name!!");
        return 0;
    }

    ROS_INFO("target name : %s",target_name.c_str());

    Tello tello;
    tello.cmd_vel = nh.advertise<geometry_msgs::Twist>("/" + target_name + "/cmd_vel", 1);
    tello.ar_pose = nh.subscribe("/" + target_name + "/ar_pose_marker",1,&Tello::poseCallback,&tello);

    vec offset;
    nh.getParam("offsetx",offset.x);
    nh.getParam("offsety",offset.y);
    nh.getParam("offsetz",offset.z);
    tello.set_offset(offset);


    //call callbacks until ROS shutdown
    try {
        ros::spin();
    } catch (...) {
        ROS_ERROR("--- ERROR IN spin(), shutting down! ---");
        ros::shutdown();
    }
    return 0;
}
