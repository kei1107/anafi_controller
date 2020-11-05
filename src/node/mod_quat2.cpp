#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <pcl_ros/transforms.h>


const double pi = acos(-1.0);

int main(int argc, char **argv) {
    //Initialize ROS, name node track_tag
    ros::init(argc, argv, "mod_quat2");

    //create the NodeHandle
    ros::NodeHandle nh("~");

    tf::TransformBroadcaster tf_broadcaster;
    tf::TransformListener listener_;

    tf::Transform baseFrame2wpFrame;
    tf::Transform wpFrame2way_point;
    tf::Transform baseFrame2telloFrame;

    baseFrame2wpFrame.setOrigin(tf::Vector3(-1,0.2,0.3));
    baseFrame2wpFrame.setRotation(tf::createQuaternionFromRPY(0,-pi/3,0));

    wpFrame2way_point.setOrigin(tf::Vector3(-0.3,0,0.2));
    wpFrame2way_point.setRotation(tf::createQuaternionFromRPY(0,-pi,0));

    baseFrame2telloFrame.setOrigin(tf::Vector3(0,0,0.5));
    baseFrame2telloFrame.setRotation(tf::createQuaternionFromRPY(0,-pi/4,0));


    ros::Publisher pub = nh.advertise<nav_msgs::Path>("/path",1);
    ros::Publisher pub2 = nh.advertise<nav_msgs::Path>("/path2",1);

    ros::Rate rate(50);
    while(ros::ok()) {
        ros::Time nowt = ros::Time::now();
        tf_broadcaster.sendTransform(tf::StampedTransform(baseFrame2wpFrame,nowt,"baseFrame","wpFrame"));
        tf_broadcaster.sendTransform(tf::StampedTransform(wpFrame2way_point,nowt,"wpFrame","way_point"));
        tf_broadcaster.sendTransform(tf::StampedTransform(baseFrame2telloFrame,nowt,"baseFrame","telloFrame"));

        tf::StampedTransform transform;
        try{
            listener_.waitForTransform("telloFrame", "way_point", ros::Time(0), ros::Duration(1.0));
            listener_.lookupTransform("telloFrame","way_point",ros::Time(0),transform);
        }catch (tf::TransformException& ex) {
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        tf::Quaternion tello2targetQ = transform.getRotation();
        tf::Transform angle_tf;
        angle_tf.setRotation(tello2targetQ);
        tf::Vector3 angle_tv = angle_tf * tf::Vector3(0,0,1);
        double angular = std::atan2(angle_tv.x(), angle_tv.z());
        // ROS_INFO_STREAM(transform.frame_id_ << " " <<transform.child_frame_id_);
        ROS_INFO("x : %lf , y : %lf , z : %lf",angle_tv.x(),angle_tv.y(),angle_tv.z());
        ROS_INFO("angle : %lf/%lf",angular,angular/pi*180);
        {
            nav_msgs::Path info2;
            info2.header.seq = 1;
            info2.header.stamp = nowt;
            info2.header.frame_id = "telloFrame";

            info2.poses.resize(2);
            info2.poses[0].pose.position.x = 0;
            info2.poses[0].pose.position.y = 0;
            info2.poses[0].pose.position.z = 0;

            info2.poses[1].pose.position.x = angle_tv.x();
            info2.poses[1].pose.position.y = angle_tv.y();
            info2.poses[1].pose.position.z = angle_tv.z();
            pub2.publish(info2);
        }

        tf::Transform t;
        t.setOrigin(baseFrame2telloFrame.getOrigin());
        t.setRotation(baseFrame2telloFrame.getRotation()*tf::createQuaternionFromRPY(0,angular,0));
        tf_broadcaster.sendTransform(tf::StampedTransform(t, nowt, "baseFrame", "telloFrame_rot"));

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}