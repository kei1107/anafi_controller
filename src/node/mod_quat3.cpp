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

    tf::Transform baseFrame2telloFrame;
    baseFrame2telloFrame.setOrigin(tf::Vector3(0,0,0.5));
    baseFrame2telloFrame.setRotation(tf::createQuaternionFromRPY(0,-pi/4,0));

    tf::Quaternion t2b_q;
    t2b_q.setX(-baseFrame2telloFrame.getRotation().x());
    t2b_q.setY(-baseFrame2telloFrame.getRotation().y());
    t2b_q.setZ(-baseFrame2telloFrame.getRotation().z());
    t2b_q.setW(baseFrame2telloFrame.getRotation().w());

    ros::Publisher pub = nh.advertise<nav_msgs::Path>("/path",1);
    ros::Publisher pub2 = nh.advertise<nav_msgs::Path>("/path2",1);

    ros::Rate rate(50);
    while(ros::ok()) {
        ros::Time nowt = ros::Time::now();
        tf_broadcaster.sendTransform(tf::StampedTransform(baseFrame2telloFrame,nowt,"baseFrame","telloFrame"));

        tf::StampedTransform transform;
        try{
            listener_.waitForTransform("telloFrame", "baseFrame", ros::Time(0), ros::Duration(1.0));
            listener_.lookupTransform("telloFrame","baseFrame",ros::Time(0),transform);
        }catch (tf::TransformException& ex) {
            ros::spinOnce();
            rate.sleep();
            continue;
        }

//        ROS_INFO("base2tello   : %lf,%lf,%lf,%lf",baseFrame2telloFrame.getRotation().x(), baseFrame2telloFrame.getRotation().y() , baseFrame2telloFrame.getRotation().z(),baseFrame2telloFrame.getRotation().w());
//        ROS_INFO("tello2base   : %lf,%lf,%lf,%lf",transform.getRotation().x(),transform.getRotation().y() , transform.getRotation().z() ,transform.getRotation().w());
//        ROS_INFO("tello2base_q : %lf,%lf,%lf,%lf",t2b_q.x(),t2b_q.y() ,t2b_q.z(),t2b_q.w());

        nav_msgs::Path info;
        info.header.seq = 1;
        info.header.stamp = nowt;
        info.header.frame_id = "baseFrame";

        info.poses.resize(2);
        info.poses[0].pose.position.x = 0;
        info.poses[0].pose.position.y = 0;
        info.poses[0].pose.position.z = 0.5;

        info.poses[1].pose.position.x = 1;
        info.poses[1].pose.position.y = 0.4;
        info.poses[1].pose.position.z = 0.5;
        pub.publish(info);

        double dx = 1 - 0;
        double dy = 0.4 - 0;
        double dz = 0.5 - 0.5;

        tf::Quaternion tello2targetQ = transform.getRotation();
        tf::Transform angle_tf;
        // angle_tf.setRotation(tello2targetQ);
        angle_tf.setRotation(t2b_q);
        tf::Vector3 angle_tv = angle_tf * tf::Vector3(dx, dy,dz);
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

        double angle_val = std::atan2(angle_tv.x(), angle_tv.z());

        ROS_INFO("x : %lf , y : %lf , z : %lf",angle_tv.x(),angle_tv.y(),angle_tv.z());
        ROS_INFO("angle : %lf/%lf",angle_val,angle_val/pi*180);

        tf::Transform t;
        t.setOrigin(baseFrame2telloFrame.getOrigin());
        tf::Quaternion tQinv = transform.getRotation().inverse();

        ROS_INFO("base2tello   : %lf,%lf,%lf,%lf",baseFrame2telloFrame.getRotation().x(), baseFrame2telloFrame.getRotation().y() , baseFrame2telloFrame.getRotation().z(),baseFrame2telloFrame.getRotation().w());
        ROS_INFO("base2tello   : %lf,%lf,%lf,%lf",tQinv.x(), tQinv.y() ,tQinv.z(),tQinv.w());

        t.setRotation(tQinv*tf::createQuaternionFromRPY(0,angle_val,0));
        // t.setRotation(baseFrame2telloFrame.getRotation()*tf::createQuaternionFromRPY(0,angle_val,0));
        tf_broadcaster.sendTransform(tf::StampedTransform(t, nowt, "baseFrame", "telloFrame_rot"));


//        tf::Transform t;
//        t.setOrigin(baseFrame2telloFrame.getOrigin());
//        t.setRotation(baseFrame2telloFrame.getRotation()*tf::createQuaternionFromRPY(0,angular,0));
        // tf_broadcaster.sendTransform(tf::StampedTransform(transform, nowt, "telloFrame", "telloFrame_rot"));

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}