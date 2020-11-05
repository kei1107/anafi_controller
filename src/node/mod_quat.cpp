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


//   <node pkg="tf" type="static_transform_publisher" name="camera5_to_tello0" output="screen" args="0 0 2 -0.0265666 0.923372  0.0125344  0.382781 camera5_color_optical_frame tello0 10"/>
//main method, entrance to ROS node
int main(int argc, char **argv) {
    //Initialize ROS, name node track_tag
    ros::init(argc, argv, "mod_quat");
    //create the NodeHandle
    ros::NodeHandle nh("~");

    double nx = 0,ny = 0,nz = 0.5;
    double qx = -0.0265666, qy = 0.923372, qz = 0.0125344, qw = 0.382781;

    tf::Transform transformer;
    transformer.setOrigin(tf::Vector3(nx,ny,nz));
    transformer.setRotation(tf::Quaternion(qx,qy,qz,qw));

    double pi = acos(-1.0);
    tf::Quaternion tQ = tf::createQuaternionFromRPY(0,-pi/2,0);

    tf::Quaternion attitude_or = transformer * tQ;
    ROS_INFO_STREAM(attitude_or.x() << " " << attitude_or.y() << " " << attitude_or.z() << " " << attitude_or.w());

    ros::Publisher pub = nh.advertise<nav_msgs::Path>("/path",1);
    ros::Publisher pub2 = nh.advertise<nav_msgs::Path>("/path2",1);

    ros::Rate rate(50);


    tf::Quaternion b2t_q,t2b_q;
    double Qx =-0.00992226;
    double Qy =   0.382256;
    double Qz =  0.0276486;
    double Qw =    0.92359;

    b2t_q.setX(Qx);
    b2t_q.setY(Qy);
    b2t_q.setZ(Qz);
    b2t_q.setW(Qw);

    t2b_q.setX(-Qx);
    t2b_q.setY(-Qy);
    t2b_q.setZ(-Qz);
    t2b_q.setW(Qw);

    while(ros::ok()) {
        ros::Time nowt = ros::Time::now();

        tf::TransformBroadcaster tf_broadcaster;
        tf::Transform t(b2t_q, tf::Vector3(nx, ny, nz));
        tf::StampedTransform BaseToTello(t, nowt, "camera7_color_optical_frame", "tello0");
        tf_broadcaster.sendTransform(BaseToTello);

        nav_msgs::Path info;
        info.header.seq = 1;
        info.header.stamp = nowt;
        info.header.frame_id = "camera7_color_optical_frame";

        info.poses.resize(2);
        info.poses[0].pose.position.x = 0;
        info.poses[0].pose.position.y = 0;
        info.poses[0].pose.position.z = 0.5;

        info.poses[1].pose.position.x = 1;
        info.poses[1].pose.position.y = -0.4;
        info.poses[1].pose.position.z = 0;
        pub.publish(info);

        double dx = 1 - 0;
        double dy = -0.4 - 0;
        double dz = 0 - 0.5;

        tf::Transform angle_tf;
        angle_tf.setRotation(t2b_q);
        tf::Vector3 angle_tv = angle_tf * tf::Vector3(dx, dy,dz);
        nav_msgs::Path info2;
        info2.header.seq = 1;
        info2.header.stamp = nowt;
        info2.header.frame_id = "tello0";

        info2.poses.resize(2);
        info2.poses[0].pose.position.x = 0;
        info2.poses[0].pose.position.y = 0;
        info2.poses[0].pose.position.z = 0;

        info2.poses[1].pose.position.x = angle_tv.x();
        info2.poses[1].pose.position.y = angle_tv.y();
        info2.poses[1].pose.position.z = angle_tv.z();
        pub2.publish(info2);

        double angle_val = -std::atan2(angle_tv.z(), angle_tv.x());
        double pi = acos(-1.0);
        ROS_INFO("x : %lf , y : %lf , z : %lf",angle_tv.x(),angle_tv.y(),angle_tv.z());
        ROS_INFO("angle : %lf/%lf",angle_val,angle_val/pi*180);
        // ROS_INFO("%lf",angle_val/pi*180);

        tf::Vector3 TV(0, 0, 0.5);
        t.setOrigin(TV);
        t.setRotation(b2t_q*tf::createQuaternionFromRPY(0,angle_val,0));

        tf::StampedTransform BaseToTello2(t, nowt, "camera7_color_optical_frame", "tello0mod");
        tf_broadcaster.sendTransform(BaseToTello2);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}