#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char **argv) {
    //Initialize ROS, name node track_tag
    ros::init(argc, argv, "moveTo_test");
    //create the NodeHandle
    ros::NodeHandle nh("~");

    ros::Rate rate(0.2);

    ros::Publisher takeoff_pub =  nh.advertise<std_msgs::Empty>("/anafi/takeoff",1);
    ros::Publisher land_pub =  nh.advertise<std_msgs::Empty>("/anafi/land",1);
    ros::Publisher moveTo_pub = nh.advertise<geometry_msgs::Twist>("/anafi/gps_moveto",1);

    double home_lati = 48.8789000031;
    double home_long = 2.36778003052;
    double home_alti = 1.00026154518;


    double target_lati = 48.8789127653;
    double target_long = 2.36772056226;
    double target_alti = 1.70950913429;


    sleep(3);
    ROS_INFO("start");
    std_msgs::Empty dummy_msg;
    takeoff_pub.publish(dummy_msg);
    ROS_INFO("pub takeoff");
    sleep(10);

    int counter = 0;
    int MAX_counter = 5;

    // goto home
    {
        ros::Time nowt = ros::Time::now();

        double now_lati = home_lati + (target_lati - home_lati)*counter/(double)MAX_counter;
        double now_long = home_long + (target_long - home_long)*counter/(double)MAX_counter;
        double now_alti = home_alti + (target_alti - home_alti)*counter/(double)MAX_counter;


        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = now_lati;
        twist_msg.linear.y = now_long;
        twist_msg.linear.z = now_alti;

        moveTo_pub.publish(twist_msg);
        counter++;
        ROS_INFO("goto home");
        sleep(10);
    }

    while(ros::ok()) {
        if(counter > MAX_counter) break;
        ROS_INFO("count : %d , [s]",counter);

        double now_lati = home_lati + (target_lati - home_lati)*counter/(double)MAX_counter;
        double now_long = home_long + (target_long - home_long)*counter/(double)MAX_counter;
        double now_alti = home_alti + (target_alti - home_alti)*counter/(double)MAX_counter;


        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = now_lati;
        twist_msg.linear.y = now_long;
        twist_msg.linear.z = now_alti;

        moveTo_pub.publish(twist_msg);

        counter++;
        ros::spinOnce();
        rate.sleep();
    }

//    sleep(10);
//    land_pub.publish(dummy_msg);
//    ROS_INFO("pub land");
    return 0;
}