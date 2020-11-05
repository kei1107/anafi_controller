////////////////////////////////////////////////////////////////////////////////
/// @file           track_target_points.cpp
/// @brief          A program to control UAV position and posture
/// @author         Keisuke KIMURA
/// @date           2020/06/01
/// $Version:       1.0.0
/// @note           Prerequisite for use with ROS nodelet.
/// @par            History
///                 2020/06/01 : Support for doxygen
///
/// Copyright (c) 2020 Keisuke KIMURA. All Rights reserved.
/// This software is released under the MIT License.
/// http://opensource.org/licenses/mit-license.php
///
////////////////////////////////////////////////////////////////////////////////


#include <boost/version.hpp>

#if ((BOOST_VERSION / 100) % 1000) >= 53

#include <boost/thread/lock_guard.hpp>

#endif

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <fstream>
#include <deque>

namespace tello_controller {
    const double pi = acos(-1.0);   //!< Definition of PI
    const double eps = 1e-9;    //!< Definition of EPS

    /**
     * @brief track_target_points
     * @brief A class to control UAV position and posture
     */
    class track_target_points : public nodelet::Nodelet {
        // ========================================================== //
        // 変数定義
        // ========================================================== //
        double offset_x;    //!< x(right) offset
        double offset_y;    //!< y(down) offset
        double offset_z;    //!< z(forward) offset
        double MAX_ANGULAR; //!< maximum cmd angular
        double MAX_SPEED;   //!< maximum cmd speed

        double ReachDist;   //!< waypoint arrival parameter(distance)
        double ReachAngle;  //!< waypoint arrival parameter(angle)

        int target_num; //!< way point num
        int now_target; //!< next way point

        std::deque<bool> ok;    //!< Determination of arrival in the last few frames

        bool enable_wp_loop;    //!< way point loop flag
        std::string telloFrame; //!< UAV Frame
        std::string wpFrame;    //!< base Frame
        std::vector<std::vector<double>> wp_info; //!< way point information

        ros::Timer timer_pub_cmd; //!< ros timer for publish cmd_vel
        ros::Timer timer_pub_wp;    //!< ros time for publish next way point TF
        tf::TransformListener listener_;    //!< TF listener
        ros::Publisher cmd_vel; //!< Publisher of cmd_vel
        ros::Publisher img_saver_dummy;    //!< Publisher of dummy save image information
        ros::ServiceClient img_client;  //!< Call of save image information

        tf::TransformBroadcaster tf_broadcaster;    //!< TF broadcaster

        // ========================================================== //
        // 関数定義
        // ========================================================== //
        //　initializer
        /**
         * @brief initializer
         */
        virtual void onInit();

        // 補助関数
        /**
         * @brief convert degree to radian
         * @param deg degree
         * @return radian
         */
        double degree2radian(double deg);

        /**
         * @brief convert radian to degree
         * @param rad radian
         * @return degree
         */
        double radian2degree(double rad);

        // ウェイポイントの読み取り
        /**
         * @brief read way point information
         * @param way_point_file way point file path
         * @return success?
         */
        bool read_way_point(const std::string &way_point_file);

        // 指令値出力用関数
        /**
         * @brief Callback function for publish cmd_vel
         */
        void timer_pub_cmd_callback(const ros::TimerEvent &);

        // 次のウェイポイント出力用関数
        /**
         * @brief Callback function for publish next way point TF
         */
        void timer_pub_wp_callback(const ros::TimerEvent &);

        // clamp関数
        /**
         * @brief clamp function
         * @param toClamp target
         * @param min mininum value
         * @param max maximum value
         */
        void clamp(double &toClamp, double min, double max);
    };

    void track_target_points::onInit() {
        NODELET_INFO("Tello_position_estimator_no_move_posture Init");
        ros::NodeHandle &nh = getNodeHandle();
        ros::NodeHandle &private_nh = getPrivateNodeHandle();

        now_target = 0;
        // Read parameters
        int queue_size;
        private_nh.param("queue_size", queue_size, 5);

        private_nh.param("start", now_target, 0);

        private_nh.param("offset_x", offset_x, 0.0);
        private_nh.param("offset_y", offset_y, 0.0);
        private_nh.param("offset_z", offset_z, 0.0);

        private_nh.param("MAX_ANGULAR", MAX_ANGULAR, 10.0);
        MAX_ANGULAR = degree2radian(MAX_ANGULAR);
        private_nh.param("MAX_SPEED", MAX_SPEED, 0.25);
        private_nh.param("ReachDist", ReachDist, 0.1);
        private_nh.param("ReachAngle", ReachAngle, 5.0);
        ReachAngle = degree2radian(ReachAngle);

        private_nh.param("telloFrame", telloFrame, std::string("telloFrame"));
        private_nh.param("wpFrame", wpFrame, std::string("wpFrame"));
        private_nh.param("enable_wp_loop", enable_wp_loop, false);

        NODELET_INFO("offset_x : %lf[m]", offset_x);
        NODELET_INFO("offset_y : %lf[m]", offset_y);
        NODELET_INFO("offset_z : %lf[m]", offset_z);

        std::string way_point_file;
        private_nh.param("way_point_file", way_point_file, std::string(""));
        // Read mask rules
        if (way_point_file == std::string("")) {
            NODELET_FATAL("Parameter not set : way_point_file\nStop program.");
            return;
        } else {
            bool ok = read_way_point(way_point_file);
            if (!ok) {
                NODELET_FATAL("Stop program.");
                return;
            }
        }

        cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        img_saver_dummy = nh.advertise<std_msgs::Empty>("/image_saver/save", 1);
        img_client = nh.serviceClient<std_srvs::Empty>("/image_saver/save");
        timer_pub_cmd = nh.createTimer(ros::Duration(0.1), &track_target_points::timer_pub_cmd_callback,
                                       this); //  0.1 = 10Hz
        timer_pub_wp = nh.createTimer(ros::Duration(0.1), &track_target_points::timer_pub_wp_callback,
                                      this); //  0.1 = 10Hz / 0.01 = 100Hz

    }

    double track_target_points::degree2radian(double deg) {
        return deg * pi / 180.0;
    }

    double track_target_points::radian2degree(double rad) {
        return rad * 180.0 / pi;
    }

    bool track_target_points::read_way_point(const std::string &way_point_file) {
        std::ifstream ifs(way_point_file.c_str());
        if (!ifs) {
            NODELET_FATAL("Don't open file : %s", way_point_file.c_str());
            return false;
        }
        int wpnum, infonum;
        ifs >> wpnum >> infonum;
        wp_info.assign(wpnum, std::vector<double>(infonum, 0.0));
        target_num = wpnum;
        for (int i = 0; i < wpnum; i++) {
            for (int j = 0; j < infonum; j++) {
                ifs >> wp_info[i][j];
            }
            if (infonum >= 6) {
                std::swap(wp_info[i][4], wp_info[i][5]);
                wp_info[i][4] = -1.0 * wp_info[i][4] + pi / 2.0;
            }
            for (int j = 3; j < infonum; j++) {
                // wp_info[i][j] = degree2radian(wp_info[i][j]);
            }
        }
        ROS_INFO("Read %d way point", wpnum);
        return true;
    }

    void track_target_points::timer_pub_cmd_callback(const ros::TimerEvent &) {
        if (now_target >= target_num) return;

        std::string targetFrame = "way_point";
        tf::StampedTransform transform;
        try {
            listener_.waitForTransform(telloFrame, targetFrame, ros::Time(0), ros::Duration(1.0));
            listener_.lookupTransform(telloFrame, targetFrame, ros::Time(0), transform);
        } catch (tf::TransformException &ex) {
            NODELET_ERROR("%s", ex.what());
            return;
        }

        double outforward = 1e9, outleft = 1e9, outup = 1e9;
        double outangle = 0.0;

        double target_dist = std::numeric_limits<double>::max();
        double target_angle = 0.0;

        { // 次のウェイポインtへの移動前に、そのウェイポイントにそって垂直方向を向くかどうか。（現在、未使用）
//            tf::Vector3 origin = transform.getOrigin();
//            double dx = origin.x() - offset_x;
//            double dz = origin.z() - offset_z;
//
//            // theta = (-pi, pi)
//            double theta = -std::atan2(dz,dx);
//            theta += pi; // theta = (0,2*pi)
//
//            int cnt = 0;
//            while(theta > pi/4.0 - eps){
//                cnt++;
//                theta -= pi/4.0;
//            }
//            if(cnt%2 == 0){
//
//            }else{
//                theta = theta - pi/4.0;
//            }
//            if(std::abs(theta)  > degree2radian(10)){
//                NODELET_INFO("rotate : %lf",radian2degree(theta));
//
//                clamp(theta, -MAX_ANGULAR,MAX_ANGULAR);
//                geometry_msgs::Twist move;
//                move.angular.z = theta;
//
//                cmd_vel.publish(move);
//                return ;
//            }
        }

        { // 指令値計算（位置）
            tf::Vector3 origin = transform.getOrigin();
            outforward = (origin.z() - offset_z);
            outleft = -(origin.x() - offset_x);
            outup = -(origin.y() - offset_y);

            NODELET_INFO("Now target %d , %s to %s is (%lf,%lf,%lf)", now_target, telloFrame.c_str(),
                         targetFrame.c_str(), outforward, outleft, outup);
            target_dist = std::sqrt(outforward * outforward + outleft * outleft + outup * outup);

//            double sp = 1.0;
//            double p = 1.0;
//            outforward = outforward > 0 ? sp * pow(fabs(outforward), p) : -sp * pow(fabs(outforward), p);
//            outleft = outleft > 0 ? sp * pow(fabs(outleft), p) : -sp * pow(fabs(outleft), p);
//            outup = outup > 0 ? 1.5 * sp * pow(fabs(outup), p) : -1.5 * sp * pow(fabs(outup), p);
        }

        // 指令値計算（角度）　指令値（位置）が一定値以内の場合のみ動作　：　まずおおよそ位置を合わせてそのあと角度を合わせたい
        if (target_dist < ReachDist && wp_info[0].size() >= 6) { // calclate angle cmd
            tf::Quaternion tello2targetQ = transform.getRotation();
            tf::Transform angle_tf;
            angle_tf.setRotation(tello2targetQ);
            tf::Vector3 angle_tv = angle_tf * tf::Vector3(0, 0, 1);
            outangle = -std::atan2(angle_tv.x(), angle_tv.z()); // cmd_z : counter-clockwise
            target_angle = std::abs(outangle);
            NODELET_INFO("Dist ok. cmd_angular %lf", outangle);
        }

        // 直近10フレームのうち、いくつが有効な位置に入っているかを計算
        if (target_dist < ReachDist && target_angle < ReachAngle) {
            ok.push_back(true);
        } else {
            ok.push_back(false);
        }
        while (ok.size() > 10) ok.pop_front();
        if (ok.size() != 10) return;

        int ok_cnt = std::count(ok.begin(), ok.end(), true);

        // 直近10フレームのうち5フレーム以上ウェイポイントに到達しているとみなされれば到達判定
        if (ok_cnt >= 5) {
            ok.clear();
            std_msgs::Empty dummy_msg;
            std_srvs::Empty msg;

            img_saver_dummy.publish(dummy_msg);
            img_client.call(msg);
            NODELET_INFO("Reach Target %d", now_target);
            // 次のウェイポイントに更新
            now_target = now_target + 1;
            if (now_target >= target_num) {
                if (enable_wp_loop) {
                    now_target -= target_num;
                }
            }
            return;
        }

        //apply clamp and thresh
        clamp(outforward, -MAX_SPEED, MAX_SPEED);
        clamp(outleft, -MAX_SPEED, MAX_SPEED);
        clamp(outup, -MAX_SPEED, MAX_SPEED);
        clamp(outangle, -MAX_ANGULAR, MAX_ANGULAR);

        // 指令値生成
        geometry_msgs::Twist move;
        move.linear.x = outforward;
        move.linear.y = outleft;
        move.linear.z = outup;
        move.angular.z = outangle;

        NODELET_INFO("forward : %lf, left : %lf, up : %lf, angle : %lf", outforward, outleft, outup, outangle);
        // 指令値配信
        cmd_vel.publish(move);
    }

    void track_target_points::timer_pub_wp_callback(const ros::TimerEvent &) {
        tf::Quaternion tQ;
        if (wp_info[0].size() <= 3) {
            tQ = tf::createQuaternionFromRPY(0, 0, 0);
        } else {
            tQ = tf::createQuaternionFromRPY(wp_info[now_target][3], wp_info[now_target][4], wp_info[now_target][5]);
        }
        tf::Transform t(tQ, tf::Vector3(wp_info[now_target][0], wp_info[now_target][1], wp_info[now_target][2]));
        ros::Time nowt = ros::Time::now();
        tf::StampedTransform wpTF(t, nowt, wpFrame, "way_point");
        tf_broadcaster.sendTransform(wpTF);
    }

    void track_target_points::clamp(double &toClamp, double min, double max) {
        if (toClamp > max) toClamp = max;
        else if (toClamp < min) toClamp = min;
    }
} // namespace tello_controller
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(tello_controller::track_target_points, nodelet::Nodelet);
