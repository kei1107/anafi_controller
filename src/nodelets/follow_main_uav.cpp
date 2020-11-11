////////////////////////////////////////////////////////////////////////////////
/// @file           follow_main_uav.cpp
/// @brief          A program to follow main UAV ( for RTF )
/// @author         Keisuke KIMURA
/// @date           2020/11/11
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

namespace gps_lib {
    // reference : https://www.mk-mode.com/blog/2020/10/14/cpp-geodesical-calculation-by-vincenty-1/#
    // reference : https://www.ngs.noaa.gov/PUBS_LIB/inverse.pdf

    class Vincenty {
        double b;                     // 極半径（短半径）
        double phi_1;                 // 地点 1 の緯度
        double l_1;                   // 地点 1 の経度
        double tan_u_1;               // tan(地点 1 の更成緯度)
        double u_1;                   // 地点 1 の更成緯度
        double deg2rad(double);       // 度 => ラジアン
        double rad2deg(double);       // ラジアン => 度
        double calc_a(double);        // A 計算
        double calc_b(double);        // B 計算
        double calc_c(double);        // C 計算
        double calc_dlt_sgm(double, double, double, double);  // Δσ 計算
        double norm_lng(double);      // 経度正規化
        double norm_az(double);       // 方位角正規化

    public:
        Vincenty(double, double);  // コンストラクタ
        std::tuple<double, double, double> calc_distance(double, double);
        // 距離と方位角 1, 2 を計算（Vincenty 逆解法）
    };

// 定数
    constexpr double kA = 6378137.0;             // GRS80 長半径
    constexpr double kF = 1.0 / 298.257222101;   // GRS80 扁平率
    constexpr double kPi = 3.14159265358979323846;  // PI
    constexpr double kPi2 = kPi * 2.0;             // PI * 2
    constexpr double kPi180 = kPi / 180.0;           // PI / 180
    constexpr double kPi180Inv = 1.0 / kPi180;          // 1 / (PI / 180)
    constexpr double kEps = 1.0e-11;               // 1^(-11) = 10^(-12) で 0.06mm の精度

    Vincenty::Vincenty(double lat_1, double lng_1) {
        try {
            b = kA * (1.0 - kF);
            phi_1 = deg2rad(lat_1);
            l_1 = deg2rad(lng_1);
            tan_u_1 = (1.0 - kF) * std::tan(phi_1);
            u_1 = std::atan(tan_u_1);
        } catch (...) {
            throw;
        }
    }

    /**
     * @brief      距離と方位角 1, 2 を計算
     *             （Vincenty 逆解法）
     *
     * @param[in]  lat_2: 緯度(double)
     * @param[in]  lng_2: 経度(double)
     * @return     {
     *                  s: 距離と方位角1,2を計算(double),
     *               az_1: 地点 1 における方位角(double),
     *               az_2: 地点 2 における方位角(double)
     *             }(tuple)
     */
    std::tuple<double, double, double> Vincenty::calc_distance(
            double lat_2, double lng_2) {
        double s = 0.0;  // 地点 1 と 2 の距離
        double az_1 = 0.0;  // 地点 1 における方位角
        double az_2 = 0.0;  // 地点 2 における方位角
        double phi_2;
        double l_2;
        double u_2;
        double cos_u_1;  // cos(u_1)
        double cos_u_2;  // cos(u_2)
        double sin_u_1;  // sin(u_1)
        double sin_u_2;  // sin(u_2)
        double su1_su2;  // sin(u_1) * sin(u_2)
        double su1_cu2;  // sin(u_1) * cos(u_2)
        double cu1_su2;  // cos(u_1) * sin(u_2)
        double cu1_cu2;  // cos(u_1) * cos(u_2)
        double l;
        double lmd;
        double lmd_prev;
        double cos_lmd;
        double sin_lmd;
        double t_0;
        double t_1;
        double sin_sgm;
        double cos_sgm;
        double sgm;
        double sin_alp;
        double cos2_alp;
        double cos_2_sgm_m;
        double aa;     // A
        double bb;     // B
        double cc;     // C
        double u2;
        double d_sgm;  // Δσ
        double alp_1;
        double alp_2;

        try {
            phi_2 = deg2rad(lat_2);
            l_2 = deg2rad(lng_2);
            u_2 = std::atan((1.0 - kF) * std::tan(phi_2));
            cos_u_1 = std::cos(u_1);
            cos_u_2 = std::cos(u_2);
            sin_u_1 = std::sin(u_1);
            sin_u_2 = std::sin(u_2);
            su1_su2 = sin_u_1 * sin_u_2;
            su1_cu2 = sin_u_1 * cos_u_2;
            cu1_su2 = cos_u_1 * sin_u_2;
            cu1_cu2 = cos_u_1 * cos_u_2;
            l = norm_lng(l_2 - l_1);
            lmd = l;
            lmd_prev = kPi2;
            cos_lmd = std::cos(lmd);
            sin_lmd = std::sin(lmd);

            while (std::abs(lmd - lmd_prev) > kEps) {
                t_0 = cos_u_2 * sin_lmd;
                t_1 = cu1_su2 - su1_cu2 * cos_lmd;
                sin_sgm = std::sqrt(t_0 * t_0 + t_1 * t_1);
                cos_sgm = su1_su2 + cu1_cu2 * cos_lmd;
                sgm = std::atan2(sin_sgm, cos_sgm);
                sin_alp = cu1_cu2 * sin_lmd / sin_sgm;
                cos2_alp = 1 - sin_alp * sin_alp;
                cos_2_sgm_m = cos_sgm - 2 * su1_su2 / cos2_alp;
                cc = calc_c(cos2_alp);
                lmd_prev = lmd;
                lmd = l + (1.0 - cc) * kF * sin_alp
                          * (sgm + cc * sin_sgm
                                   * (cos_2_sgm_m + cc * cos_sgm
                                                    * (-1.0 + 2.0 * cos_2_sgm_m * cos_2_sgm_m)));
                cos_lmd = std::cos(lmd);
                sin_lmd = std::sin(lmd);
                if (lmd > kPi) {
                    lmd = kPi;
                    break;
                }
            }

            u2 = cos2_alp * (kA * kA - b * b) / (b * b);
            aa = calc_a(u2);
            bb = calc_b(u2);
            d_sgm = calc_dlt_sgm(bb, cos_sgm, sin_sgm, cos_2_sgm_m);
            s = b * aa * (sgm - d_sgm);
            alp_1 = std::atan2(cos_u_2 * sin_lmd, cu1_su2 - su1_cu2 * cos_lmd);
            alp_2 = std::atan2(cos_u_1 * sin_lmd, -su1_cu2 + cu1_su2 * cos_lmd) + kPi;
            az_1 = rad2deg(norm_az(alp_1));
            az_2 = rad2deg(norm_az(alp_2));

            return {s, az_1, az_2};
        } catch (...) {
            throw;
        }

        return {s, az_1, az_2};  // 計算成功
    }

    /**
     * @brief      度 => ラジアン
     *
     * @param[in]  deg: 度      (double)
     * @return     rad: ラジアン(double)
     */
    double Vincenty::deg2rad(double deg) {
        try {
            return deg * kPi180;
        } catch (...) {
            return 0.0;
        }
    }

    /**
     * @brief      ラジアン => 度
     *
     * @param[in]  rad: ラジアン(double)
     * @return     deg: 度      (double)
     */
    double Vincenty::rad2deg(double rad) {
        try {
            return rad * kPi180Inv;
        } catch (...) {
            return 0.0;
        }
    }

    /**
     * @brief  A 計算
     *
     * @param[in] u2: u^2 の値
     * @return     a: A の値(double)
     */
    double Vincenty::calc_a(double u2) {
        try {
            return 1.0 + u2 / 16384.0 * (4096.0 + u2 * (-768.0 + u2 * (320.0
                                                                       - 175.0 * u2)));
        } catch (...) {
            return 0.0;
        }
    }

    /**
     * @brief  B 計算
     *
     * @param[in] u2: u^2 の値
     * @return     b: B の値(double)
     */
    double Vincenty::calc_b(double u2) {
        try {
            return u2 / 1024.0 * (256.0 + u2 * (-128.0 + u2 * (74.0 - 47.0 * u2)));
        } catch (...) {
            return 0.0;
        }
    }

    /**
     * @brief  C 計算
     *
     * @param[in] cos2_alp: cos^2(α) の値
     * @return           c: C の値(double)
     */
    double Vincenty::calc_c(double cos2_alp) {
        try {
            return kF * cos2_alp * (4.0 + kF * (4.0 - 3.0 * cos2_alp)) / 16.0;
        } catch (...) {
            return 0.0;
        }
    }

    /**
     * Δσ 計算
     *
     * @param[in]          bb: B の値
     * @param[in]     cos_sgm: cos(σ) の値
     * @param[in]     sin_sgm: sin(σ) の値
     * @param[in] cos_2_sgm_m: cos(2σ_m) の値
     * @return        dlt_sgm: Δσ の値
     */
    double Vincenty::calc_dlt_sgm(
            double bb, double cos_sgm, double sin_sgm, double cos_2_sgm_m) {
        try {
            return bb * sin_sgm * (cos_2_sgm_m
                                   + bb / 4.0 * (cos_sgm * (-1.0 + 2.0 * cos_2_sgm_m * cos_2_sgm_m)
                                                 - bb / 6.0 * cos_2_sgm_m * (-3.0 + 4.0 * sin_sgm * sin_sgm)
                                                   * (-3.0 + 4.0 * cos_2_sgm_m * cos_2_sgm_m)));
        } catch (...) {
            return 0.0;
        }
    }

    /**
     * @brief      経度正規化
     *
     * @param[in]  lng: 正規化前の経度(rad)(double)
     * @return     lng: 正規化後の経度(rad)(double)
     */
    double Vincenty::norm_lng(double lng) {
        try {
            while (lng < -kPi) lng += kPi2;
            while (lng > kPi) lng -= kPi2;
        } catch (...) {
            return 0.0;
        }

        return lng;
    }

    /**
     * 方位角正規化
     *
     *  @param[in]  alp: 正規化前の方位角(rad)
     *  @return     alp: 正規化後の方位角(rad)
     */
    double Vincenty::norm_az(double alp) {
        try {
            if (alp < 0.0) alp += kPi2;
            if (alp > kPi2) alp -= kPi2;
        } catch (...) {
            return 0.0;
        }

        return alp;
    }

}  // namespace gps_lib



namespace anafi_controller {
    const double pi = acos(-1.0);   //!< Definition of PI
    const double eps = 1e-9;    //!< Definition of EPS

    struct gps_info {
        double latitude;
        double longitude;
        double altitude;

        gps_info() : latitude(0), longitude(0), altitude(0) {}

        gps_info(double lat, double lng, double alt) : latitude(lat), longitude(lng), altitude(alt) {}
    };

    /**
     * @brief follow_main_uav
     * @brief A class to control UAV position and posture
     */
    class follow_main_uav : public nodelet::Nodelet {
        // ========================================================== //
        // 変数定義
        // ========================================================== //
        int wp_num; //!< the number of waypoint
        int sub_num; //!< the number of sub-uav
        std::vector<gps_info> main_wp_info;         //!< main way point information
        std::vector<double> main_wp_time;           //!< main way point time information
        double main_wp_time_interval;               //!< main way point time interval
        std::vector<double> cum_main_wp_time;

        std::vector<gps_info> sub_wp_info[4];   //!< sub way point information

        ros::Timer timer_pub_cmd; //!< ros timer for publish cmd_vel
        ros::Publisher takeoff_pub[4];
        ros::Publisher land_pub[4];
        ros::Publisher moveTo_pub[4];

        bool takeoff_flag;
        bool start_flag;
        ros::Subscriber sub_start_msg;
        ros::Time start_time;

        int counter;
        // ========================================================== //
        // 関数定義
        // ========================================================== //
        //　initializer
        /**
         * @brief initializer
         */
        virtual void onInit();

        // ウェイポイントの読み取り
        /**
         * @brief read way point information
         * @param way_point_file way point file path
         * @return success?
         */
        bool read_way_point_info(const std::string &way_point_file, std::vector<gps_info> &info);

        bool read_way_point_info(const std::string &way_point_file, std::vector<double> &time_info);

        void start_sub_callback(const std_msgs::EmptyConstPtr &msg);

        void timer_pub_cmd_callback(const ros::TimerEvent &);

        gps_info calc_target_gps(int wp_id, double elapsed_time, std::vector<gps_info> &info);
    };

    void follow_main_uav::onInit() {
        NODELET_INFO("Anafi : follow main UAV nodelets Init");
        ros::NodeHandle &nh = getNodeHandle();
        ros::NodeHandle &private_nh = getPrivateNodeHandle();

        takeoff_flag = false;
        start_flag = false;
        counter = 0;

        // Read main UAV parameters
        {
            std::string way_point_file_param = "main_way_point_file";
            std::string way_point_file;

            private_nh.param(way_point_file_param, way_point_file, std::string(""));
            if (way_point_file == std::string("")) {
                NODELET_FATAL("Parameter not set : main_way_point_file\nStop program.");
                return;
            } else {
                bool ok = read_way_point_info(way_point_file, main_wp_info);
                if (!ok) {
                    NODELET_FATAL("Stop program.");
                    return;
                }
                wp_num = (int) main_wp_info.size();
            }
            NODELET_INFO("UAV waypoint : %d", wp_num);


            private_nh.param("main_wp_time_interval", main_wp_time_interval, 0.0);
            NODELET_INFO("UAV waypoint time interval : %lf [s]", main_wp_time_interval);


            std::string way_point_time_file_param = "main_way_point_time_file";
            std::string way_point_time_file;

            private_nh.param(way_point_time_file_param, way_point_time_file, std::string(""));
            if (way_point_time_file == std::string("")) {
                NODELET_FATAL("Parameter not set : main_way_point_time_file\nStop program.");
                return;
            } else {
                bool ok = read_way_point_info(way_point_time_file, main_wp_time);
                if (!ok) {
                    NODELET_FATAL("Stop program.");
                    return;
                }
                if (wp_num != main_wp_time.size()) {
                    NODELET_FATAL("The number of waypoints does not match.　: main - time");
                    return;
                }

            }

            //! TODO : ログデータに応じて変更
            cum_main_wp_time.resize(wp_num, 0.0);
            for (int i = 1; i < wp_num; i++) {
                cum_main_wp_time[i] = cum_main_wp_time[i - 1] + main_wp_time[i] + main_wp_time_interval;
            }
        }

        // Read sub UAV parameters
        private_nh.param("sub_num", sub_num, 0);
        if (sub_num <= 0) {
            NODELET_FATAL("Parameter not set or negative value : sub_num\nStop program.");
            return;
        } else {
            for (int num = 0; num < sub_num; num++) {
                std::string way_point_file_param = "sub_way_point_file" + std::to_string(num);
                std::string way_point_file;

                private_nh.param(way_point_file_param, way_point_file, std::string(""));
                if (way_point_file == std::string("")) {
                    NODELET_FATAL("Parameter not set : sub_way_point_file%d\nStop program.", num);
                    return;
                } else {
                    bool ok = read_way_point_info(way_point_file, sub_wp_info[num]);
                    if (!ok) {
                        NODELET_FATAL("Stop program.");
                        return;
                    }
                    if (wp_num != sub_wp_info[num].size()) {
                        NODELET_FATAL("The number of waypoints does not match.　: main - sub%d", num);
                        return;
                    }
                }
            }
        }

        for (int num = 0; num < sub_num; num++) {
            takeoff_pub[num] = nh.advertise<std_msgs::Empty>("/anafi" + std::to_string(num) + "/takeoff", 1);
            land_pub[num] = nh.advertise<std_msgs::Empty>("/anafi" + std::to_string(num) + "/land", 1);
            moveTo_pub[num] = nh.advertise<geometry_msgs::Twist>("/anafi" + std::to_string(num) + "/gps_moveto", 1);
        }
        timer_pub_cmd = nh.createTimer(ros::Duration(1), &follow_main_uav::timer_pub_cmd_callback,
                                       this); //  1 = 1Hz
        sub_start_msg = nh.subscribe("/start", 1, &follow_main_uav::start_sub_callback, this);
    }

    bool follow_main_uav::read_way_point_info(const std::string &way_point_file, std::vector<gps_info> &info) {
        std::ifstream ifs(way_point_file.c_str());
        if (!ifs) {
            NODELET_FATAL("Don't open file : %s", way_point_file.c_str());
            return false;
        }
        double lat, lng, alt;
        while (ifs >> lat >> lng >> alt) {
            info.emplace_back(lat, lng, alt);
        }
        ROS_INFO("Read %d way point", (int) info.size());
        return true;
    }

    bool follow_main_uav::read_way_point_info(const std::string &way_point_file, std::vector<double> &time_info) {
        std::ifstream ifs(way_point_file.c_str());
        if (!ifs) {
            NODELET_FATAL("Don't open file : %s", way_point_file.c_str());
            return false;
        }
        double wp_time;
        while (ifs >> wp_time) {
            time_info.emplace_back(wp_time);
        }
        ROS_INFO("Read %d way point time", (int) time_info.size());
        return true;
    }

    void follow_main_uav::start_sub_callback(const std_msgs::EmptyConstPtr &msg) {
        start_flag = true;
        start_time = ros::Time::now();
    }


    void follow_main_uav::timer_pub_cmd_callback(const ros::TimerEvent &) {
        if (!takeoff_flag) {
            std_msgs::Empty empty_msg;
            for (int num = 0; num < sub_num; num++) {
                takeoff_pub[num].publish(empty_msg);
            }
            takeoff_flag = true;
            return;
        }
        if (!start_flag) {
            for (int num = 0; num < sub_num; num++) {
                geometry_msgs::Twist twist_msg;

                double lat = sub_wp_info[num][0].latitude;
                double lng = sub_wp_info[num][0].longitude;
                double alt = sub_wp_info[num][0].altitude;

                twist_msg.linear.x = lat;
                twist_msg.linear.y = lng;
                twist_msg.linear.z = alt;

                gps_lib::Vincenty v(lat, lng);
                auto info = v.calc_distance(main_wp_info[0].latitude, main_wp_info[0].longitude);
                twist_msg.angular.z = std::get<1>(info);
                moveTo_pub[num].publish(twist_msg);
            }
            return;
        }

        double elapsed_time = (ros::Time::now() - start_time).toSec();
        size_t wp_id = std::distance(cum_main_wp_time.begin(),
                                     std::upper_bound(cum_main_wp_time.begin(), cum_main_wp_time.end(), elapsed_time));

        if (wp_id == wp_num) {
            if(counter < 5){
                counter++;

                for (int num = 0; num < sub_num; num++) {
                    geometry_msgs::Twist twist_msg;

                    double lat = sub_wp_info[num][wp_id-1].latitude;
                    double lng = sub_wp_info[num][wp_id-1].longitude;
                    double alt = sub_wp_info[num][wp_id-1].altitude;

                    twist_msg.linear.x = lat;
                    twist_msg.linear.y = lng;
                    twist_msg.linear.z = alt;

                    gps_lib::Vincenty v(lat, lng);
                    auto info = v.calc_distance(main_wp_info[wp_id-1].latitude, main_wp_info[wp_id-1].longitude);
                    twist_msg.angular.z = std::get<1>(info);
                    moveTo_pub[num].publish(twist_msg);
                }
            }
            std_msgs::Empty empty_msg;
            for (int num = 0; num < sub_num; num++) {
                land_pub[num].publish(empty_msg);
            }
            return;
        }

        auto main_gps_target = calc_target_gps(wp_id,elapsed_time,main_wp_info);

        for(int num = 0; num < sub_num; num++){
            geometry_msgs::Twist twist_msg;

            auto sub_gps_target = calc_target_gps(wp_id,elapsed_time,sub_wp_info[num]);
            gps_lib::Vincenty v(sub_gps_target.latitude,sub_gps_target.longitude);
            auto info = v.calc_distance(main_gps_target.latitude,main_gps_target.longitude);

            twist_msg.linear.x = sub_gps_target.latitude;
            twist_msg.linear.y = sub_gps_target.longitude;
            twist_msg.linear.z = sub_gps_target.altitude;

            twist_msg.angular.z = std::get<1>(info);

            moveTo_pub[num].publish(twist_msg);


        }
    }

    gps_info follow_main_uav::calc_target_gps(int wp_id, double elapsed_time, std::vector<gps_info> &info) {
        gps_info target_gps_info;
        double diff_time = cum_main_wp_time[wp_id] - cum_main_wp_time[wp_id - 1];
        double section_time = elapsed_time - cum_main_wp_time[wp_id - 1];

        target_gps_info.latitude =  (section_time/diff_time) * (info[wp_id].latitude - info[wp_id-1].latitude) + info[wp_id-1].latitude;
        target_gps_info.longitude =  (section_time/diff_time) * (info[wp_id].longitude - info[wp_id-1].longitude) + info[wp_id-1].longitude;
        target_gps_info.altitude =  (section_time/diff_time) * (info[wp_id].altitude - info[wp_id-1].altitude) + info[wp_id-1].altitude;
        
        return target_gps_info;
    }

} // namespace anafi_controller
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(anafi_controller::follow_main_uav, nodelet::Nodelet);
