#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

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



int main(int argc, char **argv) {
    //Initialize ROS, name node track_tag
    ros::init(argc, argv, "moveTo_test");
    //create the NodeHandle
    ros::NodeHandle nh("~");

    ros::Rate rate(1);

    ros::Publisher takeoff_pub = nh.advertise<std_msgs::Empty>("/anafi/takeoff", 1);
    ros::Publisher land_pub = nh.advertise<std_msgs::Empty>("/anafi/land", 1);
    ros::Publisher moveTo_pub = nh.advertise<geometry_msgs::Twist>("/anafi/gps_moveto", 1);

    double home_lati = 48.8789000031;
    double home_long = 2.36778003052;
    double home_alti = 1.00026154518;


    double target_lati = 48.8789127653;
    double target_long = 2.36772056226;
    double target_alti = 1.70950913429;

    double main_lati = 48.878876834;
    double main_long = 2.36775697536;
    double main_alti = 1.49990868568;

    sleep(3);
    ROS_INFO("start");
    std_msgs::Empty dummy_msg;
    takeoff_pub.publish(dummy_msg);
    ROS_INFO("pub takeoff");
    sleep(10);

    int counter = 0;
    int MAX_counter = 100;

    // goto home
    {
        ros::Time nowt = ros::Time::now();

        double now_lati = home_lati + (target_lati - home_lati) * counter / (double) MAX_counter;
        double now_long = home_long + (target_long - home_long) * counter / (double) MAX_counter;
        double now_alti = home_alti + (target_alti - home_alti) * counter / (double) MAX_counter;


        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = now_lati;
        twist_msg.linear.y = now_long;
        twist_msg.linear.z = now_alti;

        gps_lib::Vincenty v(now_lati,now_long);
        auto info = v.calc_distance(main_lati,main_long);
        twist_msg.angular.z = std::get<1>(info);

        moveTo_pub.publish(twist_msg);
        counter++;
        ROS_INFO("goto home");
        sleep(10);
    }

    while (ros::ok()) {
        if (counter > MAX_counter) break;
        ROS_INFO("count : %d , [s]", counter);

        double now_lati = home_lati + (target_lati - home_lati) * counter / (double) MAX_counter;
        double now_long = home_long + (target_long - home_long) * counter / (double) MAX_counter;
        double now_alti = home_alti + (target_alti - home_alti) * counter / (double) MAX_counter;


        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = now_lati;
        twist_msg.linear.y = now_long;
        twist_msg.linear.z = now_alti;

        gps_lib::Vincenty v(now_lati,now_long);
        auto info = v.calc_distance(main_lati,main_long);
        twist_msg.angular.z = std::get<1>(info);

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