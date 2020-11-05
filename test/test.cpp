#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>

cv::Mat hsv_img;
std::vector<std::pair<cv::Scalar, cv::Scalar>> mask_rules{
        {cv::Scalar(150, 64, 0), cv::Scalar(180, 255, 255)},  // 赤
        {cv::Scalar(90, 65, 0),  cv::Scalar(150, 255, 255)},   // 青
};

// 計測用マクロ
#define MEASUREMENT(func_name, sz, loop) \
    measurement(#func_name, func_name, sz, loop);

void measurement(const std::string func_name,
                 void func(cv::Mat &), cv::Size sz, int loop) {
    cv::Mat image = cv::Mat::zeros(sz, CV_8UC3);
    double f = 1000.0f / cv::getTickFrequency();
    int64 start = cv::getTickCount();
    for (int i = 0; i < loop; i++) {
        func(image);
    }
    int64 end = cv::getTickCount();
    std::cout << func_name << ": " << (end - start) * f << "[ms]" << std::endl;
}

void v1(cv::Mat &img) {
    cv::Mat maskMat = cv::Mat::zeros(img.size(), CV_8UC1);
    for (auto mask_rule:mask_rules) {
        cv::Mat tmpMat;
        cv::inRange(hsv_img, mask_rule.first, mask_rule.second, tmpMat);
        cv::bitwise_or(maskMat, tmpMat, maskMat);
    }
    cv::Mat res;
    cv::bitwise_and(img, img, res, maskMat);
    img = res;
}

void v2(cv::Mat &img) {
    for (int y = 0; y < img.rows; y++) {
        cv::Vec3b *img_ptr = img.ptr<cv::Vec3b>(y);
        cv::Vec3b *hsv_ptr = hsv_img.ptr<cv::Vec3b>(y);

        for (int x = 0; x < img.cols; x++) {
            bool masked = false;

            uchar h = (*hsv_ptr)[0];
            uchar s = (*hsv_ptr)[1];
            uchar v = (*hsv_ptr)[2];
            for (auto mask_rule:mask_rules) {
                if (mask_rule.first[0] <= h && h <= mask_rule.second[0] &&
                    mask_rule.first[1] <= s && s <= mask_rule.second[1] &&
                    mask_rule.first[2] <= v && v <= mask_rule.second[2]) {
                    masked = true;
                    break;
                }
            }


            if (!masked) {
                (*img_ptr)[0] = 0;
                (*img_ptr)[1] = 0;
                (*img_ptr)[2] = 0;
            }

            img_ptr++;
            hsv_ptr++;
        }
    }
}


int main() {
    std::string file_name = "../images/Mandrill.bmp";
    cv::Mat img = cv::imread(file_name, cv::IMREAD_UNCHANGED);
    cv::imshow("origin", img);

    cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);

    int loop = 100;
    std::cout << "v1" << std::endl;
    MEASUREMENT(v1, img.size(), loop);

    std::cout << "v2" << std::endl;
    MEASUREMENT(v2, img.size(), loop);


    return 0;
}