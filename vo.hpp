#ifndef VISUAL_ODOM_HPP
#define VISUAL_ODOM_HPP

#include <vector>
#include <string>
#include <map>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace std;
using namespace cv;

struct Feature
{
    int id;
    Point2f left_pt;
    Point2f right_pt;
};

class StereoTracker : public rclcpp::Node
{
    private:
        cv::Mat prev_left_img;
        vector<Point2f> prev_left_feature_pts;
        vector<int> prev_feature_ids;
        int next_id = 0;
        vector<Feature> current_matches;
        unordered_map<int, cv::Point3d> global_map;

        double b = 0.1101; // baseline in meters

        const int max_features = 150;
        const Size window_size = Size(21, 21);
        const int max_level = 3;
        const cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

        Mat K_left = (cv::Mat_<double>(3, 3) << 458.654,  0, 367.215,
                                                0, 457.296, 367.215,
                                                0,  0,  1); // left intrinsic matrix

        vector<double> D_left = {-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05}; // left distortion coefficients
        Mat K_right = (cv::Mat_<double>(3, 3) << 458.654,  0, 367.215,
                                                0, 457.296, 248.375,
                                                0,  0,  1);

        vector<double> D_right = {-0.28368365,  0.07451284, -0.00010473, -3.55590700e-05};
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub;
        
    public:
        StereoTracker(string nodeName);
        void processFrame(string left_img_path, string right_img_path);
        void visualizeMatch(string left_img_path, string right_img_path);
        void triangulate(cv::Mat& left_img, cv::Mat& right_img, vector<Point2f>& feature_pts, const int numb_features, const cv::Mat& mask);
        void publishPointCloud();
};

#endif