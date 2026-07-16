#ifndef VISUAL_ODOM_HPP
#define VISUAL_ODOM_HPP

#include <vector>
#include <string>
#include <map>
#include <iostream>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/path.hpp>

#define FRAME_THRESH 5
#define REPROJ_ERR 5
#define FEATURE_THRESH 0.5
#define OBSERVATION_THRESH 5
#define MAX_OBSERVATIONS 15

using namespace std;
using namespace cv;

struct Feature
{
    int id;
    Point2f left_pt;
    Point2f right_pt;
};

struct Observation
{
    int frame_id;
    Point2f frame_location;
};

struct Landmark
{
    Point3d pos;
    int outlier_count = 0;
    int reprojection_count = 0;
    vector<Observation> observations;
};

struct Pose
{
    cv::Mat rotation;
    cv::Mat translation;
};

class StereoTracker : public rclcpp::Node
{
    private:
        cv::Mat prev_left_img;
        vector<Point2f> prev_left_feature_pts;
        vector<int> prev_feature_ids;
        int next_id = 0;
        vector<Feature> current_matches;
        unordered_map<int, Landmark> global_map;

        const double b = 0.1101; // baseline in meters
        const int max_features = 150;
        const Size window_size = Size(21, 21);
        const int max_level = 3;
        const cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

        Mat K_left = (cv::Mat_<double>(3, 3) << 458.654,  0, 367.215,
                                                0, 457.296, 248.375,
                                                0,  0,  1); // left intrinsic matrix

        cv::Mat D_left = (cv::Mat_<double>(1, 4) << -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05); // left distortion coefficients
        Mat K_right = (cv::Mat_<double>(3, 3) << 457.587,  0, 379.999,
                                                0, 456.134, 255.238,
                                                0,  0,  1);

        cv::Mat D_right = (cv::Mat_<double>(1, 4) << -0.28368365,  0.07451284, -0.00010473, -3.55590700e-05);
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
        nav_msgs::msg::Path path;
        unordered_map<int, Pose> camera_pose;

        // Extrinsic matrix from left camera to IMU body frame
        cv::Mat R_B_C0 = (cv::Mat_<double>(3, 3) << 0.0148655429818, -0.999880929698,  0.00414029679422,
                                                    0.999557249008,   0.0149672133247,  0.025715529948,
                                                    -0.0257744366974,  0.00375618835797, 0.999660727178);
        cv::Mat t_B_C0 = (cv::Mat_<double>(3, 1) << -0.0216401454975, -0.064676986768, 0.00981073058949);

        // Extrinsic matrix from right camera to IMU body frame
        cv::Mat R_B_C1 = (cv::Mat_<double>(3, 3) << 0.0125552670891, -0.999755099723,  0.0182237714554,
                                                    0.999598781151,   0.0130119051815, 0.0251588363115,
                                                    -0.0253898008918,  0.0179005838253, 0.999517347078);
        cv::Mat t_B_C1 = (cv::Mat_<double>(3, 1) << -0.0198435579556, 0.0453689425024, 0.00786212447038);

        // Inverted extrinsics left camera -> IMU body frame
        cv::Mat R_C_B0 = R_B_C0.t();
        cv::Mat t_C_B0 = -R_B_C0.t() * t_B_C0;

        // Transformation matrix: left camera -> world transformation
        cv::Mat R_W_C;
        cv::Mat t_W_C;

        // Transformation from left->right camera
        cv::Mat R_left_right = R_B_C1.t() * R_B_C0;
        cv::Mat t_left_right = R_B_C1.t() * (t_B_C0 - t_B_C1);
        cv::Mat R1, R2, P1, P2, Q;
        cv::Size image_size{752, 480};

        // Initial orientation  and translation to align Vicon Frame with world frame
        Eigen::Quaterniond q_init{0.993217, -0.00929, 0.022669, 0.113664};
        Eigen::Matrix3d R_init = q_init.toRotationMatrix();
        Eigen::Vector3d t_init{0.786802375, 2.176626308, 1.06203836};

        // World->Camera T, used in solvePnPRansac
        cv::Mat rvec, tvec;
        std::ofstream optimization_log;

    public:
        StereoTracker(string nodeName);
        void processFrame(string left_img_path, string right_img_path);
        void visualizeMatch(string left_img_path, string right_img_path);
        void triangulate(cv::Mat& left_img, cv::Mat& right_img, vector<Point2f>& feature_pts, const int numb_features, const cv::Mat& mask, 
                        vector<cv::Point2f>& added_left_pts, vector<int>& added_ids);
        void publishMap();
        void getPose();
        void outputPose(string filename);
        double reprojectionError(string left_img_path, const vector<Point2f> curr_left_feature_pts, const vector<int> curr_feature_ids);
        double reprojectionError(const vector<Point2f> curr_left_feature_pts, const vector<int> curr_feature_ids);
        void optimizeLandmark();
        ~StereoTracker()
        {
            optimization_log.close();
        }

        int frame_id = -1;
        int numb_keyframes = 0;
        double avg_mae = 0;
};

#endif