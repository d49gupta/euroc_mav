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
#define MIN_PARALLAX_APPEND_RAD 0.0034906585 // 0.2 degs
#define MIN_PARALLAX_REFINE_RAD 0.0174532925 // 1 degree

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
    int last_optimized_frame = -1;
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

        Eigen::Matrix3d R_C_B0_eigen = (Eigen::Matrix3d() <<
            R_C_B0.at<double>(0,0), R_C_B0.at<double>(0,1), R_C_B0.at<double>(0,2),
            R_C_B0.at<double>(1,0), R_C_B0.at<double>(1,1), R_C_B0.at<double>(1,2),
            R_C_B0.at<double>(2,0), R_C_B0.at<double>(2,1), R_C_B0.at<double>(2,2)).finished();
        Eigen::Vector3d t_C_B0_eigen{t_C_B0.at<double>(0), t_C_B0.at<double>(1), t_C_B0.at<double>(2)};

        // Transformation matrix: left camera -> world transformation
        cv::Mat R_W_C;
        cv::Mat t_W_C;

        // Transformation from left->right camera
        cv::Mat R_left_right = R_B_C1.t() * R_B_C0;
        cv::Mat t_left_right = R_B_C1.t() * (t_B_C0 - t_B_C1);
        cv::Mat R1, R2, P1, P2, Q;
        cv::Size image_size{752, 480};

        // Transformation from body->vicon frame
        cv::Mat R_V_B = (cv::Mat_<double>(3, 3) <<
            0.33638, -0.02078,  0.94150,
            -0.01749, -0.99972, -0.01582,
            0.94156, -0.01114, -0.33665);

        cv::Mat t_V_B = (cv::Mat_<double>(3, 1) <<
            0.092907, -0.028556, -0.107015);

        Eigen::Matrix3d R_V_B_eigen = (Eigen::Matrix3d() <<
            R_V_B.at<double>(0,0), R_V_B.at<double>(0,1), R_V_B.at<double>(0,2),
            R_V_B.at<double>(1,0), R_V_B.at<double>(1,1), R_V_B.at<double>(1,2),
            R_V_B.at<double>(2,0), R_V_B.at<double>(2,1), R_V_B.at<double>(2,2)).finished();
        Eigen::Quaterniond q_V_B{R_V_B_eigen};

        // Forward T_BS (vicon marker -> body), recovered by inverting R_V_B/t_V_B
        Eigen::Matrix3d R_B_V = R_V_B_eigen.transpose();
        Eigen::Vector3d t_V_B_eigen{t_V_B.at<double>(0), t_V_B.at<double>(1), t_V_B.at<double>(2)};
        Eigen::Vector3d t_B_V = -R_B_V * t_V_B_eigen;

        // Ground truth vicon marker pose at t=0 (vicon0/data.csv, timestamp 1403715273265220000)
        // Maps vicon marker -> room (ground truth)
        Eigen::Quaterniond q_gt0{0.993271232, -0.008876672, 0.022388759, 0.113279336};
        Eigen::Vector3d t_gt0{0.786659422, 2.176692611, 1.061934815};

        // World(VO) -> Vicon room-frame alignment, derived in the constructor from q_gt0/t_gt0
        Eigen::Matrix3d R_init;
        Eigen::Vector3d t_init;

        // World->Camera T, used in solvePnPRansac
        cv::Mat rvec, tvec;
        std::ofstream optimization_log;

    public:
        StereoTracker(string nodeName);
        void processFrame(string left_img_path, string right_img_path, uint64_t timestep);
        void visualizeMatch(string left_img_path, string right_img_path);
        void triangulate(cv::Mat& left_img, cv::Mat& right_img, vector<Point2f>& feature_pts, const int numb_features, const cv::Mat& mask, 
                        vector<cv::Point2f>& added_left_pts, vector<int>& added_ids);
        void publishMap();
        void getPose(uint64_t timestep);
        void outputPose(string filename);
        double reprojectionError(string left_img_path, const vector<Point2f> curr_left_feature_pts, const vector<int> curr_feature_ids);
        double reprojectionError(const vector<Point2f> curr_left_feature_pts, const vector<int> curr_feature_ids);
        void optimizeLandmark();
        ~StereoTracker()
        {
            optimization_log.close();
        }
        double computeError(const cv::Point3d& pos, vector<Observation>& observations);
        double computeParallax(const cv::Point3d& pos, const Observation& obs1, const Observation& obs2);

        int frame_id = 0;
        int numb_keyframes = 0;
        double avg_mae = 0;
};

#endif