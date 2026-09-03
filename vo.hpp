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
#include "dataset_config.hpp"

#define FRAME_THRESH 5
#define REPROJ_ERR 5
#define FEATURE_THRESH 0.5
#define OBSERVATION_THRESH 5
#define MAX_OBSERVATIONS 20
#define MIN_PARALLAX_APPEND_RAD 0.0034906585 // 0.2 degs
#define MIN_PARALLAX_REFINE_RAD 0.0174532925 // 1 degree
#define MAX_STEP_DIST 1.25
#define MAX_KEYFRAME_GAP 20

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
    protected:
        cv::Mat prev_left_img;
        vector<Point2f> prev_left_feature_pts;
        vector<int> prev_feature_ids;
        int next_id = 0;
        vector<Feature> current_matches;
        unordered_map<int, Landmark> global_map;

        vector<int> last_inlier_ids;
        vector<Point2f> last_inlier_pts;

        const double b = 0.1101; // baseline in meters
        const int max_features = 150;
        const Size window_size = Size(21, 21);
        const int max_level = 3;
        const cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

        // Camera intrinsics/distortion, loaded from DatasetConfig (cam0/cam1 sensor.yaml)
        Mat K_left, D_left, K_right, D_right;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
        nav_msgs::msg::Path path;
        unordered_map<int, Pose> camera_pose;

        // Extrinsic matrix from left camera to IMU body frame
        cv::Mat R_B_C0, t_B_C0;
        Eigen::Matrix3d R_B_C0_eigen;
        Eigen::Vector3d t_B_C0_eigen;

        // Extrinsic matrix from right camera to IMU body frame
        cv::Mat R_B_C1, t_B_C1;

        // Inverted extrinsics left camera -> IMU body frame
        cv::Mat R_C_B0, t_C_B0;
        Eigen::Matrix3d R_C_B0_eigen;
        Eigen::Vector3d t_C_B0_eigen;

        // Transformation matrix: left camera -> world transformation
        cv::Mat R_W_C;
        cv::Mat t_W_C;

        // Transformation from left->right camera
        cv::Mat R_left_right, t_left_right;
        cv::Mat R1, R2, P1, P2, Q;
        cv::Size image_size;

        // Extrinsic matrix from vicon marker to IMU body frame (T_BS in vicon0/sensor.yaml)
        Eigen::Matrix3d R_B_V;
        Eigen::Vector3d t_B_V;

        // Ground truth vicon marker pose at t=0 (vicon0/data.csv row closest to the first camera frame)
        // Maps vicon marker -> room (ground truth)
        Eigen::Quaterniond q_vicon0;
        Eigen::Vector3d t_vicon0;

        // Ground truth state estimate at t=0
        Eigen::Quaterniond q_gt0;
        Eigen::Vector3d t_gt_0;

        // World(VO) -> Vicon room-frame alignment, derived in the constructor from q_vicon0/t_vicon0
        Eigen::Matrix3d R_init;
        Eigen::Vector3d t_init;

        // World->Camera T, used in solvePnPRansac
        cv::Mat rvec, tvec;
        std::ofstream optimization_log;
        Eigen::Quaterniond prev_q_W_C = Eigen::Quaterniond::Identity();

        cv::Ptr<cv::ORB> orb = cv::ORB::create();

    public:
        StereoTracker(string nodeName, const DatasetConfig& config);
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
        bool keyframe;
        int last_keyframe_frame_id = 0;
};

#endif