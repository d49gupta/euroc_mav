#ifndef VISUAL_INERTIAL_HPP
#define VISUAL_INERTIAL_HPP

#include "vo.hpp"
#include "ahrs.hpp"
#include <deque>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <array>
#include <DBoW3/DBoW3.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#define TEMPORAL_GAP 100 // 20 hz so around 5 seconds
#define SCORE_THRESHOLD 0.05 // bounded [0, 1] where 1.0 is identical
// Run analysis on score threshold after offline pass for better dynamic threshold
#define MIN_PNP_INLIERS 15 // minimum RANSAC inliers to accept a loop-closure geometric verification
#define MIN_UPDATE_DEPTH 0.1 // reject an EKF landmark update if predicted depth is at/behind the camera (meters)
#define CHI_SQUARE_95_2DOF 5.99 // Mahalanobis gate for a 2-dof (u,v) residual at 95% confidence
#define MAX_KEYFRAME_GAP 20

using namespace std;
using namespace cv;

struct stateEstimate
{
    uint64_t timestamp = 0.0;
    Eigen::Vector3d accel_bias{0.0, 0.0, 0.0};
    Eigen::Quaterniond orient_estimate = Eigen::Quaterniond::Identity();
    Eigen::Vector3d gyro_bias{0.0, 0.0, 0.0};
    Eigen::Vector3d pos_estimate{0.0, 0.0, 0.0};
    Eigen::Vector3d vel_estimate{0.0, 0.0, 0.0};
};

struct KeyframeNode
{
    uint64_t timestamp;
    int frame_id;
    cv::Mat tvec;
    cv::Mat rvec;
    vector<cv::KeyPoint> keypoints; // keypoints and descriptors of left image
    cv::Mat descriptors;
    vector<Point3d> landmarks;
    Eigen::Matrix<double, 15, 15> P;
};

class VisualInertialOdometry : public StereoTracker
{
    public:
        VisualInertialOdometry(const string nodeName, const DatasetConfig& config) : StereoTracker(nodeName, config)
        {
            slam_thread = std::thread(&VisualInertialOdometry::slamLoop, this);
        }

        void predict(IMUSample& data, double dt); // IMU preintegration
        void update(uint64_t timestep); // replace with VIO EKF
        void outputState(string filename); // properly transform all states
        void outputTrajectory(string filename); // writes keyframe_store's (SLAM-corrected) poses

        void init_params();
        void update_covariance(IMUSample& data, double dt);
        void resetCovariance(Eigen::Matrix<double, 15, 15>& P);
        ~VisualInertialOdometry()
        {
            running = false;
            queue_cv.notify_all();
            if (slam_thread.joinable())
                slam_thread.join(); // guarantees the SLAM thread is fully done before we read keyframe_store
            outputTrajectory("src/build/slam_trajectory.csv");
            optimization_log.close();
            cout<<"Number of loops detected: "<<loop_frames.size()<<endl;
            for (auto loop : loop_frames)
                cout<<loop<<" ";
            cout<<endl;
        }
        KeyframeNode createNode();
        void enqueueKeyframe(KeyframeNode node); // thread-safe hand-off to the SLAM thread

        vector<stateEstimate> state_estimate;
        stateEstimate last_estimate;
        Eigen::Vector3d g_ref; // gravity vector at initial orientation
        Eigen::Matrix<double, 15, 15> P_keyframe; // covariance matrix that resets at keyframes
        std::vector<KeyframeNode> keyframe_store; // store keyframe nodes that match with BoW database
        vector<int> loop_frames;

    private:
        void slamLoop(); // runs on slam_thread: waits for keyframes, will host loop-closure/pose-graph work

        Eigen::Vector<double, 15> xr; // error state vector (nominal state vector held in this->state_estimate)
        Eigen::Matrix<double, 15, 15> P; // covariance matrix
        Eigen::Matrix<double, 15, 15> A; // state transition matrix (Jacobian)
        Eigen::Matrix<double, 15, 15> Q; // process noise covariance matrix
        Eigen::Matrix<double, 2, 2> R; // measurement noise covariance matrix
        Eigen::Matrix<double, 2, 15> H; // observation matrix (Jacobian)

        // SLAM thread syncronization tools
        std::deque<KeyframeNode> keyframe_queue;
        std::mutex queue_mutex;
        std::condition_variable queue_cv;
        std::atomic<bool> running{true};
        std::thread slam_thread;

        // BoW vocabulary and database
        DBoW3::Vocabulary vocab;
        DBoW3::Database db;

        // pose graph: one [rx,ry,rz,tx,ty,tz] block per keyframe, aligned by index with
        // keyframe_store; std::deque so push_back never invalidates pointers ceres::Problem holds
        ceres::Problem pose_graph;
        std::deque<std::array<double, 6>> pose_params;
};

#endif