#ifndef DATASET_CONFIG_HPP
#define DATASET_CONFIG_HPP

#include <string>
#include <cstdint>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

struct DatasetConfig
{
    // Camera intrinsics/distortion + shared resolution (cam0/cam1 sensor.yaml)
    cv::Mat K_left, D_left, K_right, D_right;
    cv::Size image_size;

    // Sensor -> body extrinsics (T_BS) for each camera and the vicon marker
    cv::Mat R_B_C0, t_B_C0;
    cv::Mat R_B_C1, t_B_C1;
    cv::Mat R_B_V, t_B_V;

    // Ground truth vicon marker pose (vicon0/data.csv), row closest to the first camera frame
    Eigen::Quaterniond q_vicon0;
    Eigen::Vector3d t_vicon0;

    // Ground truth body pose + accel bias (state_groundtruth_estimate0/data.csv), row closest to the first camera frame
    Eigen::Quaterniond q_gt0;
    Eigen::Vector3d t_gt_0;
    Eigen::Vector3d init_accel_bias;
};

// dataset_root is the EuRoC sequence directory containing mav0/ (e.g. ".../V1_01_easy/V1_01_easy");
// first_cam_timestamp anchors the ground-truth rows to whichever frame processing actually starts on
DatasetConfig loadDatasetConfig(const string& dataset_root, uint64_t first_cam_timestamp);

#endif
