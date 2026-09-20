#include "sim_dataset_config.hpp"

namespace
{
    cv::Mat cameraInfoToK(const sensor_msgs::msg::CameraInfo &info)
    {
        return (cv::Mat_<double>(3, 3) << info.k[0], info.k[1], info.k[2],
                                           info.k[3], info.k[4], info.k[5],
                                           info.k[6], info.k[7], info.k[8]);
    }

    cv::Mat cameraInfoToD(const sensor_msgs::msg::CameraInfo &info)
    {
        cv::Mat D(1, static_cast<int>(info.d.size()), CV_64F);
        for (size_t i = 0; i < info.d.size(); i++)
            D.at<double>(0, static_cast<int>(i)) = info.d[i];
        return D;
    }
}

DatasetConfig loadSimDatasetConfig(
    const sensor_msgs::msg::CameraInfo &left_info,
    const sensor_msgs::msg::CameraInfo &right_info,
    const nav_msgs::msg::Odometry::SharedPtr &ground_truth)
{
    DatasetConfig config;

    config.K_left = cameraInfoToK(left_info);
    config.D_left = cameraInfoToD(left_info);
    config.K_right = cameraInfoToK(right_info);
    config.D_right = cameraInfoToD(right_info);
    config.image_size = cv::Size(static_cast<int>(left_info.width), static_cast<int>(left_info.height));

    // Fixed link-frame -> optical-frame axis remap (x_opt = -y_link,
    // y_opt = -z_link, z_opt = x_link), composed with each camera's
    // (identity) mounting rotation. See header comment for why this isn't
    // just identity.
    cv::Mat R_link_to_optical = (cv::Mat_<double>(3, 3) <<
         0,  0, 1,
        -1,  0, 0,
         0, -1, 0);

    config.R_B_C0 = R_link_to_optical.clone();
    config.t_B_C0 = (cv::Mat_<double>(3, 1) << 0.1, 0.06, 0.0);

    config.R_B_C1 = R_link_to_optical.clone();
    config.t_B_C1 = (cv::Mat_<double>(3, 1) << 0.1, -0.06, 0.0);

    // Keep as 0 transformation to convert body to vicon
    config.R_B_V = cv::Mat::eye(3, 3, CV_64F);
    config.t_B_V = cv::Mat::zeros(3, 1, CV_64F);

    if (ground_truth)
    {
        // account for initial rotation and translation
        const auto &p = ground_truth->pose.pose.position;
        const auto &q = ground_truth->pose.pose.orientation;
        config.t_gt_0 = Eigen::Vector3d(p.x, p.y, p.z);
        config.q_gt0 = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
        config.t_vicon0 = config.t_gt_0;
        config.q_vicon0 = config.q_gt0;
    }
    config.init_accel_bias = Eigen::Vector3d::Zero(); // not modeled by the simulated IMU

    return config;
}
