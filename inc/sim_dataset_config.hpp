#ifndef SIM_DATASET_CONFIG_HPP
#define SIM_DATASET_CONFIG_HPP

#include "dataset_config.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>

DatasetConfig loadSimDatasetConfig(
    const sensor_msgs::msg::CameraInfo &left_info,
    const sensor_msgs::msg::CameraInfo &right_info,
    const nav_msgs::msg::Odometry::SharedPtr &ground_truth = nullptr);

#endif
