#ifndef SIM_DATASET_CONFIG_HPP
#define SIM_DATASET_CONFIG_HPP

// Builds a DatasetConfig (see dataset_config.hpp) from the Gazebo sim rig
// (sim/worlds/euroc_mav_sim.sdf) instead of an on-disk EuRoC sequence, so
// whatever VO/VIO code consumes DatasetConfig can run against live sim
// data without caring where it came from.
//
// left_info/right_info: from SimSubscribers::latestLeftCameraInfo() /
// latestRightCameraInfo() -- K/D/image_size are read from these directly
// since they're live and don't need to be hardcoded.
//
// ground_truth: optional (pass nullptr to skip); if given, seeds
// q_gt0/t_gt_0 (and q_vicon0/t_vicon0, since the sim only has one true
// ground-truth source, not a separate marker frame like EuRoC's vicon0).
// init_accel_bias is left at zero -- the simulated IMU doesn't model bias.
//
// R_B_C0/t_B_C0 and R_B_C1/t_B_C1 are hardcoded from the rig's known,
// fixed geometry (see the SDF: cameras at (0.1, +/-0.06, 0) on base_link,
// both with identity rotation in gz-sim's link-frame convention). Two
// things worth knowing about these numbers:
//
//   1. They're static, same as EuRoC's T_BS -- the rig is rigid, so this
//      doesn't change frame to frame.
//   2. The rotation is NOT identity, even though the SDF pose rotation is.
//      gz-sim/SDF poses use the link convention (+x forward, +y left,
//      +z up), but DatasetConfig's R_B_C (like EuRoC's T_BS) is expressed
//      in the standard camera *optical* frame (+z forward, +x right,
//      +y down) that OpenCV-style projection assumes. The two differ by a
//      fixed 90-degree axis remap; R_B_C0/R_B_C1 already include it below,
//      the translations don't need it.

#include "dataset_config.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>

DatasetConfig loadSimDatasetConfig(
    const sensor_msgs::msg::CameraInfo &left_info,
    const sensor_msgs::msg::CameraInfo &right_info,
    const nav_msgs::msg::Odometry::SharedPtr &ground_truth = nullptr);

#endif
