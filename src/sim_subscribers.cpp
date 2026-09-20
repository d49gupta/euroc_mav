#include "sim_subscribers.hpp"
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;
using std::placeholders::_2;

SimSubscribers::SimSubscribers() : rclcpp::Node("sim_subscribers")
{
    // sim/launch/sim.launch.py bridges everything at world/sim rate with
    // use_sim_time set on the bridge node; set it here too so this node's
    // clock/throttled logging line up with the simulation, not wall time.
    // this->declare_parameter<bool>("use_sim_time", true);

    auto sensor_qos = rclcpp::SensorDataQoS();

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", sensor_qos, std::bind(&SimSubscribers::imuCallback, this, _1));

    lidar_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "lidar/points", sensor_qos, std::bind(&SimSubscribers::lidarPointsCallback, this, _1));

    lidar_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "lidar/scan", sensor_qos, std::bind(&SimSubscribers::lidarScanCallback, this, _1));

    left_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "stereo/left/camera_info", sensor_qos,
        std::bind(&SimSubscribers::leftCameraInfoCallback, this, _1));

    right_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "stereo/right/camera_info", sensor_qos,
        std::bind(&SimSubscribers::rightCameraInfoCallback, this, _1));

    // Odometry benefits from a bit more history than the default depth-10
    // sensor QoS since it's small and you don't want to drop ground truth.
    ground_truth_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "ground_truth/odom", rclcpp::QoS(50), std::bind(&SimSubscribers::groundTruthCallback, this, _1));

    // Stereo images: subscribed through message_filters and synchronized so
    // stereoCallback only fires once a matched left/right pair is available,
    // instead of two independent callbacks you'd have to pair up yourself.
    left_image_sub_.subscribe(this, "stereo/left/image_raw", rmw_qos_profile_sensor_data);
    right_image_sub_.subscribe(this, "stereo/right/image_raw", rmw_qos_profile_sensor_data);
    stereo_sync_ = std::make_shared<message_filters::Synchronizer<StereoSyncPolicy>>(
        StereoSyncPolicy(10), left_image_sub_, right_image_sub_);
    stereo_sync_->registerCallback(std::bind(&SimSubscribers::stereoCallback, this, _1, _2));

    RCLCPP_INFO(this->get_logger(),
        "sim_subscribers up: imu, lidar/points, lidar/scan, "
        "synced stereo/{left,right}/{image_raw,camera_info}, ground_truth/odom");
}

void SimSubscribers::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_imu_ = msg;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "imu: accel=(%.3f, %.3f, %.3f) gyro=(%.3f, %.3f, %.3f)",
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
        msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
}

void SimSubscribers::lidarPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_lidar_points_ = msg;
    }
    const size_t num_points = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "lidar/points: %zu points, frame_id=%s", num_points, msg->header.frame_id.c_str());
}

void SimSubscribers::lidarScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_lidar_scan_ = msg;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "lidar/scan: %zu ranges", msg->ranges.size());
}

void SimSubscribers::stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
                                     const sensor_msgs::msg::Image::ConstSharedPtr &right_msg)
{
    StereoFrame frame;
    frame.stamp = left_msg->header.stamp;
    try
    {
        // toCvCopy (not toCvShare) so the cv::Mat owns its data and stays
        // valid after this callback returns / the source message is freed.
        frame.left = cv_bridge::toCvCopy(left_msg, "mono8")->image;
        frame.right = cv_bridge::toCvCopy(right_msg, "mono8")->image;
        frame.valid = true;
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion failed: %s", e.what());
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_stereo_frame_ = frame;
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "stereo pair: %dx%d, dt(left-right)=%.4f s",
        frame.left.cols, frame.left.rows,
        (rclcpp::Time(left_msg->header.stamp) - rclcpp::Time(right_msg->header.stamp)).seconds());
}

void SimSubscribers::leftCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_left_info_ = msg;
}

void SimSubscribers::rightCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    latest_right_info_ = msg;
}

void SimSubscribers::groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ground_truth_ = msg;
    }
    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "ground_truth/odom: pos=(%.3f, %.3f, %.3f) quat=(w=%.3f, x=%.3f, y=%.3f, z=%.3f)",
        p.x, p.y, p.z, q.w, q.x, q.y, q.z);
}

sensor_msgs::msg::Imu::SharedPtr SimSubscribers::latestImu() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_imu_;
}

sensor_msgs::msg::PointCloud2::SharedPtr SimSubscribers::latestLidarPoints() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_lidar_points_;
}

sensor_msgs::msg::LaserScan::SharedPtr SimSubscribers::latestLidarScan() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_lidar_scan_;
}

StereoFrame SimSubscribers::latestStereoFrame() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_stereo_frame_;
}

sensor_msgs::msg::CameraInfo::SharedPtr SimSubscribers::latestLeftCameraInfo() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_left_info_;
}

sensor_msgs::msg::CameraInfo::SharedPtr SimSubscribers::latestRightCameraInfo() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_right_info_;
}

nav_msgs::msg::Odometry::SharedPtr SimSubscribers::latestGroundTruth() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_ground_truth_;
}
