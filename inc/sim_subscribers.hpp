#ifndef SIM_SUBSCRIBERS_HPP
#define SIM_SUBSCRIBERS_HPP

// Subscribes to every topic published by sim/worlds/euroc_mav_sim.sdf (via
// the ros_gz_bridge config in sim/config/bridge.yaml) and holds the latest
// message received on each. No fusion/estimation logic lives here -- this
// is just the data-in layer; wire these getters into vo/vio/whatever
// consumes them.
//
// The two camera topics are time-synchronized and converted to cv::Mat via
// cv_bridge, so latestStereoFrame() hands back a ready-to-use OpenCV pair
// instead of raw sensor_msgs::Image.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core.hpp>
#include <mutex>

// A time-synchronized, already-decoded stereo pair.
struct StereoFrame
{
    rclcpp::Time stamp;
    cv::Mat left;   // CV_8UC1 (mono8)
    cv::Mat right;  // CV_8UC1 (mono8)
    bool valid = false;
};

class SimSubscribers : public rclcpp::Node
{
public:
    SimSubscribers();

    // Thread-safe accessors for whatever was last received on each topic.
    // Each returns nullptr (or valid=false for the stereo frame) until at
    // least one message has arrived.
    sensor_msgs::msg::Imu::SharedPtr latestImu() const;
    sensor_msgs::msg::PointCloud2::SharedPtr latestLidarPoints() const;
    sensor_msgs::msg::LaserScan::SharedPtr latestLidarScan() const;
    StereoFrame latestStereoFrame() const;
    sensor_msgs::msg::CameraInfo::SharedPtr latestLeftCameraInfo() const;
    sensor_msgs::msg::CameraInfo::SharedPtr latestRightCameraInfo() const;
    // Ground truth: exact simulated pose/twist, published by the
    // OdometryPublisher plugin (zero noise -- not an estimate).
    nav_msgs::msg::Odometry::SharedPtr latestGroundTruth() const;

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void lidarPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void lidarScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void stereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr &left_msg,
                         const sensor_msgs::msg::Image::ConstSharedPtr &right_msg);
    void leftCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void rightCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_points_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;

    // Stereo images are subscribed via message_filters so the left/right
    // callbacks fire together as one matched pair instead of independently.
    using ImageSubscriber = message_filters::Subscriber<sensor_msgs::msg::Image>;
    using StereoSyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    ImageSubscriber left_image_sub_;
    ImageSubscriber right_image_sub_;
    std::shared_ptr<message_filters::Synchronizer<StereoSyncPolicy>> stereo_sync_;

    mutable std::mutex mutex_;
    sensor_msgs::msg::Imu::SharedPtr latest_imu_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_lidar_points_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_lidar_scan_;
    StereoFrame latest_stereo_frame_;
    sensor_msgs::msg::CameraInfo::SharedPtr latest_left_info_;
    sensor_msgs::msg::CameraInfo::SharedPtr latest_right_info_;
    nav_msgs::msg::Odometry::SharedPtr latest_ground_truth_;
};

#endif
