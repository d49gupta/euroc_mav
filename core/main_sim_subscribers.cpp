#include "sim_subscribers.hpp"
#include "sim_dataset_config.hpp"
#include "vo.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>

using namespace std::chrono_literals;
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimSubscribers>();
    rclcpp::Rate wait_rate(20ms);
    const auto wait_start = node->now();
    const rclcpp::Duration wait_timeout = rclcpp::Duration::from_seconds(10.0);

    while (rclcpp::ok() &&
           (!node->latestLeftCameraInfo() || !node->latestRightCameraInfo()))
    {
        rclcpp::spin_some(node); // one non blocking pass over any callbacks ready for that node
        if (node->now() - wait_start > wait_timeout)
        {
            RCLCPP_ERROR(node->get_logger(),
                "Timed out waiting for stereo/left|right camera_info -- is the sim running?");
            rclcpp::shutdown();
            return 1;
        }
        wait_rate.sleep();
    }

    rclcpp::spin_some(node);
    auto ground_truth = node->latestGroundTruth();

    DatasetConfig config = loadSimDatasetConfig(
        *node->latestLeftCameraInfo(), *node->latestRightCameraInfo(), ground_truth);

    RCLCPP_INFO(node->get_logger(),
        "DatasetConfig built from sim: image_size=%dx%d, ground_truth=%s",
        config.image_size.width, config.image_size.height,
        ground_truth ? "seeded" : "not yet available, defaulted to identity/zero");

    auto tracker = std::make_shared<StereoTracker>("sim_stereo_tracker", config);

    std::ofstream ground_truth_csv("src/build/sim_ground_truth.csv");
    ground_truth_csv << "timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z\n";
    ground_truth_csv << std::fixed << std::setprecision(6);
    int64_t last_gt_ns = -1;

    int64_t last_frame_ns = -1;
    rclcpp::Rate poll_rate(10ms); // faster than the sim's camera rate so no frame is missed
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        StereoFrame frame = node->latestStereoFrame();
        if (frame.valid && frame.stamp.nanoseconds() != last_frame_ns)
        {
            tracker->processFrame(frame.left, frame.right, static_cast<uint64_t>(frame.stamp.nanoseconds()));
            last_frame_ns = frame.stamp.nanoseconds();
        }

        auto gt = node->latestGroundTruth();
        if (gt)
        {
            int64_t gt_ns = static_cast<int64_t>(gt->header.stamp.sec) * 1000000000LL + gt->header.stamp.nanosec;
            if (gt_ns != last_gt_ns)
            {
                const auto &p = gt->pose.pose.position;
                const auto &q = gt->pose.pose.orientation;
                ground_truth_csv << gt_ns << "," << p.x << "," << p.y << "," << p.z << ","
                                  << q.w << "," << q.x << "," << q.y << "," << q.z << "\n";
                last_gt_ns = gt_ns;
            }
        }
        poll_rate.sleep();
    }

    tracker->outputPose("src/build/sim_vo_estimate.csv");
    ground_truth_csv.close();
    rclcpp::shutdown();
    return 0;
}
