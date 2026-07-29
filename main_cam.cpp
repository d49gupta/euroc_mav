#include "image_reader.hpp"
#include "vo.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::unique_ptr<imageReader> reader = std::make_unique<imageReader>("src/vicon_room1/vicon_room1/V1_01_easy/V1_01_easy");
    reader->datasetLoader();
    
    cout<<"First timestamp: "<<reader->frame_cam_timestamp[0][0]<<endl;

    auto stereo_tracker = std::make_shared<VisualOdometry>("stereo_tracker");
    for (size_t i = 0; i < reader->frame_cam_names.size() && rclcpp::ok(); i++)
    {
        string left_path = reader->frame_cam_names[i][0];
        string right_path = reader->frame_cam_names[i][1];
        uint64_t timestamp = reader->frame_cam_timestamp[i][0];
        stereo_tracker->processFrame(left_path, right_path, timestamp);
        // stereo_tracker->visualizeMatch(left_path, right_path);
        rclcpp::spin_some(stereo_tracker);
    }
    cout<<"Number of keyframes: "<<stereo_tracker->numb_keyframes<<" / "<<stereo_tracker->frame_id<<endl;
    cout<<"Average reprojection MAE: "<<double(stereo_tracker->avg_mae / (stereo_tracker->numb_keyframes))<<endl;
    stereo_tracker->outputPose("src/build/vo_estimate.csv");
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}