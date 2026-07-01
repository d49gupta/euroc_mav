#include "image_reader.hpp"
#include "vo.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::unique_ptr<imageReader> reader = std::make_unique<imageReader>("src/vicon_room1/vicon_room1/V1_01_easy/V1_01_easy");
    reader->datasetLoader();
    
    cout<<"First timestamp: "<<reader->frame_cam_timestamp[0][0]<<endl;
    auto stereo_tracker = std::make_shared<StereoTracker>("stereo_tracker");

    for (size_t i = 0; i < reader->frame_cam_names.size(); i++)
    {
        string left_path = reader->frame_cam_names[i][0];
        string right_path = reader->frame_cam_names[i][1];
        stereo_tracker->processFrame(left_path, right_path);
        // stereo_tracker->visualizeMatch(left_path, right_path);
        stereo_tracker->publishPointCloud();
    }
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}