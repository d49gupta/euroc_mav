#include "image_reader.hpp"
#include "vo.hpp"

int main()
{
    std::unique_ptr<imageReader> reader = std::make_unique<imageReader>("../vicon_room1/vicon_room1/V1_01_easy/V1_01_easy");
    reader->datasetLoader();
    
    cout<<"First timestamp: "<<reader->frame_cam_timestamp[0][0]<<endl;
    std::unique_ptr<StereoTracker> tracker = std::make_unique<StereoTracker>();

    for (size_t i = 0; i < reader->frame_cam_names.size(); i++)
    {
        string left_path = reader->frame_cam_names[i][0];
        string right_path = reader->frame_cam_names[i][1];
        tracker->processFrame(left_path, right_path);
        tracker->visualizeMatch(left_path, right_path);
    }
    cv::destroyAllWindows();
    return 0;
}