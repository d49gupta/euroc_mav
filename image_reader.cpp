#include "image_reader.hpp"

void imageReader::datasetLoader()
{
    string dataset_prefix = this->dir_path + "/mav0/cam";

    map<unsigned long long, int> synced_time;
    unordered_map<unsigned long long, string> timestamp_str;
    for (int cam = 0; cam < numb_cams; cam++)
    {
        vector<string> names;
        cv::glob(dataset_prefix + to_string(cam) + "/data/*.png", names);
        total_imgs = std::max(total_imgs, static_cast<int>(names.size()));
        for (const auto &name : names)
        {
            const auto p0 = name.find_last_of('/');
            const auto p1 = name.find_last_of('.');
            const string ts_str = name.substr(p0 + 1, p1 - p0 - 1);
            unsigned long long time_stamp = stoul(ts_str);
            
            synced_time[time_stamp] += 1; 
            if (synced_time[time_stamp] == 1) 
                timestamp_str[time_stamp] = ts_str;
        }
    }

    vector<vector<string>> cam_i_names(numb_cams);
    vector<vector<unsigned long long>> cam_i_timestamp(numb_cams);
    for (const auto &ts : synced_time)
    {
        if (ts.second == numb_cams)
        {
            for (int cam = 0; cam < numb_cams; cam++)
            {
                const string image_path = dataset_prefix + to_string(cam) + "/data/" + timestamp_str.at(ts.first) + ".png";
                cam_i_names[cam].push_back(image_path); // [camera_number][image_path]
                cam_i_timestamp[cam].push_back(ts.first); // [camera_number][timestamp]
            }
        }
    }
    
    synced_imgs = cam_i_names[0].size();
    frame_cam_names.resize(synced_imgs);
    frame_cam_timestamp.resize(synced_imgs);
    for (size_t frame = 0; frame < frame_cam_names.size(); frame++)
    {
        vector<string> cam_name_in_frame(cam_i_names.size());
        vector<unsigned long long>  cam_timestamp_in_frame(cam_i_names.size());
        for (size_t cam = 0; cam < cam_i_names.size(); cam++)
        {
            cam_name_in_frame[cam] = cam_i_names[cam][frame];
            cam_timestamp_in_frame[cam] = cam_i_timestamp[0][frame];
        }
        
        frame_cam_names[frame] = cam_name_in_frame; //[frame][camera_i] = path
        frame_cam_timestamp[frame] = cam_timestamp_in_frame; //[frame][camera_i] = timestamp
    }

    cout<< "Total number of frames: " << total_imgs << endl;
    cout<< "Total synced frames: " << synced_imgs << endl;
}