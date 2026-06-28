#ifndef IMAGE_READER_HPP
#define IMAGE_READER_HPP

#include <vector>
#include <string>
#include <map>
#include <iostream>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

class imageReader
{
    public:
        imageReader(string dir_path) : dir_path(dir_path) {}
        void datasetLoader();
        vector<vector<string>> frame_cam_names;
    private:
        int numb_cams = 2;
        string dir_path;
        int total_imgs = 0;
        int synced_imgs = 0;
};


#endif