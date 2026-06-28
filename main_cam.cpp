#include "image_reader.hpp"

int main()
{
    std::unique_ptr<imageReader> reader = std::make_unique<imageReader>("../vicon_room1/vicon_room1/V1_01_easy/V1_01_easy");
    reader->datasetLoader();
    
    return 0;
}