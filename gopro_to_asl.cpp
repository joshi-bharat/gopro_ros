//
// Created by bjoshi on 10/29/20.
//

#include "VideoExtractor.h"
#include "ImuExtractor.h"

#include "color_codes.h"

#include <experimental/filesystem>
#include <iostream>

using namespace std;

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        cout << RED << " Must provide video_file and base directory as input" << RESET << endl;
        return -1;
    }

    string video_file(argv[1]);
    string base_dir(argv[2]);
    int height = 540, width = 960;

    string image_folder = base_dir + "/mav0/cam0";

    if (!experimental::filesystem::is_directory(image_folder))
    {
        experimental::filesystem::create_directories(image_folder);
    }

    GoProVideoExtractor video_extractor(video_file);
    // video_extractor.extract_frames(image_folder, width, height);

    string imu_folder = base_dir + "/mav0/imu0";
    if (!experimental::filesystem::is_directory(imu_folder))
    {
        experimental::filesystem::create_directories(imu_folder);
    }

    string imu_file = imu_folder + "/data.csv";
    GoProIMUExtractor imu_extractor(video_file);
    imu_extractor.display_video_framerate();
    imu_extractor.show_gpmf_structure();
    imu_extractor.save_imu_stream(imu_file);
    return 0;
}