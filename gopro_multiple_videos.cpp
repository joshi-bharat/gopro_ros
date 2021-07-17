//
// Created by bjoshi on 10/29/20.
//

#include "VideoExtractor.h"
#include "ImuExtractor.h"

#include "color_codes.h"

#include <experimental/filesystem>
#include <iostream>

#include <ros/ros.h>

using namespace std;
namespace fs = std::experimental::filesystem;

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "gopro");
    ros::NodeHandle nh_private("~");

    string gopro_folder;
    string rosbag;

    ROS_FATAL_STREAM_COND(!nh_private.getParam("gopro_folder", gopro_folder), "No video folder specified");
    ROS_FATAL_STREAM_COND(!nh_private.getParam("rosbag", rosbag), "No rosbag file specified");

    double scaling = 1.0;
    if (nh_private.hasParam("scale"))
        nh_private.getParam("scale", scaling);

    vector<fs::path> video_files;
    std::copy(fs::directory_iterator(gopro_folder), fs::directory_iterator(), std::back_inserter(video_files));
    std::sort(video_files.begin(), video_files.end());

    rosbag::Bag bag;
    bag.open(rosbag, rosbag::bagmode::Write);

    for (uint32_t i = 0; i < video_files.size(); i++)
    {
        if (video_files[i].extension() != ".MP4")
        {
            ROS_WARN_STREAM("Skipping " << video_files[i].filename().string() << " as it is not a .MP4 file");
            continue;
        }
        fs::path file = video_files[i];
        ROS_DEBUG_STREAM("GPMF meta for video: " << file.filename().string());
        GoProImuExtractor imu_extractor(file.string());

        vector<uint64_t> start_stamps;
        vector<uint32_t> samples;
        imu_extractor.getPayloadStamps(STR2FOURCC("ACCL"), start_stamps, samples);
        // for (uint32_t i = 0; i < samples.size(); i++)
        // {
        ROS_INFO_STREAM("Total Payloads: " << start_stamps.size() << " Start stamp: " << start_stamps[0]
                                           << " End stamp: " << start_stamps[samples.size() - 1]
                                           << " Total Samples: " << samples.at(samples.size() - 1));
        // }

        uint64_t end_stamp = 0;
        if (i < video_files.size() - 1)
        {
            GoProImuExtractor imu_extractor_next(video_files[i + 1].string());
            end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("ACCL"), 0);
        }

        imu_extractor.writeImuData(bag, "/gopro/imu", end_stamp);
        ROS_WARN_STREAM("Video " << i << " End stamp: " << end_stamp);
    }

    bag.close();
    return 0;
}
