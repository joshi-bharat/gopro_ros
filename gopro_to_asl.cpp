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

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "gopro_to_asl");
    ros::NodeHandle nh_private("~");

    string gopro_video;
    string asl_dir;

    ROS_FATAL_STREAM_COND(!nh_private.getParam("gopro_video", gopro_video), "No video file specified");
    ROS_FATAL_STREAM_COND(!nh_private.getParam("asl_dir", asl_dir), "No asl directory specified");

    double scaling = 1.0;
    if (nh_private.hasParam("scale"))
        nh_private.getParam("scale", scaling);

    string image_folder = asl_dir + "/mav0/cam0";

    if (!experimental::filesystem::is_directory(image_folder))
    {
        experimental::filesystem::create_directories(image_folder);
    }

    string imu_folder = asl_dir + "/mav0/imu0";
    if (!experimental::filesystem::is_directory(imu_folder))
    {
        experimental::filesystem::create_directories(imu_folder);
    }

    string imu_file = imu_folder + "/data.csv";

    GoProImuExtractor imu_extractor(gopro_video);
    imu_extractor.display_video_framerate();

    GoProVideoExtractor video_extractor(gopro_video, scaling);

    vector<uint64_t> start_stamps;
    vector<uint32_t> samples;
    imu_extractor.getFrameStamps(start_stamps, samples);
    vector<uint64_t> image_stamps;
    video_extractor.getFrameStamps(image_stamps);

    for (int i = 0; i < start_stamps.size(); i++)
    {
        cout << "start_stamp: " << i << ": " << start_stamps[i] << "  total_samples: " << samples[i] << endl;
    }

    uint32_t gpmf_frame_count = imu_extractor.getImageCount();
    uint32_t ffmpeg_frame_count = video_extractor.getFrameCount();
    if (gpmf_frame_count != ffmpeg_frame_count)
    {
        ROS_FATAL_STREAM_COND(gpmf_frame_count != ffmpeg_frame_count,
                              "Video and metadata frame count do not match");
    }
    assert(gpmf_frame_count == ffmpeg_frame_count);

    uint64_t gpmf_video_time = imu_extractor.getVideoCreationTime();
    uint64_t ffmpeg_video_time = video_extractor.getVideoCreationTime();

    if (ffmpeg_video_time != gpmf_video_time)
    {
        ROS_FATAL_STREAM_COND(ffmpeg_video_time != gpmf_video_time,
                              "Video creation time does not match");
    }
    assert(ffmpeg_video_time == gpmf_video_time);

    uint32_t first_stamp = start_stamps.at(0);

    uint32_t total_payloads = samples.size();
    uint32_t last_match = 0;
    for (uint32_t i = 0; i < start_stamps.size(); ++i)
    {
        uint64_t image_stamp;
        if (i == 0)
            image_stamp = image_stamps.at(0);
        else
            image_stamp = image_stamps.at(samples.at(i - 1));

        if (image_stamp != start_stamps.at(i) - first_stamp)
        {
            total_payloads = i;
            ROS_WARN_STREAM("Timestamps from gpmf and ffmpeg do not match" << endl);
            ROS_WARN_STREAM("Indx: " << i << "\tImage Count: " << samples.at(i - 1) << endl);
            ROS_WARN_STREAM("Image Stamps: " << image_stamp << "\t" << start_stamps.at(i) - first_stamp << RESET << endl);
            break;
        }
        last_match = image_stamp;
    }

    ROS_INFO_STREAM("Last stamp matched: " << last_match);
    // imu_extractor.show_current_payload(samples.size() - 1);
    // imu_extractor.save_imu_stream(imu_file);
    // video_extractor.extract_frames(image_folder);
    uint32_t skips = 0;
    if (samples.size() != total_payloads)
    {
        //
        skips = samples.size() - total_payloads;
        ROS_WARN_STREAM("Skipping " << skips << " payloads");
        ROS_FATAL_STREAM_COND(skips > 5, "Too many skips");
    }

    uint64_t last_image_stamp_ns = (uint64_t)(last_match * 1000);
    video_extractor.extractFrames(image_folder, last_image_stamp_ns);
    return 0;
}