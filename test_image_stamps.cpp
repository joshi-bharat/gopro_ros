#include <string>
#include <cstring>
#include <cassert>

#include "ImuExtractor.h"
#include "VideoExtractor.h"

using namespace std;

int main(int argc, char *argv[])
{
    string video_file = "/home/bjoshi/Downloads/GX010037.MP4";
    string folder = "/home/bjoshi/gopro9";

    cout << RED << "Opening Video File: " << video_file << RESET << endl;
    GoProImuExtractor imu_extractor(video_file);

    vector<uint64_t> start_stamps;
    vector<uint32_t> samples;
    imu_extractor.display_video_framerate();
    imu_extractor.getPayloadStamps(STR2FOURCC("CORI"), start_stamps, samples);
    imu_extractor.show_current_payload(samples.size() - 1);

    vector<uint64_t> image_stamps;
    GoProVideoExtractor video_extractor(video_file);
    video_extractor.getFrameStamps(image_stamps);
    // video_extractor.extract_frames(folder, 960, 540);
    assert(start_stamps.size() == samples.size());

    uint64_t first_stamp = start_stamps.at(0);
    cout << GREEN << "Total images: " << image_stamps.size() << RESET << endl;

    for (uint32_t i = 0; i < start_stamps.size(); ++i)
    {
        uint64_t image_stamp;
        if (i == 0)
            image_stamp = image_stamps.at(0);
        else
            image_stamp = image_stamps.at(samples.at(i - 1));

        if (image_stamp != start_stamps.at(i) - first_stamp)
        {
            cout << RED << "Timestamps from meta and ffmpeg do not match" << endl;
            cout << "Indx: " << i << "\tImage Count: " << samples.at(i - 1) << endl;
            cout << "Image Stamps: " << image_stamp << "\t" << start_stamps.at(i) - first_stamp << RESET << endl;
        }
        // assert(image_stamp == (start_stamps.at(i) - first_stamp));
    }
    return 0;
}