//
// Created by bjoshi on 10/29/20.
//

#pragma once

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <string>
#include <iostream>
#include <vector>

#include "GPMF_parser.h"
#include "GPMF_mp4reader.h"
#include "color_codes.h"

class GoProImuExtractor
{

private:
    char *video;
    GPMF_stream metadata_stream;
    GPMF_stream *ms;
    double metadatalength;
    size_t mp4;
    uint32_t payloads;
    uint32_t *payload = NULL;
    size_t payloadres = 0;

    //Video Metadata
    uint32_t frame_count;
    uint32_t frame_rate;
    uint64_t movie_creation_time;

public:
    GoProImuExtractor(const std::string file);
    bool display_video_framerate();
    void cleanup();
    void show_gpmf_structure();
    GPMF_ERR get_scaled_data(uint32_t fourcc, std::vector<std::vector<double>> &readings);
    int save_imu_stream(std::string file);
    uint64_t get_stamp(uint32_t fourcc);
    uint32_t getNumofSamples(uint32_t fourcc);
    GPMF_ERR show_current_payload(uint32_t index);

    void getFrameStamps(std::vector<uint64_t> &start_stamps, std::vector<uint32_t> &samples);

    inline uint32_t getImageCount() { return frame_count; }
    inline uint64_t getVideoCreationTime() { return movie_creation_time; }
};
