//
// Created by bjoshi on 10/29/20.
//

#pragma once

extern "C"
{

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
}

#include <cstdio>
#include <string>
#include <vector>

class GoProVideoExtractor
{

private:
    std::string video_file;

    // Video properties
    AVFormatContext *pFormatContext = NULL;
    uint32_t videoStreamIndex;
    AVCodecContext *pCodecContext = NULL;
    AVCodec *pCodec = NULL;
    AVFrame *pFrame = NULL;
    AVFrame *pFrameRGB = NULL;
    AVPacket packet;

    AVDictionary *optionsDict = NULL;
    AVDictionaryEntry *tag_dict = NULL;
    struct SwsContext *sws_ctx = NULL;
    AVStream *video_stream = NULL;
    AVCodecParameters *codecParameters;

    uint64_t video_creation_time;
    uint32_t image_width;
    uint32_t image_height;
    uint32_t num_frames;

public:
    GoProVideoExtractor(const std::string file, double scaling_factor = 1.0);
    ~GoProVideoExtractor();

    void save_to_png(AVFrame *frame, AVCodecContext *codecContext, int width, int height,
                     AVRational time_base, std::string filename);

    void save_raw(AVFrame *pFrame, int width, int height, std::string filename);

    int extract_frames(const std::string &base_folder);

    int getFrameStamps(std::vector<uint64_t> &stamps);

    inline uint32_t getFrameCount() { return num_frames; }
    inline uint64_t getVideoCreationTime() { return video_creation_time; }
};
