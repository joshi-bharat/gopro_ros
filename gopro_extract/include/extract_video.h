//
// Created by bjoshi on 10/29/20.
//

#ifndef VIDEO_SYNCHRONIZATION_EXTRACT_VIDEO_H
#define VIDEO_SYNCHRONIZATION_EXTRACT_VIDEO_H

extern "C"{

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
}

#include <cstdio>
#include <string>

class GoProVideoExtractor{

private:
    std::string video_file;

public:
    GoProVideoExtractor(std::string file){
        video_file = file;
    }

    void save_to_png(AVFrame *frame, AVCodecContext *codecContext,  int width, int height,
                     AVRational time_base, std::string filename);

    void save_raw(AVFrame *pFrame, int width, int height, std::string filename);

    int extract_frames(const std::string& base_folder, int width, int height);

};
#endif //VIDEO_SYNCHRONIZATION_EXTRACT_VIDEO_H
