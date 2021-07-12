// tutorial01.c
//
// This tutorial was written by Stephen Dranger (dranger@gmail.com).
//
// Code based on a tutorial by Martin Bohme (boehme@inb.uni-luebeckREMOVETHIS.de)
// Tested on Gentoo, CVS version 5/01/07 compiled with GCC 4.1.1

// A small sample program that shows how to use libavformat and libavcodec to
// read video from a file.
//
// Use the Makefile to build all examples.
//
// Run using
//
// tutorial01 myvideofile.mpg
//
// to write the first five frames from "myvideofile.mpg" to disk in PPM
// format.

#include "extract_video.h"
#include "color_codes.h"

#include <utils.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <experimental/filesystem>
#include <iomanip>

void GoProVideoExtractor::save_to_png(AVFrame *frame, AVCodecContext *codecContext,  int width, int height,
                 AVRational time_base, std::string filename){

	AVCodec *outCodec = avcodec_find_encoder(AV_CODEC_ID_PNG);
	AVCodecContext *outCodecCtx = avcodec_alloc_context3(outCodec);

	outCodecCtx->width = width;
	outCodecCtx->height = height;
	outCodecCtx->pix_fmt = AV_PIX_FMT_RGB24;
	outCodecCtx->codec_type = AVMEDIA_TYPE_VIDEO;
	outCodecCtx->codec_id = AV_CODEC_ID_PNG;
	outCodecCtx->time_base.num = codecContext->time_base.num;
	outCodecCtx->time_base.den = codecContext->time_base.den;

	frame->height = height;
	frame->width = width;
	frame->format = AV_PIX_FMT_RGB24;

	if (!outCodec || avcodec_open2(outCodecCtx, outCodec, NULL) < 0) {
		return;
	}

	AVPacket outPacket;
	av_init_packet(&outPacket);
	outPacket.size = 0;
	outPacket.data = NULL;

	avcodec_send_frame(outCodecCtx, frame);
	int ret = -1;
	while (ret < 0){
		ret = avcodec_receive_packet(outCodecCtx, &outPacket);
	}

	filename  = filename + ".png";
	FILE * outPng = fopen(filename.c_str(), "wb");
	fwrite(outPacket.data, outPacket.size, 1, outPng);
	fclose(outPng);
}

void GoProVideoExtractor::save_raw(AVFrame *pFrame, int width, int height, std::string filename) {
	FILE *pFile;
	int  y;

	// Open file
	filename = filename + ".ppm";
	pFile=fopen(filename.c_str(), "wb");
	if(pFile==NULL)
		return;

	// Write header
	fprintf(pFile, "P6\n%d %d\n255\n", width, height);

	// Write pixel data
	for(y=0; y<height; y++)
		fwrite(pFrame->data[0]+y*pFrame->linesize[0], 1, width*3, pFile);

	// Close file
	fclose(pFile);
}

int GoProVideoExtractor::extract_frames(const std::string& image_folder, int width, int height) {

    std::string image_data_folder = image_folder + "/data";
    if(! std::experimental::filesystem::is_directory(image_data_folder)){
        std::experimental::filesystem::create_directories(image_data_folder);
    }

    std::string image_file = image_folder+"/data.csv";

    std::ofstream image_stream;
    image_stream.open(image_file);
    image_stream << std::fixed << std::setprecision(19);
    image_stream << "#timestamp [ns],filename" << std::endl;

    av_register_all();

    AVFormatContext *pFormatContext = NULL;
	int i, videoStreamIndex;
	AVCodecContext *pCodecContext = NULL;
	AVCodec *pCodec = NULL;
	AVFrame *pFrame = NULL;
	AVFrame *pFrameRGB = NULL;
	AVPacket packet;
	int frameFinished;
	int numBytes;
	uint8_t *buffer = NULL;

	AVDictionary *optionsDict = NULL;
	AVDictionaryEntry *tag_dict = NULL;
	struct SwsContext *sws_ctx = NULL;
	AVStream *video_stream = NULL;


	// Open video file
    std::cout << "Opening Video File: " << video_file << std::endl;
	pFormatContext = avformat_alloc_context();
	if (!pFormatContext) {
		printf("ERROR could not allocate memory for Format Context");
		return -1;
	}

	if (avformat_open_input(&pFormatContext, video_file.c_str(), NULL, NULL) != 0){
	    std::cout << RED << "Could not open file" << video_file.c_str() << RESET << std::endl;
        return -1;
	}
	// Retrieve stream information
	if (avformat_find_stream_info(pFormatContext, NULL) < 0)
		return -1; // Couldn't find stream information

	// Dump information about file onto standard error
	av_dump_format(pFormatContext, 0, video_file.c_str(), 0);

	std::string video_creation_time;

	AVCodecParameters *codecParameters;

	// Find the first video stream
	videoStreamIndex = -1;
	for (i = 0; i < pFormatContext->nb_streams; i++){
		codecParameters = pFormatContext->streams[i]->codecpar;
		if (codecParameters->codec_type == AVMEDIA_TYPE_VIDEO) {

			tag_dict = av_dict_get(pFormatContext->metadata, "", tag_dict, AV_DICT_IGNORE_SUFFIX);
			while(tag_dict){
				if(strcmp(tag_dict->key, "creation_time") == 0){
					std::stringstream  ss;
					ss << tag_dict->value;
					ss >> video_creation_time;
				}
				tag_dict = av_dict_get(pFormatContext->metadata, "", tag_dict, AV_DICT_IGNORE_SUFFIX);
			}

			videoStreamIndex = i;
			break;
		}
	}

	if(videoStreamIndex==-1)
		return -1; // Didn't find a video stream

	// Get a pointer to the codec context for the video stream
	pCodec = avcodec_find_decoder(pFormatContext->streams[videoStreamIndex]->codecpar->codec_id);
	video_stream = pFormatContext->streams[videoStreamIndex];

	// Find the decoder for the video stream
	if(pCodec==nullptr) {
		fprintf(stderr, "Unsupported codec!\n");
		return -1; // Codec not found
	}

	pCodecContext = avcodec_alloc_context3(pCodec);
	if (!pCodecContext)
	{
		printf("failed to allocated memory for AVCodecContext");
		return -1;
	}
	if (avcodec_parameters_to_context(pCodecContext, codecParameters) < 0)
	{
		printf("failed to copy codec params to codec context");
		return -1;
	}

	// Open codec
	if(avcodec_open2(pCodecContext, pCodec, &optionsDict)<0)
		return -1; // Could not open codec

	// Allocate video frame
	pFrame=av_frame_alloc();

	// Allocate an AVFrame structure
	pFrameRGB=av_frame_alloc();
	if(pFrameRGB==nullptr)
		return -1;

	//PIX_FMT_RGB24
	// Determine required buffer size and allocate buffer
	numBytes=av_image_get_buffer_size(AV_PIX_FMT_RGB24, width,
	                                  height, 1);
	buffer=(uint8_t *)av_malloc(numBytes*sizeof(uint8_t));

	sws_ctx =
			sws_getContext
					(
							pCodecContext->width,
							pCodecContext->height,
							pCodecContext->pix_fmt,
							width,
							height,
							AV_PIX_FMT_RGB24,
							SWS_BILINEAR,
							nullptr,
							nullptr,
							nullptr
					);
//
	// Assign appropriate parts of buffer to image planes in pFrameRGB
	// Note that pFrameRGB is an AVFrame, but AVFrame is a superset
	// of AVPicture
	av_image_fill_arrays(pFrameRGB->data, pFrameRGB->linesize, buffer, AV_PIX_FMT_RGB24,
	                     width, height, 1);


	uint64_t start_time = parseISO(video_creation_time);
//	std::cout << "Video Creation Time: " << uint64_to_string(start_time) << std::endl;

	double global_clock;
	uint64_t global_video_pkt_pts = AV_NOPTS_VALUE;

	while(av_read_frame(pFormatContext, &packet)>=0) {
		// Is this a packet from the video stream?
		if(packet.stream_index==videoStreamIndex) {
			// Decode video frame
//			avcodec_send_packet(pCodecContext, &packet);
//			frameFinished = avcodec_receive_frame(pCodecContext, pFrameRGB);
			avcodec_decode_video2(pCodecContext, pFrame, &frameFinished,
			                      &packet);


			// Did we get a video frame?

			if(packet.dts != AV_NOPTS_VALUE){
				global_clock = av_frame_get_best_effort_timestamp(pFrame);
				global_video_pkt_pts = packet.pts;

			}else if(global_video_pkt_pts && global_video_pkt_pts != AV_NOPTS_VALUE){
				global_clock = global_video_pkt_pts;
			}else{
				global_clock = 0;
			}

			double frame_delay = av_q2d(video_stream->time_base);
			global_clock *= frame_delay;

			//Only if we are repeating the
			if(pFrame->repeat_pict > 0){
				double extra_delay = pFrame->repeat_pict * (frame_delay * 0.5);
				global_clock += extra_delay;
			}


			if(frameFinished) {
				// Convert the image from its native format to RGB
				sws_scale
						(
								sws_ctx,
								(uint8_t const * const *)pFrame->data,
								pFrame->linesize,
								0,
								pCodecContext->height,
								pFrameRGB->data,
								pFrameRGB->linesize
						);

				// Save the frame to disk
//				std::cout << "Global Clock: " << global_clock << std::endl;
				uint64_t nanosecs = (uint64_t)(global_clock * 1e9);
//				std::cout << "Nano secs: " << nanosecs << std::endl;
				uint64_t current_stamp = start_time + nanosecs;
				std::string string_stamp = uint64_to_string(current_stamp);
				std::string stamped_image_filename = image_data_folder + "/" + string_stamp;
                image_stream << string_stamp << "," << string_stamp+".png" << std::endl;
				save_to_png(pFrameRGB, pCodecContext, width, height, video_stream->time_base, stamped_image_filename);
			}
		}

		// Free the packet that was allocated by av_read_frame
		av_packet_unref(&packet);
	}


	image_stream.close();

	// Free the RGB image
	av_free(buffer);
	av_free(pFrameRGB);

	// Free the YUV frame
	av_free(pFrame);

	// Close the codec
	avcodec_close(pCodecContext);

	// Close the video file
	avformat_close_input(&pFormatContext);

	return 0;
}