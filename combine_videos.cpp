extern "C" {

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>

#include <experimental/filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

ros::Time saveBag(rosbag::Bag& bag,
                  const std::string& video_file,
                  std::string& img_topic,
                  ros::Time start_time,
                  double duration,
                  double scaling_factor,
                  bool compressed) {
  ros::Time current_time = start_time;

  AVFormatContext* pFormatContext = NULL;
  uint32_t videoStreamIndex;
  AVCodecContext* pCodecContext = NULL;
  AVCodec* pCodec = NULL;
  AVFrame* pFrame = NULL;
  AVFrame* pFrameRGB = NULL;
  AVPacket packet;

  AVDictionary* optionsDict = NULL;
  AVDictionaryEntry* tag_dict = NULL;
  struct SwsContext* sws_ctx = NULL;
  AVStream* video_stream = NULL;
  AVCodecParameters* codecParameters;

  av_register_all();

  // Open video file
  std::cout << "Opening Video File: " << video_file << std::endl;
  pFormatContext = avformat_alloc_context();
  if (!pFormatContext) {
    printf("ERROR could not allocate memory for Format Context");
  }

  if (avformat_open_input(&pFormatContext, video_file.c_str(), NULL, NULL) != 0) {
    ROS_FATAL_STREAM("Could not open file" << video_file.c_str());
  }
  // Retrieve stream information
  if (avformat_find_stream_info(pFormatContext, NULL) < 0)
    ROS_FATAL_STREAM("Couldn't find stream information");

  // Find the first video stream
  videoStreamIndex = -1;

  for (uint32_t i = 0; i < pFormatContext->nb_streams; i++) {
    codecParameters = pFormatContext->streams[i]->codecpar;
    if (codecParameters->codec_type == AVMEDIA_TYPE_VIDEO) {
      videoStreamIndex = i;
      break;
    }
  }

  if (videoStreamIndex == -1) ROS_FATAL_STREAM("Didn't find a video stream");

  video_stream = pFormatContext->streams[videoStreamIndex];

  // Get a pointer to the codec context for the video stream
  pCodec = avcodec_find_decoder(pFormatContext->streams[videoStreamIndex]->codecpar->codec_id);

  // Find the decoder for the video stream
  if (pCodec == nullptr) {
    ROS_FATAL_STREAM("Unsupported codec!");
  }

  pCodecContext = avcodec_alloc_context3(pCodec);
  if (!pCodecContext) {
    ROS_FATAL_STREAM("Failed to allocated memory for AVCodecContext");
  }

  if (avcodec_parameters_to_context(pCodecContext, codecParameters) < 0) {
    ROS_FATAL_STREAM("failed to copy codec params to codec context");
  }

  // Open codec
  if (avcodec_open2(pCodecContext, pCodec, &optionsDict) < 0)
    ROS_FATAL_STREAM("Could not open codec");

  // Allocate video frame
  pFrame = av_frame_alloc();

  // Allocate an AVFrame structure
  pFrameRGB = av_frame_alloc();
  if (pFrameRGB == nullptr) ROS_FATAL_STREAM("Cannot allocate RGB Frame");

  uint32_t image_height = pCodecContext->height;
  uint32_t image_width = pCodecContext->width;

  if (scaling_factor != 1.0) {
    image_height = (int)((double)image_height * scaling_factor);
    image_width = (int)((double)image_width * scaling_factor);
  }

  // PIX_FMT_RGB24
  // Determine required buffer size and allocate buffer
  int numBytes = av_image_get_buffer_size(AV_PIX_FMT_RGB24, image_width, image_height, 1);
  uint8_t* buffer = (uint8_t*)av_malloc(numBytes * sizeof(uint8_t));

  sws_ctx = sws_getContext(pCodecContext->width,
                           pCodecContext->height,
                           pCodecContext->pix_fmt,
                           image_width,
                           image_height,
                           AV_PIX_FMT_RGB24,
                           SWS_BILINEAR,
                           nullptr,
                           nullptr,
                           nullptr);
  //
  // Assign appropriate parts of buffer to image planes in pFrameRGB
  // Note that pFrameRGB is an AVFrame, but AVFrame is a superset
  // of AVPicture
  av_image_fill_arrays(
      pFrameRGB->data, pFrameRGB->linesize, buffer, AV_PIX_FMT_RGB24, image_width, image_height, 1);

  int frameFinished;
  while (av_read_frame(pFormatContext, &packet) >= 0) {
    // Is this a packet from the video stream?
    if (packet.stream_index == videoStreamIndex) {
      // Decode video frame
      avcodec_decode_video2(pCodecContext, pFrame, &frameFinished, &packet);

      if (frameFinished) {
        // Convert the image from its native format to RGB
        sws_scale(sws_ctx,
                  (uint8_t const* const*)pFrame->data,
                  pFrame->linesize,
                  0,
                  pCodecContext->height,
                  pFrameRGB->data,
                  pFrameRGB->linesize);

        cv::Mat img(image_height, image_width, CV_8UC3, pFrameRGB->data[0], pFrameRGB->linesize[0]);
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

        std_msgs::Header header;
        current_time = current_time + ros::Duration(duration);
        header.stamp = current_time;
        header.frame_id = "gopro";
        if (compressed) {
          sensor_msgs::CompressedImagePtr img_msg =
              cv_bridge::CvImage(header, "bgr8", img).toCompressedImageMsg();
          bag.write(img_topic + "/compressed", current_time, img_msg);
        } else {
          sensor_msgs::ImagePtr imgmsg =
              cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img).toImageMsg();
          bag.write(img_topic, current_time, imgmsg);
        }
      }
    }

    // Free the packet that was allocated by av_read_frame
    av_packet_unref(&packet);
  }

  // Free the buufer
  av_free(buffer);

  // Free the RGB image
  av_free(pFrameRGB);

  // Free the YUV frame
  av_free(pFrame);

  // Close the codec
  avcodec_close(pCodecContext);

  // Close the video formatcontext
  avformat_close_input(&pFormatContext);

  return current_time;
}

int main(int argc, char** argv) {
  std::string first_video = "/home/bjoshi/Downloads/GX010054.MP4";
  std::string second_video = "/home/bjoshi/Downloads/GX010058.MP4";
  std::string bag_file = "/home/bjoshi/gopro9/gopro_outdoor.bag";

  bool compress = false;
  double scaling_factor = 0.5;
  double duration = 1.0 / 30.0;

  ros::init(argc, argv, "gopro_bag");
  ros::NodeHandle n;

  ros::Time time = ros::Time::now();

  std::string img_topic = "/gopro/image_raw";

  rosbag::Bag bag;
  bag.open(bag_file, rosbag::bagmode::Write);

  time = saveBag(bag, first_video, img_topic, time, duration, scaling_factor, compress);
  time = time + ros::Duration(duration);
  saveBag(bag, second_video, img_topic, time, duration, scaling_factor, compress);
  bag.close();

  return 0;
}