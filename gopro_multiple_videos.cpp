//
// Created by bjoshi on 10/29/20.
//

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>

#include <experimental/filesystem>
#include <iostream>

#include "ImuExtractor.h"
#include "VideoExtractor.h"

using namespace std;
namespace fs = std::experimental::filesystem;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "gopro");
  ros::NodeHandle nh_private("~");

  string gopro_folder;
  string rosbag;

  ROS_FATAL_STREAM_COND(!nh_private.getParam("gopro_folder", gopro_folder),
                        "No video folder specified");
  ROS_FATAL_STREAM_COND(!nh_private.getParam("rosbag", rosbag), "No rosbag file specified");

  double scaling = 1.0;
  if (nh_private.hasParam("scale")) nh_private.getParam("scale", scaling);

  vector<fs::path> video_files;
  std::copy(fs::directory_iterator(gopro_folder),
            fs::directory_iterator(),
            std::back_inserter(video_files));
  std::sort(video_files.begin(), video_files.end());

  rosbag::Bag bag;
  bag.open(rosbag, rosbag::bagmode::Write);

  // Opening the last video file
  vector<uint64_t> start_stamps;
  vector<uint32_t> samples;
  GoProImuExtractor imu_extractor(video_files.back().string());
  imu_extractor.getPayloadStamps(STR2FOURCC("ACCL"), start_stamps, samples);
  ROS_INFO_STREAM("ACCL Time End: " << start_stamps.back() << " Samples: " << samples.back());

  imu_extractor.getPayloadStamps(STR2FOURCC("GYRO"), start_stamps, samples);
  ROS_INFO_STREAM("GYRO Time End: " << start_stamps.back() << " Samples: " << samples.back());

  std::deque<AcclMeasurement> accl_queue;
  std::deque<GyroMeasurement> gyro_queue;

  for (uint32_t i = 0; i < video_files.size(); i++) {
    if (video_files[i].extension() != ".MP4") {
      ROS_WARN_STREAM("Skipping " << video_files[i].filename().string()
                                  << " as it is not a .MP4 file");
      continue;
    }
    fs::path file = video_files[i];
    ROS_DEBUG_STREAM("GPMF meta for video: " << file.filename().string());
    GoProImuExtractor imu_extractor(file.string());

    imu_extractor.getPayloadStamps(STR2FOURCC("ACCL"), start_stamps, samples);
    ROS_INFO_STREAM("[ACCL] Payloads: " << start_stamps.size()
                                        << " Start stamp: " << start_stamps[0]
                                        << " End stamp: " << start_stamps[samples.size() - 1]
                                        << " Total Samples: " << samples.at(samples.size() - 1));
    imu_extractor.getPayloadStamps(STR2FOURCC("GYRO"), start_stamps, samples);
    ROS_INFO_STREAM("[GYRO] Payloads: " << start_stamps.size()
                                        << " Start stamp: " << start_stamps[0]
                                        << " End stamp: " << start_stamps[samples.size() - 1]
                                        << " Total Samples: " << samples.at(samples.size() - 1));

    uint64_t accl_end_stamp = 0, gyro_end_stamp = 0;
    if (i < video_files.size() - 1) {
      GoProImuExtractor imu_extractor_next(video_files[i + 1].string());
      accl_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("ACCL"), 0);
      gyro_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("GYRO"), 0);
    }

    imu_extractor.readImuData(accl_queue, gyro_queue, accl_end_stamp, gyro_end_stamp);
    ROS_INFO_STREAM("[ACCL] Payloads: " << accl_queue.size());
    ROS_INFO_STREAM("[GYRO] Payloads: " << gyro_queue.size());
  }

  while (!accl_queue.empty() && !gyro_queue.empty()) {
    AcclMeasurement accl = accl_queue.front();
    GyroMeasurement gyro = gyro_queue.front();
    // ROS_INFO_STREAM("***************Here**********");
    int64_t diff = accl.timestamp_ - gyro.timestamp_;
    if (abs(diff) > 100000) {
      // I will need to handle this case more carefully using interpolation
      ROS_FATAL_STREAM("[ACCL] " << diff << " ns between imu and accl");
      ros::requestShutdown();
    } else {
      Timestamp stamp = accl.timestamp_;
      uint32_t secs = stamp * 1e-9;
      uint32_t n_secs = stamp % 1000000000;
      ros::Time ros_time(secs, n_secs);

      sensor_msgs::Imu imu_msg;
      std_msgs::Header header;
      header.stamp = ros_time;
      header.frame_id = "body";
      imu_msg.header = header;
      imu_msg.linear_acceleration.x = accl.accl_.x();
      imu_msg.linear_acceleration.y = accl.accl_.y();
      imu_msg.linear_acceleration.z = accl.accl_.z();
      imu_msg.angular_velocity.x = gyro.gyro_.x();
      imu_msg.angular_velocity.y = gyro.gyro_.y();
      imu_msg.angular_velocity.z = gyro.gyro_.z();

      bag.write("/gopro/imu", ros_time, imu_msg);

      accl_queue.pop_front();
      gyro_queue.pop_front();
    }
  }

  bag.close();

  return 0;
}
