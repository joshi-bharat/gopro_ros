cmake_minimum_required(VERSION 3.10)
project(gopro_extract)

add_library(${PROJECT_NAME} SHARED extract_imu.cpp extract_video.cpp)
target_link_libraries(${PROJECT_NAME} ${FFMPEG_LIBRARIES} ${OpenCV_LIBS}  utils gpmf-parser )


