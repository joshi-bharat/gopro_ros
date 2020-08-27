cmake_minimum_required(VERSION 3.10)
project(video_synchronization)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(ffmpeg REQUIRED)
#find_package(SDL2 REQUIRED)
find_package(OpenCV REQUIRED)

if(FFMPEG_FOUND)
    message("Found FFMPEG/LibAV libraries")
else()
    message("Can't find libavcodec, libavformat or libavutil. Add them!")
endif()

include_directories(${FMPEG_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
#                    ${SDL2_INCLUDE_DIRS}
                    utils/include
                    )

add_subdirectory(utils)
add_executable(video_synchronization main.cpp)
target_link_libraries(video_synchronization ${FFMPEG_LIBRARIES} ${OpenCV_LIBS}  utils)
#target_link_libraries(video_synchronization ${FFMPEG_LIBRARIES} ${OpenCV_LIBS} ${SDL2_LIBRARIES} utils)

include_directories(gpmf-parser/include)
add_subdirectory(gpmf-parser)

add_executable(extract_imu extract_imu.cpp)
target_link_libraries(extract_imu gpmf-parser)
