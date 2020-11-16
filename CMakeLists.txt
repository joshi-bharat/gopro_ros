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
                    )

include_directories(utils/include)
add_subdirectory(utils)

include_directories(gpmf-parser/include)
add_subdirectory(gpmf-parser)

include_directories(gopro_extract/include)
add_subdirectory(gopro_extract)

add_executable(gopro_to_asl gopro_to_asl.cpp)
target_link_libraries(gopro_to_asl ${FFMPEG_LIBRARIES} ${OpenCV_LIBS}  utils gpmf-parser gopro_extract -lstdc++fs)

#add_executable(demo GPMF_demo.c)
#target_link_libraries(demo utils gpmf-parser gopro_extract -lstdc++fs)
