cmake_minimum_required(VERSION 3.10)
project(gopro_ros)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(catkin REQUIRED COMPONENTS sensor_msgs roscpp geometry_msgs )

catkin_package(
		INCLUDE_DIRS include
		CATKIN_DEPENDS sensor_msgs  roscpp geometry_msgs 
		LIBRARIES ${PROJECT_NAME}
)

find_package(ffmpeg REQUIRED)
#find_package(SDL2 REQUIRED)
find_package(OpenCV REQUIRED)

if(FFMPEG_FOUND)
    message("Found FFMPEG/LibAV libraries")
else()
    message("Can't find libavcodec, libavformat or libavutil. Add them!")
endif()

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${FMPEG_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )


file(GLOB c_src "src/*.c")
file(GLOB cpp_src "src/*.cpp")

add_library(${PROJECT_NAME} SHARED ${c_src} ${cpp_src})
target_link_libraries(${PROJECT_NAME} PRIVATE ${FFMPEG_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(gopro_to_asl gopro_to_asl.cpp)
target_link_libraries(gopro_to_asl ${PROJECT_NAME} -lstdc++fs)

add_executable(image_stamp_test test_image_stamps.cpp)
target_link_libraries(image_stamp_test ${PROJECT_NAME})
#add_executable(demo GPMF_demo.c)
#target_link_libraries(demo utils gpmf-parser gopro_extract -lstdc++fs)
