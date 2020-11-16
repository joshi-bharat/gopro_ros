# CMakeLists.txt
cmake_minimum_required (VERSION 3.8.2)
project (gpmf-parser)

file(GLOB LIB_SOURCES "GPMF_parser.c" "GPMF_mp4reader.c" "GPMF_print.cpp")

add_library(${PROJECT_NAME} SHARED ${LIB_SOURCES} ../gopro_to_asl.cpp)

