cmake_minimum_required(VERSION 3.16)
project(utils)

set(CMAKE_CXX_STANDARD 14)

add_library(${PROJECT_NAME} SHARED utils.cpp)
