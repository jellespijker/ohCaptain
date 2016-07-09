cmake_minimum_required(VERSION 3.5)
project(ohCaptain C CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(CMAKE_MODULE_PATH  "${ohCaptain_SOURCE_DIR}/../cmake")
set(Boost_INCLUDE_DIR "/usr/include/boost")
include_directories(Boost_INCLUDE_DIR)

set(ohCaptain_INCLUDE_DIR ${ohCaptain_SOURCE_DIR}/include)
set(ohCaptain_SOURCE_DIR ${ohCaptain_SOURCE_DIR}/src)

file(GLOB_RECURSE ohCaptain_INC_FILES ${ohCaptain_INCLUDE_DIR} *.h)
file(GLOB_RECURSE ohCaptain_SRC_FILES ${ohCaptain_SOURCE_DIR} *.cpp)

add_library(ohCaptain SHARED ${ohCaptain_SRC_FILES} ${ohCaptain_INC_FILES})