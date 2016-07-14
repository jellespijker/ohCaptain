cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

project(ohCaptain_main)

add_subdirectory(ohCaptain)
add_subdirectory(unitTest)