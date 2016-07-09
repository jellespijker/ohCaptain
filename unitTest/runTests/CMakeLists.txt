cmake_minimum_required(VERSION 3.5)
project(runTests)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

add_executable(runTests
        Task_check.cpp)

target_link_libraries(runTests gtest gtest_main)
target_link_libraries(runTests ohCaptain)