cmake_minimum_required(VERSION 3.5)
project(runTests CXX)

set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=c++14")

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
set(BUILD_GMOCK             OFF)
set(BUILD_GTEST             ON)
set(BUILD_SHARED_LIBS       ON)

find_package(Boost)
include_directories(${Boost_INCLUDE_DIR})

if (ARCH_CROSS_COMPILE)
    include_directories("${ohCaptain_main_SOURCE_DIR}/build/arch_rootfs/usr/include/${CXX_COMPILER_ID}/")
endif (ARCH_CROSS_COMPILE)

add_executable(runTests Task_check.cpp)
target_link_libraries(runTests gtest gtest_main)
target_link_libraries(runTests ohCaptain)
target_link_libraries(runTests pthread)