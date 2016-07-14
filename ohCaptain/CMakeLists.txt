cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_FLAGS             "${CMAKE_CXX_FLAGS} -std=c++14")
project(ohCaptain)

set(Boost_USE_STATIC_LIBS       OFF)
set(Boost_USE_MULTITHREADED     ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
find_package(Boost)
include_directories(${Boost_INCLUDE_DIR})

if (ARCH_CROSS_COMPILE)
    include_directories("${ohCaptain_main_SOURCE_DIR}/build/arch_rootfs/usr/include/${CXX_COMPILER_ID}/")
endif (ARCH_CROSS_COMPILE)

set(ohCaptain_INCLUDE_DIR       "${ohCaptain_SOURCE_DIR}/include")
set(ohCaptain_SOURCE_DIR        "${ohCaptain_SOURCE_DIR}/src")

file(GLOB_RECURSE ohCaptain_INC_FILES ${ohCaptain_INCLUDE_DIR} *.h)
file(GLOB_RECURSE ohCaptain_SRC_FILES ${ohCaptain_SOURCE_DIR} *.cpp)

add_library(ohCaptain SHARED ${ohCaptain_SRC_FILES} ${ohCaptain_INC_FILES})
target_link_libraries(ohCaptain ${Boost_LIBRARIES})
target_link_libraries(ohCaptain pthread)
