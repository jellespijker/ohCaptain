cmake_minimum_required(VERSION 3.5)
project(LoRaRangeChecker CXX)

set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=c++14")

#setup boost
set(Boost_USE_STATIC_LIBS       OFF)
set(Boost_USE_MULTITHREADED     ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
find_package(Boost COMPONENTS system chrono REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

#Cross compile folders
if (ARCH_CROSS_COMPILE)
    include_directories("${ohCaptain_main_SOURCE_DIR}/build/arch_rootfs/usr/include/${CXX_COMPILER_ID}/")
endif (ARCH_CROSS_COMPILE)

#build executable
add_executable(LoRaRangeChecker main.cpp)
target_link_libraries(LoRaRangeChecker ohCaptain)
target_link_libraries(LoRaRangeChecker pthread)

install(TARGETS LoRaRangeChecker DESTINATION "${CMAKE_SOURCE_DIR}/binaries/bin" )