cmake_minimum_required(VERSION 3.5)
project(runTests CXX)

set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=c++14")

#setup GTest
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
set(BUILD_GMOCK             OFF)
set(BUILD_GTEST             ON)
set(BUILD_SHARED_LIBS       ON)

#setup boost
set(Boost_USE_STATIC_LIBS       OFF)
set(Boost_USE_MULTITHREADED     ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
find_package(Boost COMPONENTS system chrono REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

#Cross compile folders
if (ARCH_CROSS_COMPILE)
    include_directories("${ohCaptain_main_SOURCE_DIR}/build/arch_rootfs/usr/include/${CXX_COMPILER_ID}/")
    set(DEPLOY_FOLDER      "${ohCaptain_main_SOURCE_DIR}/build/share")
endif (ARCH_CROSS_COMPILE)

#build executable
add_executable(runTests Tests.cpp)
target_link_libraries(runTests gtest gtest_main)
target_link_libraries(runTests ohCaptain)
target_link_libraries(runTests pthread)

install( TARGETS runTests DESTINATION "${CMAKE_SOURCE_DIR}/binaries/bin" )