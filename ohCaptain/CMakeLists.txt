cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_FLAGS             "${CMAKE_CXX_FLAGS} -std=c++14 -g -DEIGEN_NO_DEBUG")
set(CMAKE_BUILD_TYPE Debug)
project(ohCaptain)

#setup boost
set(Boost_USE_STATIC_LIBS       OFF)
set(Boost_USE_MULTITHREADED     ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
find_package(Boost COMPONENTS filesystem system chrono REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

#setup Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIR})

#cross compile folders
if (ARCH_CROSS_COMPILE)
    include_directories("${ohCaptain_main_SOURCE_DIR}/build/arch_rootfs/usr/include/${CXX_COMPILER_ID}/")
endif (ARCH_CROSS_COMPILE)

#Setup include files
set(ohCaptain_INCLUDE_DIR       "${ohCaptain_SOURCE_DIR}/include")
set(ohCaptain_SOURCE_DIR        "${ohCaptain_SOURCE_DIR}/src")
file(GLOB_RECURSE ohCaptain_INC_FILES ${ohCaptain_INCLUDE_DIR} *.h)
file(GLOB_RECURSE ohCaptain_SRC_FILES ${ohCaptain_SOURCE_DIR} *.cpp)

#Build documentation
find_package(Doxygen)
if(DOXYGEN_FOUND)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(docu
            ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen" VERBATIM
            )
endif(DOXYGEN_FOUND)

#Build shared library
add_library(ohCaptain SHARED ${ohCaptain_SRC_FILES} ${ohCaptain_INC_FILES})
target_link_libraries(ohCaptain ${Boost_LIBRARIES})
target_link_libraries(ohCaptain pthread)

install( TARGETS ohCaptain DESTINATION "${CMAKE_SOURCE_DIR}/binaries/lib" )
install( DIRECTORY  "${ohCaptain_INCLUDE_DIR}/" DESTINATION "${CMAKE_SOURCE_DIR}/binaries/include" )
install( DIRECTORY  "${CMAKE_SOURCE_DIR}/arm/device-tree-overlay/dtbo" DESTINATION "${CMAKE_SOURCE_DIR}/binaries/bin" )
