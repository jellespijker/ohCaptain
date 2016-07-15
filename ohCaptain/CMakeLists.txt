cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_FLAGS             "${CMAKE_CXX_FLAGS} -std=c++14")
project(ohCaptain)

#setup boost
set(Boost_USE_STATIC_LIBS       OFF)
set(Boost_USE_MULTITHREADED     ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
find_package(Boost)
include_directories(${Boost_INCLUDE_DIR})

#cross compile folders
if (ARCH_CROSS_COMPILE)
    include_directories("${ohCaptain_main_SOURCE_DIR}/build/arch_rootfs/usr/include/${CXX_COMPILER_ID}/")
    set(DEPLOY_FOLDER           "${ohCaptain_main_SOURCE_DIR}/build/share")
endif (ARCH_CROSS_COMPILE)

#Setup include files
set(ohCaptain_INCLUDE_DIR       "${ohCaptain_SOURCE_DIR}/include")
set(ohCaptain_SOURCE_DIR        "${ohCaptain_SOURCE_DIR}/src")
file(GLOB_RECURSE ohCaptain_INC_FILES ${ohCaptain_INCLUDE_DIR} *.h)
file(GLOB_RECURSE ohCaptain_SRC_FILES ${ohCaptain_SOURCE_DIR} *.cpp)

#Build shared library
add_library(ohCaptain SHARED ${ohCaptain_SRC_FILES} ${ohCaptain_INC_FILES})
target_link_libraries(ohCaptain ${Boost_LIBRARIES})
target_link_libraries(ohCaptain pthread)

#install targets
if (ARCH_CROSS_COMPILE)
    install( TARGETS ohCaptain DESTINATION "${DEPLOY_FOLDER}/lib" )
    install( DIRECTORY  "${ohCaptain_INCLUDE_DIR}/" DESTINATION "${DEPLOY_FOLDER}/include" )
    else()
    install( TARGETS ohCaptain DESTINATION "/usr/lib" )
    install( DIRECTORY "${ohCaptain_INCLUDE_DIR}/" DESTINATION "usr/include")
endif(ARCH_CROSS_COMPILE)
