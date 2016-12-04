cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_FLAGS             "${CMAKE_CXX_FLAGS} -std=c++14")
project(ohCaptain)

#setup boost
set(Boost_USE_STATIC_LIBS       OFF)
set(Boost_USE_MULTITHREADED     ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
find_package(Boost COMPONENTS filesystem system chrono REQUIRED)
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

#Build documentation
#find_package(Doxygen)
#if(DOXYGEN_FOUND)
#    set(DOXYGEN_INPUT           "${ohCaptain_SOURCE_DIR}/Doxyfile")
#    set(DOXYGEN_OUTPUT          "${ohCaptain_SOURCE_DIR}/doc")

#    ADD_CUSTOM_COMMAND(
#            OUTPUT              ${DOXYGEN_OUTPUT}
#            COMMAND             ${CMAKE_COMMAND} -E echo_append "Building Documentation..."
#            COMMAND             ${DOXYGEN_EXECUTABLE} ${DOXYGEN_INPUT}
#            COMMAND             ${CMAKE_COMMAND} -E echo "Done."
#            WORKING_DIRECTORY   ${CMAKE_CURRENT_SOURCE_DIR}
#            DEPENDS             ${DOXYGEN_INPUT}
#    )

#    ADD_CUSTOM_TARGET(doc ALL DEPENDS ${DOXYGEN_INPUT})
#endif(DOXYGEN_FOUND)

#Build shared library
add_library(ohCaptain SHARED ${ohCaptain_SRC_FILES} ${ohCaptain_INC_FILES})
target_link_libraries(ohCaptain ${Boost_LIBRARIES})
target_link_libraries(ohCaptain pthread)

install( TARGETS ohCaptain DESTINATION "${CMAKE_SOURCE_DIR}/binaries/lib" )
install( DIRECTORY  "${ohCaptain_INCLUDE_DIR}/" DESTINATION "${CMAKE_SOURCE_DIR}/binaries/include" )
install( DIRECTORY  "${CMAKE_SOURCE_DIR}/arm/device-tree-overlay/dtbo" DESTINATION "${CMAKE_SOURCE_DIR}/binaries/bin" )
