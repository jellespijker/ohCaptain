cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

project(ohCaptain_main)

add_subdirectory(ohCaptain)
add_subdirectory(unitTest)
add_subdirectory(tools)
add_subdirectory(thirdparty)

#Deploy project
add_custom_target(deploy_${PROJECT_NAME}
        make install
        DEPENDS ${runTests} ${ohCaptain}
        COMMENT "Installing ${PROJECT_NAME}")