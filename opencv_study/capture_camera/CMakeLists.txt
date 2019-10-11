cmake_minimum_required(VERSION 2.8.3)
project(capture_camera)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    )

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    )


add_executable(capture_camera_node
    src/capture_camera_demo1.cpp
    )

target_link_libraries(capture_camera_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
