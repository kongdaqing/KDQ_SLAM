cmake_minimum_required(VERSION 2.8.3)
project(read_image)

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


add_executable(read_image_node
    src/read_image.cpp
    )

target_link_libraries(read_image_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
