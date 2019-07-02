cmake_minimum_required(VERSION 2.8.3)
project(feature_detector)

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


add_executable(harris_detector
    src/harris_detector.cpp
    )

target_link_libraries(harris_detector ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(feature_detector_node
    src/FeatureDetectorRos.cpp
    src/featuredetector.cpp
    )
target_link_libraries(feature_detector_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
