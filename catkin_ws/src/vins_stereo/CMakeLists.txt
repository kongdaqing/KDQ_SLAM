cmake_minimum_required(VERSION 2.8.3)
project(vins_stereo)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    )

find_package(OpenCV REQUIRED)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_library(depended_lib src/parameters/parameters.cpp
    src/rosVinsInterface.cpp
    src/vins_estimator/estimator.cpp
    src/feature/feature.cpp
    src/vinsInfoFollow.cpp
    )
target_link_libraries(depended_lib ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_executable(vins_node src/rosNodeVins.cpp)

target_link_libraries(vins_node ${catkin_LIBRARIES} depended_lib)

add_executable(feature_node src/rosFeatureTest.cpp
   src/parameters/parameters.cpp
 )
target_link_libraries(feature_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
