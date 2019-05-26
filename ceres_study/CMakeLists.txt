cmake_minimum_required(VERSION 3.5)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
project(ceres_study)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ./bin)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
#find ceres directory
include_directories(
  ${Ceres_INCLUDE_DIRS})

add_executable(ceres_demo1 src/ceres_demo1.cpp)
#execute with ceres library
target_link_libraries(ceres_demo1 ${CERES_LIBRARIES})
