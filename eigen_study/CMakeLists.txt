cmake_minimum_required(VERSION 3.5)
SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
project(eigen_study)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ./bin)
find_package(Eigen3 REQUIRED)
#find ceres directory
include_directories(
  ${EIGEN3_INCLUDE_DIRS})

add_executable(quad2matrix src/quad2matrix.cpp)


