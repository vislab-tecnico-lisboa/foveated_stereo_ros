cmake_minimum_required(VERSION 3.2)

# Requirements
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)
FIND_PACKAGE(Eigen3 REQUIRED)

# Create static library
add_library(covSamplerLib include/covSampling.hpp src/covSampling.cpp)

target_include_directories(covSamplerLib PUBLIC ${EIGEN3_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)




