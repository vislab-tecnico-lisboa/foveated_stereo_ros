cmake_minimum_required(VERSION 3.2)

project(covSampler LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "No build type selected, default to Release")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)


# ------------------- SETUP ---------------------------

# Binaries and libraries location
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

# ------------------- BUILDING -------------------------

add_subdirectory ( "covSamplerLib" )
add_subdirectory ( "covSamplerTester" )




