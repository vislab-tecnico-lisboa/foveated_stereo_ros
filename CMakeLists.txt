cmake_minimum_required(VERSION 2.8.3)
project(foveated_stereo_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_filters
  roscpp
  sensor_msgs
  std_msgs
  tf
  image_transport
	pcl_ros
)

find_package(PCL)
find_package(OpenCV)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES foveated_stereo_ros
#  CATKIN_DEPENDS cv_bridge geometry_msgs message_filters roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
include_directories(lib/foveated_stereo/cpp/cpp)
include_directories(lib/stereo_calib_lib/include)

## foveated stereo lib
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/foveated_stereo/cpp)

## stereo calibration lib
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/stereo_calib_lib)

## Declare a cpp executable
add_executable(foveated_stereo_ros_node src/foveated_stereo_ros_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(foveated_stereo_ros_node
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   foveated_stereo_lib
   STEREO_CALIB_LIB
)

## Declare a cpp executable
add_executable(ego_sphere_manager_ros_node src/EgoSphereManagerRos.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ego_sphere_manager_ros_node
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   foveated_stereo_lib
   STEREO_CALIB_LIB
)



