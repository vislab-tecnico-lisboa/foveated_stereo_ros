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
  rviz_visual_tools
  tf_conversions
  message_generation
  nodelet
  actionlib_msgs

  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

find_package(PCL)
find_package(OpenCV)
#find_package(CUDA)

add_message_files(
  FILES
  Covariance.msg
  Information.msg
  Stereo.msg
)

add_action_files(
  DIRECTORY action
  FILES Gaze.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES foveated_stereo_ros
#  CATKIN_DEPENDS cv_bridge geometry_msgs message_filters roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
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
  #${CUDA_INCLUDE_DIRS}
)

include_directories(lib/foveated_stereo/cpp/cpp)
include_directories(lib/stereo_calib_lib/include)

## foveated stereo lib
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/foveated_stereo/cpp)

## stereo calibration lib
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/stereo_calib_lib)

## Declare a cpp executable
add_executable(foveated_stereo_ros_node src/FoveatedStereoRos.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(foveated_stereo_ros_node
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   foveated_stereo_lib
   STEREO_CALIB_LIB
   ${OpenCV_LIBRARIES}
)
add_dependencies(foveated_stereo_ros_node foveated_stereo_ros_generate_messages_cpp)
## Declare a cpp executable
add_executable(ego_sphere_manager_ros_node src/EgoSphereManagerRos.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ego_sphere_manager_ros_node
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   foveated_stereo_lib
   STEREO_CALIB_LIB
   ${OpenCV_LIBRARIES}
)

add_dependencies(ego_sphere_manager_ros_node foveated_stereo_ros_generate_messages_cpp)


# Nodelet libraries
add_library(foveated_stereo_ros_nodelet src/FoveatedStereoRos.cpp src/FoveatedStereoRosNodelet.cpp)
target_link_libraries(foveated_stereo_ros_nodelet    
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   foveated_stereo_lib
   STEREO_CALIB_LIB
   ${OpenCV_LIBRARIES})

install(TARGETS foveated_stereo_ros_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES foveated_stereo_nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Nodelet library
add_library(ego_sphere_manager_ros_nodedelet src/EgoSphereManagerRos.cpp src/EgoSphereManagerRosNodelet.cpp)
target_link_libraries(ego_sphere_manager_ros_nodedelet
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   foveated_stereo_lib
   STEREO_CALIB_LIB
   ${OpenCV_LIBRARIES})

install(TARGETS ego_sphere_manager_ros_nodedelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES foveated_stereo_nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(gaze src/Gaze.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(gaze
   ${catkin_LIBRARIES}
)

add_executable(gaze_client src/GazeClient.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(gaze_client
   ${catkin_LIBRARIES}
)

#add_executable(teste src/test.cpp)
## Specify libraries to link a library or executable target against
#target_link_libraries(teste
#   ${catkin_LIBRARIES}
#)

