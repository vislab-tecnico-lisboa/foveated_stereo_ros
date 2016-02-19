cmake_minimum_required(VERSION 2.8.3)
project(foveated_stereo_ros)

find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

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
  #rviz_visual_tools
  tf_conversions
  message_generation
  nodelet
  actionlib_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  move_robot_msgs
)

find_package(PCL)
find_package(OpenCV REQUIRED)
#find_package(CUDA)

add_message_files(
  FILES
  Covariance.msg
  Information.msg
  PointClouds.msg
  StereoData.msg
  EgoData.msg
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

## foveated stereo lib
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/foveated_stereo/cpp)


####################
# NODE EXECUTABLES #
####################


## Declare a cpp executable
add_executable(conventional_stereo_ros_node src/ConventionalStereoRos.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(conventional_stereo_ros_node
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   foveated_stereo_lib
   ${OpenCV_LIBRARIES}
)
add_dependencies(conventional_stereo_ros_node foveated_stereo_ros_gencpp)

## Declare a cpp executable
add_executable(peripheral_stereo_ros_node src/PeripheralStereoRos.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(peripheral_stereo_ros_node
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   foveated_stereo_lib
   ${OpenCV_LIBRARIES}
)
add_dependencies(peripheral_stereo_ros_node foveated_stereo_ros_gencpp)

## Declare a cpp executable
add_executable(peripheral_foveal_stereo_ros_node src/PeripheralFovealStereoRos.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(peripheral_foveal_stereo_ros_node
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   foveated_stereo_lib
   ${OpenCV_LIBRARIES}
)
add_dependencies(peripheral_foveal_stereo_ros_node foveated_stereo_ros_gencpp)


## Declare a cpp executable
add_executable(ego_sphere_manager_ros_node src/EgoSphereManagerRos.cpp src/DecisionMaking.cpp src/UpperConfidenceBound.cpp src/AcquisitionFunction.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ego_sphere_manager_ros_node
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   foveated_stereo_lib
   ${OpenCV_LIBRARIES}
)

add_dependencies(ego_sphere_manager_ros_node foveated_stereo_ros_gencpp)

#####################
# NODELET LIBRARIES #
#####################

# Nodelet library
add_library(conventional_stereo_ros_nodelet src/ConventionalStereoRos.cpp src/nodelets/ConventionalStereoRosNodelet.cpp)
target_link_libraries(conventional_stereo_ros_nodelet
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   foveated_stereo_lib
   ${OpenCV_LIBRARIES})

install(TARGETS conventional_stereo_ros_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Nodelet library
add_library(peripheral_stereo_ros_nodelet src/PeripheralStereoRos.cpp src/nodelets/PeripheralStereoRosNodelet.cpp)
target_link_libraries(peripheral_stereo_ros_nodelet    
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   foveated_stereo_lib
   ${OpenCV_LIBRARIES})
add_dependencies(peripheral_stereo_ros_nodelet foveated_stereo_ros_gencpp)

install(TARGETS peripheral_stereo_ros_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Nodelet library
add_library(peripheral_foveal_stereo_ros_nodelet src/PeripheralStereoRos.cpp src/nodelets/PeripheralFovealStereoRosNodelet.cpp)
target_link_libraries(peripheral_foveal_stereo_ros_nodelet
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   foveated_stereo_lib
   ${OpenCV_LIBRARIES})
add_dependencies(peripheral_foveal_stereo_ros_nodelet foveated_stereo_ros_gencpp)

install(TARGETS peripheral_foveal_stereo_ros_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Nodelet library
add_library(ego_sphere_manager_ros_nodelet src/EgoSphereManagerRos.cpp src/DecisionMaking.cpp src/nodelets/EgoSphereManagerRosNodelet.cpp)
target_link_libraries(ego_sphere_manager_ros_nodelet
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   foveated_stereo_lib
   ${OpenCV_LIBRARIES})

add_dependencies(ego_sphere_manager_ros_nodelet foveated_stereo_ros_gencpp)

install(TARGETS ego_sphere_manager_ros_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


install(FILES foveated_stereo_nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(tracker_test2 src/tracker_test/tracker.cpp src/tracker_test/nonlinearanalyticconditionalgaussianodo.cpp src/tracker_test/tracker_node.cpp)
target_link_libraries(tracker_test2 ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BFL_LIBRARIES})
add_dependencies(tracker_test2 foveated_stereo_ros_generate_messages_cpp)

add_library(3D_point_estimation_ekf src/bayesian_filtering/nonlinearanalyticconditionalgaussian3D.cpp src/bayesian_filtering/nonlinearsystempdf.cpp src/bayesian_filtering/nonlinearmeasurementpdf.cpp)

add_executable(tracker_test src/bayesian_filtering/test/test_nonlinear_kalman.cpp src/bayesian_filtering/test/mobile_robot.cpp)
target_link_libraries(tracker_test 3D_point_estimation_ekf ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BFL_LIBRARIES})

add_executable(tracker_node src/bayesian_filtering/test/BayesianFilterNode.cpp)
target_link_libraries(tracker_node 3D_point_estimation_ekf ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BFL_LIBRARIES})
