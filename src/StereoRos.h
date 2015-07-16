#ifndef STEREOROS_H
#define STEREOROS_H

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include "Stereo.h"
#include <stereo_calib_lib.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <iostream>
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "opencv2/core/core.hpp"
#include "opencv2/core/eigen.hpp"

#include <tf_conversions/tf_eigen.h>
#include <foveated_stereo_ros/StereoData.h>
#include <boost/thread.hpp>
#include <pcl_ros/transforms.h>

using namespace sensor_msgs;

using namespace message_filters;

class StereoRos
{
protected:
    int number_of_disparities;
    int ignore_border_left;
    boost::mutex connect_mutex_;
    // For visualizing things in rviz
    tf::TransformListener listener;
    tf::StampedTransform l_eye_transform;
    tf::StampedTransform r_eye_transform;
    tf::StampedTransform r_l_eye_transform;

    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    ros::Publisher marker_pub;
    ros::Publisher stereo_data_publisher;

    ros::Subscriber left_camera_info_sub;

    ros::NodeHandle nh;
    ros::NodeHandle private_node_handle;
    ros::Publisher point_clouds_publisher;
    ros::Publisher rgb_point_cloud_publisher;
    ros::Publisher uncertainty_point_cloud_publisher;
    ros::Publisher mean_point_cloud_publisher;
    std::vector<ros::Publisher> sigma_point_clouds_publishers;
    std::string ego_frame;
    std::string left_camera_frame;
    std::string right_camera_frame;
    double uncertainty_lower_bound;

    cv::Mat left_cam_intrinsic, right_cam_intrinsic;

public:
    typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

    boost::shared_ptr<message_filters::Subscriber<Image> > left_image_sub;
    boost::shared_ptr<message_filters::Subscriber<Image> > right_image_sub;

    boost::shared_ptr<Synchronizer<MySyncPolicy> >sync;

    boost::shared_ptr<stereo_calib> stereo_calibration;
    boost::shared_ptr<Stereo> stereo;

    ~StereoRos();

    StereoRos(ros::NodeHandle & nh_,
                  ros::NodeHandle & private_node_handle_
                  );


    stereo_calib_params fillStereoCalibParams(float & baseline);

    void callback(const ImageConstPtr& left_image,
                  const ImageConstPtr& right_image);

    void publishCovarianceMatrices(StereoData & sdd, const ros::Time & time);

    void publishStereoData(StereoData & sdd, const ros::Time & time);

};

#endif // STEREOROS_H
