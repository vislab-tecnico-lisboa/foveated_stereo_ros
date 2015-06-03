#ifndef FOVEATEDSTEREOROS_H
#define FOVEATEDSTEREOROS_H

#endif // FOVEATEDSTEREOROS_H
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
#include <Stereo.h>
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
using namespace sensor_msgs;

using namespace message_filters;

class FoveatedStereoNode
{
    // For visualizing things in rviz
    tf::TransformListener listener;
    tf::StampedTransform l_eye_transform;
    tf::StampedTransform r_eye_transform;
    tf::StampedTransform r_l_eye_transform;

    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    ros::Publisher marker_pub;
public:
    typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

    ros::NodeHandle nh;
    ros::Publisher point_cloud_publisher;
    ros::Publisher mean_point_cloud_publisher;
    std::vector<ros::Publisher> sigma_point_clouds_publishers;


    boost::shared_ptr<message_filters::Subscriber<Image> > left_image_sub;
    boost::shared_ptr<message_filters::Subscriber<Image> > right_image_sub;

    boost::shared_ptr<Synchronizer<MySyncPolicy> >sync;

    boost::shared_ptr<stereo_calib> stereo_calibration;
    boost::shared_ptr<Stereo> ego_sphere;

    ~FoveatedStereoNode();

    FoveatedStereoNode(ros::NodeHandle & nh_,
                       int rings_,
                       double min_radius_,
                       int interp_,
                       int full_,
                       int sectors_,
                       int sp_,
                       std::vector<double> & disparities_,
                       float sigma_,
                       float occ_likelihood_,
                       float pole_,
                       unsigned int spherical_angle_bins_,
                       float shell_radius_,
                       bool high_pass_);


    stereo_calib_params fillStereoCalibParams(float & baseline);

    void callback(const ImageConstPtr& left_image,
                  const ImageConstPtr& right_image);

    void publishPointClouds(StereoData & sdd, const ros::Time & time);

    void publishCovarianceMatrices(StereoData & sdd, const ros::Time & time);
};
