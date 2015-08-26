#ifndef STEREOROS_H
#define STEREOROS_H
#include <iostream>
#include <sstream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>


#include "opencv2/core/core.hpp"
#include "opencv2/core/eigen.hpp"

#include <foveated_stereo_ros/StereoData.h>
#include <boost/thread.hpp>


#include "Stereo.h"
#include <complete_stereo_calib_lib.h>
#include "images/imagesBase.h"
using namespace sensor_msgs;

using namespace message_filters;

class StereoRos
{
protected:
    int pre_filter_cap;
    int sad_window_size;
    int P1;
    int P2;
    int min_disparity;
    int uniqueness_ratio;
    int speckle_window_size;
    int speckle_range;
    int disp_12_max_diff;
    bool full_dp;
    unsigned int width;
    unsigned int height;

    int sectors;
    int rings;
    double min_radius;
    int interp;
    int sp;
    int full;

    double L;                                      // numer of states
    double alpha;                                 // default, tunable
    double ki;                                     // default, tunable
    double beta;
    double scaling_factor;

    int number_of_disparities;
    int ignore_border_left;
    boost::mutex connect_mutex_;
    // For visualizing things in rviz
    boost::shared_ptr<tf::TransformListener> listener;
    tf::StampedTransform l_eye_transform;
    tf::StampedTransform r_eye_transform;
    tf::StampedTransform r_l_eye_transform;

    image_transport::ImageTransport it_;
    image_transport::Publisher disparity_image_publisher;
    //image_transport::Publisher disparity_image_publishernuno;

    ros::NodeHandle nh;
    ros::NodeHandle private_node_handle;
    ros::Publisher rgb_point_cloud_publisher;
    ros::Publisher uncertainty_point_cloud_publisher;
    ros::Publisher mean_point_cloud_publisher;
    ros::Publisher marker_pub;
    ros::Publisher stereo_data_publisher;
    ros::Subscriber left_camera_info_sub;

    std::vector<ros::Publisher> sigma_point_clouds_publishers;
    std::string ego_frame;
    std::string left_camera_frame;
    std::string right_camera_frame;
    double information_lower_bound;

    cv::Mat left_cam_intrinsic, right_cam_intrinsic;

    cv::Mat left_image_mat;
    cv::Mat right_image_mat;

    cv::Mat R_left_cam_to_right_cam;
    cv::Mat t_left_cam_to_right_cam;

    cv::Mat transformation_left_cam_to_baseline_center;

    void cameraInfoCommon(const sensor_msgs::CameraInfoPtr & left_camera_info)
    {
        // Stereo SGBM params
        private_node_handle.param("numberOfDisparities", number_of_disparities, 80);
        private_node_handle.param("preFilterCap", pre_filter_cap, 63);
        private_node_handle.param("SADWindowSize", sad_window_size, 3);
        private_node_handle.param("P1", P1, 768);
        private_node_handle.param("P2", P2, 1536);
        private_node_handle.param("minDisparity", min_disparity, 0);
        private_node_handle.param("uniquenessRatio", uniqueness_ratio, 15);
        private_node_handle.param("speckleWindowSize", speckle_window_size, 50);
        private_node_handle.param("speckleRange", speckle_range, 16);
        private_node_handle.param("disp12MaxDiff", disp_12_max_diff, 0);
        private_node_handle.param("fullDP", full_dp, true);
        private_node_handle.param("ignore_border_left", ignore_border_left, 10);

        ROS_INFO_STREAM("number_of_disparities: "<<number_of_disparities);
        ROS_INFO_STREAM("pre_filter_cap: "<<pre_filter_cap);
        ROS_INFO_STREAM("sad_window_size: "<<sad_window_size);
        ROS_INFO_STREAM("P1: "<<P1);
        ROS_INFO_STREAM("P2: "<<P2);
        ROS_INFO_STREAM("min_disparity: "<<min_disparity);
        ROS_INFO_STREAM("uniqueness_ratio: "<<uniqueness_ratio);
        ROS_INFO_STREAM("speckle_window_size: "<<speckle_window_size);
        ROS_INFO_STREAM("speckle_range: "<<speckle_range);
        ROS_INFO_STREAM("disp_12_max_diff: "<<disp_12_max_diff);
        ROS_INFO_STREAM("full_dp: "<<full_dp);
        ROS_INFO_STREAM("ignore_border_left: "<<ignore_border_left);


        private_node_handle.param("sectors", sectors, 100);
        private_node_handle.param("rings", rings, 100);
        private_node_handle.param("min_radius", min_radius, 1.0);
        private_node_handle.param("interp", interp, 1);
        private_node_handle.param("sp", sp, 0);
        private_node_handle.param("full", full, 1);
        private_node_handle.param<std::string>("ego_frame", ego_frame, "ego_frame");
        private_node_handle.param<std::string>("left_camera_frame", left_camera_frame, "left_camera_frame");
        private_node_handle.param<std::string>("right_camera_frame", right_camera_frame, "right_camera_frame");
        private_node_handle.param("information_lower_bound", information_lower_bound, 0.0);
        private_node_handle.param("L", L, 1.0);
        private_node_handle.param("alpha", alpha, 1.0);
        private_node_handle.param("beta", beta, 2.0);
        private_node_handle.param("ki", ki, 1.0);
        private_node_handle.param("scaling_factor", scaling_factor, 1.0);

        ROS_INFO_STREAM("sectors: "<<sectors);
        ROS_INFO_STREAM("rings: "<<rings);
        ROS_INFO_STREAM("min_radius: "<<min_radius);
        ROS_INFO_STREAM("interp: "<<interp);
        ROS_INFO_STREAM("sp: "<<sp);
        ROS_INFO_STREAM("full: "<<full);
        ROS_INFO_STREAM("ego_frame: "<<ego_frame);
        ROS_INFO_STREAM("left_camera_frame: "<<left_camera_frame);
        ROS_INFO_STREAM("right_camera_frame: "<<right_camera_frame);
        ROS_INFO_STREAM("information_lower_bound: "<<information_lower_bound);
        ROS_INFO_STREAM("L: "<<L);
        ROS_INFO_STREAM("alpha: "<<alpha);
        ROS_INFO_STREAM("beta: "<<beta);
        ROS_INFO_STREAM("ki: "<<ki);
        ROS_INFO_STREAM("scaling_factor: "<<scaling_factor);

        ROS_INFO("Getting cameras' parameters");
        std::string right_camera_info_topic;
        private_node_handle.param<std::string>("right_camera_info_topic", right_camera_info_topic, "right_camera_frame");
        sensor_msgs::CameraInfoConstPtr right_camera_info=ros::topic::waitForMessage<sensor_msgs::CameraInfo>(right_camera_info_topic, ros::Duration(3.0));

        //set the cameras intrinsic parameters
        left_cam_intrinsic = Mat::eye(3,3,CV_64F);
        left_cam_intrinsic.at<double>(0,0) = left_camera_info->K.at(0);
        left_cam_intrinsic.at<double>(1,1) = left_camera_info->K.at(4);
        left_cam_intrinsic.at<double>(0,2) = left_camera_info->K.at(2);
        left_cam_intrinsic.at<double>(1,2) = left_camera_info->K.at(5);

        right_cam_intrinsic = Mat::eye(3,3,CV_64F);
        right_cam_intrinsic.at<double>(0,0) = right_camera_info->K.at(0);
        right_cam_intrinsic.at<double>(1,1) = right_camera_info->K.at(4);
        right_cam_intrinsic.at<double>(0,2) = right_camera_info->K.at(2);
        right_cam_intrinsic.at<double>(1,2) = right_camera_info->K.at(5);
        width=(unsigned int)left_camera_info->width;
        height=(unsigned int)left_camera_info->height;


        try
        {
            listener->waitForTransform(left_camera_frame, right_camera_frame, ros::Time(0), ros::Duration(1.0));
            listener->lookupTransform(left_camera_frame, right_camera_frame,
                                      ros::Time(0), r_l_eye_transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            return;
        }
    }

    void initTopics()
    {
        disparity_image_publisher = it_.advertise("/vizzy/disparity", 3);
        rgb_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("stereo", 10);
        mean_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("mean_pcl", 10);
        uncertainty_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("uncertainty_pcl", 10);
        /*for(int i=0; i<9; ++i)
        {
            std::stringstream ss;
            ss << i;
            std::string str = ss.str();
            //sigma_point_clouds_publishers.push_back(nh.advertise<sensor_msgs::PointCloud2>("sigma_point_clouds_"+str, 10));
        }*/
        stereo_data_publisher = nh.advertise<foveated_stereo_ros::StereoData>("stereo_data", 1);

        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("covariances", 1);

        left_image_sub=boost::shared_ptr<message_filters::Subscriber<Image> > (new message_filters::Subscriber<Image>(nh, "left_image", 10));
        right_image_sub=boost::shared_ptr<message_filters::Subscriber<Image> > (new message_filters::Subscriber<Image>(nh, "right_image", 10));

        left_to_right_sub=boost::shared_ptr<message_filters::Subscriber<geometry_msgs::TransformStamped> > (new message_filters::Subscriber<geometry_msgs::TransformStamped>(nh, "left_to_right_tf", 10));
        left_to_center_sub=boost::shared_ptr<message_filters::Subscriber<geometry_msgs::TransformStamped> > (new message_filters::Subscriber<geometry_msgs::TransformStamped>(nh, "left_to_center_tf", 10));

        sync=boost::shared_ptr<Synchronizer<MySyncPolicy> > (new Synchronizer<MySyncPolicy>(MySyncPolicy(10), *left_image_sub, *right_image_sub, *left_to_right_sub, *left_to_center_sub));
        //sync->registerCallback(boost::bind(&ConventionalStereoRos::callback, this, _1, _2));
    }

public:
    typedef sync_policies::ApproximateTime<Image, Image, geometry_msgs::TransformStamped, geometry_msgs::TransformStamped> MySyncPolicy;

    boost::shared_ptr<message_filters::Subscriber<Image> > left_image_sub;
    boost::shared_ptr<message_filters::Subscriber<Image> > right_image_sub;

    boost::shared_ptr<message_filters::Subscriber<geometry_msgs::TransformStamped> > left_to_right_sub;
    boost::shared_ptr<message_filters::Subscriber<geometry_msgs::TransformStamped> > left_to_center_sub;

    boost::shared_ptr<Synchronizer<MySyncPolicy> >sync;

    boost::shared_ptr<complete_stereo_calib> stereo_calibration;
    boost::shared_ptr<Stereo> stereo;

    ~StereoRos();

    StereoRos(ros::NodeHandle & nh_,
              ros::NodeHandle & private_node_handle_
              );


    complete_stereo_calib_params fillStereoCalibParams(const unsigned int & width, const unsigned int & height, const cv::Mat & left_cam_intrinsic, const cv::Mat & right_cam_intrinsic, const double & baseline, const double & resize_factor);


    void publishCovarianceMatrices(StereoData & sdd, const ros::Time & time);

    void publishStereoData(StereoData & sdd, const ros::Time & time);

    void callbackCommon(const ImageConstPtr& left_image,
                        const ImageConstPtr& right_image,
                        const geometry_msgs::TransformStampedConstPtr& left_to_right_transform_,
                        const geometry_msgs::TransformStampedConstPtr& left_to_center_transform_)
    {
        // 1. Get uncalibrated color images
        left_image_mat =cv_bridge::toCvCopy(left_image, "bgr8")->image;
        right_image_mat =cv_bridge::toCvCopy(right_image, "bgr8")->image;

        R_left_cam_to_right_cam=cv::Mat(3,3,CV_64F);
        t_left_cam_to_right_cam=cv::Mat(3,1,CV_64F);

        t_left_cam_to_right_cam.at<double>(0,0)=left_to_right_transform_->transform.translation.x;
        t_left_cam_to_right_cam.at<double>(1,0)=left_to_right_transform_->transform.translation.y;
        t_left_cam_to_right_cam.at<double>(2,0)=left_to_right_transform_->transform.translation.z;

        transformation_left_cam_to_baseline_center=cv::Mat::eye(4,4,CV_64F);

        transformation_left_cam_to_baseline_center.at<double>(0,0)=left_to_center_transform_->transform.translation.x;
        transformation_left_cam_to_baseline_center.at<double>(1,0)=left_to_center_transform_->transform.translation.y;
        transformation_left_cam_to_baseline_center.at<double>(2,0)=left_to_center_transform_->transform.translation.z;
    }

};

#endif // STEREOROS_H
