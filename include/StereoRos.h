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

using namespace sensor_msgs;

using namespace message_filters;
template <typename T>
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

    image_transport::Publisher left_cortical_image_publisher;
    image_transport::Publisher left_retinal_image_publisher;

    image_transport::Publisher right_cortical_image_publisher;
    image_transport::Publisher right_retinal_image_publisher;

    std::vector<ros::Publisher> sigma_point_clouds_publishers;
    std::string ego_frame;
    std::string left_camera_frame;
    std::string right_camera_frame;

    cv::Mat left_cam_intrinsic, right_cam_intrinsic;

    cv::Mat left_image_mat;
    cv::Mat right_image_mat;

    cv::Mat R_left_cam_to_right_cam;
    cv::Mat t_left_cam_to_right_cam;

    cv::Mat transformation_left_cam_to_baseline_center;


    void callback(const ImageConstPtr& left_image,
                  const ImageConstPtr& right_image,
                  const geometry_msgs::TransformStampedConstPtr& left_to_right_transform_,
                  const geometry_msgs::TransformStampedConstPtr& left_to_center_transform_)
    {
        ROS_INFO("Stereo callback...");

        ros::WallTime startTime = ros::WallTime::now();
        // 1. Get uncalibrated color images
        left_image_mat =cv_bridge::toCvCopy(left_image, "bgr8")->image;
        right_image_mat =cv_bridge::toCvCopy(right_image, "bgr8")->image;

        tf::Transform r_l_eye_transform;
        tf::transformMsgToTF(left_to_right_transform_->transform,r_l_eye_transform);
        R_left_cam_to_right_cam.at<double>(0,0)=r_l_eye_transform.getBasis().getColumn(0)[0];
        R_left_cam_to_right_cam.at<double>(1,0)=r_l_eye_transform.getBasis().getColumn(0)[1];
        R_left_cam_to_right_cam.at<double>(2,0)=r_l_eye_transform.getBasis().getColumn(0)[2];
        R_left_cam_to_right_cam.at<double>(0,1)=r_l_eye_transform.getBasis().getColumn(1)[0];
        R_left_cam_to_right_cam.at<double>(1,1)=r_l_eye_transform.getBasis().getColumn(1)[1];
        R_left_cam_to_right_cam.at<double>(2,1)=r_l_eye_transform.getBasis().getColumn(1)[2];
        R_left_cam_to_right_cam.at<double>(0,2)=r_l_eye_transform.getBasis().getColumn(2)[0];
        R_left_cam_to_right_cam.at<double>(1,2)=r_l_eye_transform.getBasis().getColumn(2)[1];
        R_left_cam_to_right_cam.at<double>(2,2)=r_l_eye_transform.getBasis().getColumn(2)[2];

        t_left_cam_to_right_cam.at<double>(0,0) = r_l_eye_transform.getOrigin()[0];
        t_left_cam_to_right_cam.at<double>(1,0) = r_l_eye_transform.getOrigin()[1];
        t_left_cam_to_right_cam.at<double>(2,0) = r_l_eye_transform.getOrigin()[2];

        while(nh.ok())
        {
            try
            {
                listener->waitForTransform(ego_frame, left_camera_frame, ros::Time(0), ros::Duration(1.0));
                listener->lookupTransform(ego_frame, left_camera_frame,
                                          ros::Time(0), l_eye_transform);
            }
            catch (tf::TransformException &ex)
            {
                //ROS_ERROR("%s",ex.what());
                //ros::Duration(1.0).sleep();
                continue;
            }
            break;
        }

        Eigen::Affine3d left_to_center_eigen;

        tf::transformTFToEigen (l_eye_transform, left_to_center_eigen );
        cv::Mat left_to_center = cv::Mat::eye(4,4,CV_64F);
        cv::eigen2cv(left_to_center_eigen.matrix(),transformation_left_cam_to_baseline_center);

        StereoData stereo_data=stereo->computeStereo(left_image_mat,
                                                     right_image_mat,
                                                     R_left_cam_to_right_cam,
                                                     t_left_cam_to_right_cam,
                                                     transformation_left_cam_to_baseline_center
                                                     );

        double total_elapsed = (ros::WallTime::now() - startTime).toSec();

        ROS_INFO(" TOTAL TIME STEREO:  %f sec", total_elapsed);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", stereo_data.disparity_image).toImageMsg();
        disparity_image_publisher.publish(msg);

        publishStereoData(stereo_data, left_image->header.stamp);
        //publishCovarianceMatrices(stereo_data, left_image->header.stamp);

        total_elapsed = (ros::WallTime::now() - startTime).toSec();

        ROS_INFO(" TOTAL TIME AFTER SENDING DATA:  %f sec", total_elapsed);
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoPtr & left_camera_info)
    {
        left_camera_info_sub.shutdown();
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

        int fovea_rows;
        int fovea_columns;
        private_node_handle.param("fovea_rows", fovea_rows, 10);
        private_node_handle.param("fovea_columns", fovea_columns, 10);


        private_node_handle.param<std::string>("ego_frame", ego_frame, "ego_frame");
        private_node_handle.param<std::string>("left_camera_frame", left_camera_frame, "left_camera_frame");
        private_node_handle.param<std::string>("right_camera_frame", right_camera_frame, "right_camera_frame");
        private_node_handle.param("L", L, 1.0);
        private_node_handle.param("alpha", alpha, 1.0);
        private_node_handle.param("beta", beta, 2.0);
        private_node_handle.param("ki", ki, 1.0);
        private_node_handle.param("scaling_factor", scaling_factor, 1.0);

        scaling_factor=(1/(2*3.0))*(1/(2*3.0));
        ROS_INFO_STREAM("sectors: "<<sectors);
        ROS_INFO_STREAM("rings: "<<rings);
        ROS_INFO_STREAM("min_radius: "<<min_radius);
        ROS_INFO_STREAM("interp: "<<interp);
        ROS_INFO_STREAM("sp: "<<sp);
        ROS_INFO_STREAM("full: "<<full);
        ROS_INFO_STREAM("fovea_rows: "<<fovea_rows);
        ROS_INFO_STREAM("fovea_columns: "<<fovea_columns);
        ROS_INFO_STREAM("ego_frame: "<<ego_frame);
        ROS_INFO_STREAM("left_camera_frame: "<<left_camera_frame);
        ROS_INFO_STREAM("right_camera_frame: "<<right_camera_frame);
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
        left_cam_intrinsic = cv::Mat::eye(3,3,CV_64F);
        left_cam_intrinsic.at<double>(0,0) = left_camera_info->K.at(0);
        left_cam_intrinsic.at<double>(1,1) = left_camera_info->K.at(4);
        left_cam_intrinsic.at<double>(0,2) = left_camera_info->K.at(2);
        left_cam_intrinsic.at<double>(1,2) = left_camera_info->K.at(5);

        right_cam_intrinsic = cv::Mat::eye(3,3,CV_64F);
        right_cam_intrinsic.at<double>(0,0) = right_camera_info->K.at(0);
        right_cam_intrinsic.at<double>(1,1) = right_camera_info->K.at(4);
        right_cam_intrinsic.at<double>(0,2) = right_camera_info->K.at(2);
        right_cam_intrinsic.at<double>(1,2) = right_camera_info->K.at(5);
        width=(unsigned int)left_camera_info->width;
        height=(unsigned int)left_camera_info->height;

        //rings=sqrt((width*height/(2*M_PI)))*log(0.5*width/min_radius);
        //sectors=round(width*height/rings);
        //rings=round(rings);
        stereo=boost::shared_ptr<T> (new T(left_cam_intrinsic,
                                           right_cam_intrinsic,
                                           width,
                                           height,
                                           cv::Point2i(width/2.0,
                                                       height/2.0),
                                           rings,
                                           min_radius,
                                           interp,
                                           full,
                                           sectors,
                                           sp,
                                           fovea_rows,
                                           fovea_columns,
                                           ego_frame,
                                           L,
                                           alpha,
                                           ki,
                                           beta,
                                           scaling_factor,
                                           number_of_disparities,
                                           pre_filter_cap,
                                           sad_window_size,
                                           P1,
                                           P2,
                                           min_disparity,
                                           uniqueness_ratio,
                                           speckle_window_size,
                                           speckle_range,
                                           disp_12_max_diff,
                                           full_dp,
                                           ignore_border_left
                                           )
                                     );

        rgb_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("rgbd_pcl", 10);
        mean_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("mean_pcl", 10);
        uncertainty_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("uncertainty_pcl", 10);
        disparity_image_publisher = it_.advertise("/vizzy/disparity", 3);


        left_retinal_image_publisher = it_.advertise("left_retinal_image",2);
        left_cortical_image_publisher = it_.advertise("left_cortical_image",2);

        right_retinal_image_publisher = it_.advertise("right_retinal_image",2);
        right_cortical_image_publisher = it_.advertise("right_cortical_image",2);

        stereo_data_publisher = nh.advertise<foveated_stereo_ros::StereoData>("stereo_data", 1);

        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("covariances", 1);

        left_image_sub=boost::shared_ptr<message_filters::Subscriber<Image> > (new message_filters::Subscriber<Image>(nh, "left_image", 10));
        right_image_sub=boost::shared_ptr<message_filters::Subscriber<Image> > (new message_filters::Subscriber<Image>(nh, "right_image", 10));

        left_to_right_sub=boost::shared_ptr<message_filters::Subscriber<geometry_msgs::TransformStamped> > (new message_filters::Subscriber<geometry_msgs::TransformStamped>(nh, "left_to_right_tf", 10));
        left_to_center_sub=boost::shared_ptr<message_filters::Subscriber<geometry_msgs::TransformStamped> > (new message_filters::Subscriber<geometry_msgs::TransformStamped>(nh, "left_to_center_tf", 10));
        sync=boost::shared_ptr<Synchronizer<MySyncPolicy> > (new Synchronizer<MySyncPolicy>(MySyncPolicy(10), *left_image_sub, *right_image_sub, *left_to_right_sub, *left_to_center_sub));
        sync->registerCallback(boost::bind(&StereoRos<T>::callback, this, _1, _2, _3, _4));

        ROS_INFO_STREAM("done");
    }
public:

    boost::shared_ptr<T> stereo;

    typedef sync_policies::ApproximateTime<Image, Image, geometry_msgs::TransformStamped, geometry_msgs::TransformStamped> MySyncPolicy;
    boost::shared_ptr<message_filters::Subscriber<Image> > left_image_sub;
    boost::shared_ptr<message_filters::Subscriber<Image> > right_image_sub;
    boost::shared_ptr<message_filters::Subscriber<geometry_msgs::TransformStamped> > left_to_right_sub;
    boost::shared_ptr<message_filters::Subscriber<geometry_msgs::TransformStamped> > left_to_center_sub;

    boost::shared_ptr<Synchronizer<MySyncPolicy> >sync;

    ~StereoRos()
    {}

    StereoRos(ros::NodeHandle & nh_, ros::NodeHandle & private_node_handle_) :
        nh(nh_),
        private_node_handle(private_node_handle_),
        it_(nh_),
        listener(new tf::TransformListener(ros::Duration(2.0)))
    {
        R_left_cam_to_right_cam=cv::Mat(3,3,CV_64F);
        t_left_cam_to_right_cam=cv::Mat(3,1,CV_64F);

        std::string left_camera_info_topic;
        private_node_handle.param<std::string>("left_camera_info_topic", left_camera_info_topic, "left_camera_frame");
        left_camera_info_sub=nh_.subscribe(left_camera_info_topic,1,&StereoRos<T>::cameraInfoCallback, this);

        return;
    }

    void publishStereoData(StereoData & sdd, const ros::Time & time)
    {
        sensor_msgs::PointCloud2 rgb_point_cloud_msg;
        pcl::toROSMsg(sdd.point_cloud, rgb_point_cloud_msg);
        rgb_point_cloud_msg.is_dense=false;
        rgb_point_cloud_msg.header.stamp=time;
        rgb_point_cloud_msg.header.frame_id=left_camera_frame;
        rgb_point_cloud_publisher.publish(rgb_point_cloud_msg);

        sensor_msgs::PointCloud2 mean_point_cloud_msg;
        pcl::toROSMsg(sdd.mean_point_cloud, mean_point_cloud_msg);
        mean_point_cloud_msg.is_dense=false;
        mean_point_cloud_msg.header.stamp=time;
        mean_point_cloud_msg.header.frame_id=left_camera_frame;
        mean_point_cloud_publisher.publish(mean_point_cloud_msg);

        sensor_msgs::PointCloud2 uncertainty_point_cloud_viz_msg;
        pcl::toROSMsg(sdd.point_cloud_uncertainty_viz, uncertainty_point_cloud_viz_msg);
        uncertainty_point_cloud_viz_msg.is_dense=false;
        uncertainty_point_cloud_viz_msg.header.stamp=time;
        uncertainty_point_cloud_viz_msg.header.frame_id=left_camera_frame;
        uncertainty_point_cloud_publisher.publish(uncertainty_point_cloud_viz_msg);

        sensor_msgs::PointCloud2 uncertainty_point_cloud_msg;
        pcl::toROSMsg(sdd.point_cloud_uncertainty, uncertainty_point_cloud_msg);
        uncertainty_point_cloud_msg.is_dense=false;
        uncertainty_point_cloud_msg.header.stamp=time;
        uncertainty_point_cloud_msg.header.frame_id=left_camera_frame;

        foveated_stereo_ros::StereoData stereo_msg;
        stereo_msg.point_clouds.rgb_point_cloud=rgb_point_cloud_msg;
        stereo_msg.header=rgb_point_cloud_msg.header;
        stereo_msg.point_clouds.uncertainty_point_cloud=uncertainty_point_cloud_msg;
        cv::Mat informations_determinants=sdd.getInformationsDeterminants();

        for(unsigned int r=0; r<sdd.information_3d.size(); ++r)
        {
            for(unsigned int c=0; c<sdd.information_3d[r].size();++c)
            {
                //if(isnan(informations_determinants.at<double>(r,c))||isnan(log(informations_determinants.at<double>(r,c)))||log(informations_determinants.at<double>(r,c))<information_lower_bound)

                if(isnan(informations_determinants.at<double>(r,c))||isnan(log(informations_determinants.at<double>(r,c))))
                    continue;
                //foveated_stereo_ros::Covariance covariance_msg;
                foveated_stereo_ros::Information information_msg;

                Eigen::Matrix<double,3,3> information_eigen;
                cv2eigen(sdd.information_3d[r][c],information_eigen);

                for(int i=0; i<3; ++i)
                {
                    for(int j=0; j<3; ++j)
                    {
                        int index=j+i*3;
                        //covariance_msg.covariance[index]=sdd.cov_3d[r][c].at<double>(i,j);
                        information_msg.information[index]=sdd.information_3d[r][c].at<double>(i,j);
                    }
                }

                //stereo_msg.covariances.push_back(covariance_msg);
                stereo_msg.informations.push_back(information_msg);
            }
        }

        stereo_data_publisher.publish(stereo_msg);

        /*for(int i=0; i<sdd.sigma_point_clouds.size();++i)
        {
            sensor_msgs::PointCloud2 point_cloud_msg;

            pcl::toROSMsg(sdd.sigma_point_clouds[i],point_cloud_msg);
            point_cloud_msg.is_dense=false;
            point_cloud_msg.header.stamp=time;

            sigma_point_clouds_publishers[i].publish(point_cloud_msg);
        }*/



        // Publish retinal and cortical images
        sensor_msgs::ImagePtr left_retinal_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", sdd.left_retinal_image).toImageMsg();
        left_retinal_image_publisher.publish(left_retinal_image_msg);

        sensor_msgs::ImagePtr left_cortical_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", sdd.left_cortical_image).toImageMsg();
        left_cortical_image_publisher.publish(left_cortical_image_msg);

        sensor_msgs::ImagePtr right_retinal_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", sdd.right_retinal_image).toImageMsg();
        right_retinal_image_publisher.publish(right_retinal_image_msg);

        sensor_msgs::ImagePtr right_cortical_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", sdd.right_cortical_image).toImageMsg();
        right_cortical_image_publisher.publish(right_cortical_image_msg);
    }

    void publishCovarianceMatrices(StereoData & sdd, const ros::Time & time)
    {

        visualization_msgs::MarkerArray marker_array;
        double scale=1.0;
        int jump=5;

        cv::Mat informations_determinants=sdd.getInformationsDeterminants();

        for(int r=0; r<sdd.information_3d.size();r=r+jump)
        {
            for(int c=0; c<sdd.information_3d[r].size();c=c+jump)
                //for(int c=ignore_border_left; c<sdd.information_3d[r].size();c=c+jump)
            {
                if(isnan(informations_determinants.at<double>(r,c))||isnan(log(informations_determinants.at<double>(r,c))))

                    //if(isnan(informations_determinants.at<double>(r,c))||isnan(log(informations_determinants.at<double>(r,c)))||log(informations_determinants.at<double>(r,c))<information_lower_bound)
                {
                    continue;
                }

                Eigen::Matrix<double,3,3> covariance_eigen;
                cv2eigen(sdd.cov_3d[r][c],covariance_eigen);

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance_eigen); // last one is the greatest eigen value

                Eigen::Quaternion<double> q(-eig.eigenvectors());
                q.normalize();

                visualization_msgs::Marker marker;
                // Set the frame ID and timestamp.  See the TF tutorials for information on these.
                marker.header.frame_id = left_camera_frame;
                marker.header.stamp = ros::Time::now();
                marker.ns = "covariances";
                marker.id = c+r*sdd.cov_3d[r].size();
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;

                // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
                marker.pose.position.x = sdd.point_cloud_cartesian.at<cv::Vec3d>(r,c)[0];
                marker.pose.position.y = sdd.point_cloud_cartesian.at<cv::Vec3d>(r,c)[1];
                marker.pose.position.z = sdd.point_cloud_cartesian.at<cv::Vec3d>(r,c)[2];
                marker.pose.orientation.x = q.x();
                marker.pose.orientation.y = q.y();
                marker.pose.orientation.z = q.z();
                marker.pose.orientation.w = q.w();

                // Set the scale of the marker -- 1x1x1 here means 1m on a side
                marker.scale.x = scale*eig.eigenvalues()(0);
                marker.scale.y = scale*eig.eigenvalues()(1);
                marker.scale.z = scale*eig.eigenvalues()(2);

                // Set the color -- be sure to set alpha to something non-zero!
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0;

                marker.lifetime = ros::Duration(1.2);
                marker_array.markers.push_back(marker);
            }
        }
        marker_pub.publish(marker_array);
    }

};

#endif // STEREOROS_H
