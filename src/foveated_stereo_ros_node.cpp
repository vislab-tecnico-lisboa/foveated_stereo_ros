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

using namespace sensor_msgs;

using namespace message_filters;

class FoveatedStereoNode
{
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

    std::vector<ros::Publisher> sigma_point_clouds_publishers;


    boost::shared_ptr<message_filters::Subscriber<Image> > left_image_sub;
    boost::shared_ptr<message_filters::Subscriber<Image> > right_image_sub;

    boost::shared_ptr<Synchronizer<MySyncPolicy> >sync;

    boost::shared_ptr<stereo_calib> stereo_calibration;
    boost::shared_ptr<Stereo> ego_sphere;

    ~FoveatedStereoNode()
    {}

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
                       bool high_pass_) : nh(nh_), it_(nh_)
    {
        left_image_sub=boost::shared_ptr<message_filters::Subscriber<Image> > (new message_filters::Subscriber<Image>(nh, "left_image", 10));
        right_image_sub=boost::shared_ptr<message_filters::Subscriber<Image> > (new message_filters::Subscriber<Image>(nh, "right_image", 10));

        image_pub_ = it_.advertise("/vizzy/disparity", 3);


        tf::StampedTransform transform;

        try
        {
            listener.waitForTransform("/l_camera_vision_link", "/r_camera_vision_link", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform("/l_camera_vision_link", "/r_camera_vision_link",
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            exit(-1);
        }

        tf::Vector3 origin=transform.getOrigin();

        float baseline=origin.length(); // meters

        sync=boost::shared_ptr<Synchronizer<MySyncPolicy> > (new Synchronizer<MySyncPolicy>(MySyncPolicy(10), *left_image_sub, *right_image_sub));
        sync->registerCallback(boost::bind(&FoveatedStereoNode::callback, this, _1, _2));
        stereo_calibration=boost::shared_ptr<stereo_calib> (new stereo_calib(fillStereoCalibParams(baseline)));

        sensor_msgs::CameraInfoConstPtr left_camera_info=ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/vizzy/l_camera/camera_info", ros::Duration(30));

        //float fx_=left_camera_info->K.at(0); // fx
        //float fy_=left_camera_info->K.at(4); // fy
        float focal_distance=left_camera_info->K.at(4); // fy


        //float focal_pixel = (image_width_in_pixels * 0.5) / tan(FOV * 0.5);
        //float tan(FOV*0.5)=(left_camera_info->width * 0.5) /
        ego_sphere=boost::shared_ptr<Stereo> (new Stereo((int)left_camera_info->width,
                                                         (int)left_camera_info->height,
                                                         cv::Point2i(left_camera_info->width/2.0,
                                                                     left_camera_info->height/2.0),
                                                         rings_,
                                                         min_radius_,
                                                         interp_,
                                                         full_,
                                                         sectors_,
                                                         sp_,
                                                         disparities_,
                                                         sigma_,
                                                         occ_likelihood_,
                                                         pole_,
                                                         focal_distance,
                                                         spherical_angle_bins_,
                                                         shell_radius_,
                                                         high_pass_)
                                              );


        point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("stereo", 10);
        for(int i=0; i<9; ++i)
        {
            std::stringstream ss;
            ss << i;
            std::string str = ss.str();
            sigma_point_clouds_publishers.push_back(nh.advertise<sensor_msgs::PointCloud2>("sigma_point_clouds_"+str, 10));
        }

        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("covariances", 1);
        return;
    }



    stereo_calib_params fillStereoCalibParams(float & baseline)
    {
        ROS_INFO("Getting cameras' paremeters");
        sensor_msgs::CameraInfoConstPtr left_camera_info=ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/vizzy/l_camera/camera_info", ros::Duration(30));
        sensor_msgs::CameraInfoConstPtr right_camera_info=ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/vizzy/r_camera/camera_info", ros::Duration(30));

        stereo_calib_params params;
        params.left_cam_cx=left_camera_info->K.at(2);
        params.left_cam_cy=left_camera_info->K.at(5);
        params.left_cam_fx=left_camera_info->K.at(0);
        params.left_cam_fy=left_camera_info->K.at(4);
        params.left_cam_resx=left_camera_info->width;
        params.left_cam_resy=left_camera_info->height;

        params.right_cam_cx=right_camera_info->K.at(2);
        params.right_cam_cy=right_camera_info->K.at(5);
        params.right_cam_fx=right_camera_info->K.at(0);
        params.right_cam_fy=right_camera_info->K.at(4);
        params.right_cam_resx=right_camera_info->width;
        params.right_cam_resy=right_camera_info->height;

        params.baseline=baseline; // meters
        params.encoders_measurements_noise=0.0000000175;
        params.encoders_state_noise=1.0;
        params.encoders_transition_noise=0.005;
        params.features_measurements_noise=25;
        params.matching_threshold=0.35;
        params.max_number_of_features=250;
        params.min_number_of_features=2;
        params.number_fixed_state_params=5;
        params.number_measurements=6;
        ROS_INFO("Done.");

        return params;
    }

    void callback(const ImageConstPtr& left_image,
                  const ImageConstPtr& right_image)
    {
        ros::WallTime startTime = ros::WallTime::now();

        // 1. Get uncalibrated color images
        cv::Mat left_image_mat =cv_bridge::toCvCopy(left_image, "bgr8")->image;
        cv::Mat right_image_mat =cv_bridge::toCvCopy(right_image, "bgr8")->image;

        // 2. Get eye angles with respect to eyes center
        try
        {
            listener.waitForTransform("/eyes_center_vision_link", "/l_camera_vision_link", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform("/eyes_center_vision_link", "/l_camera_vision_link",
                                     ros::Time(0), l_eye_transform);
            /*listener.waitForTransform("/eyes_center_vision_link", "/r_camera_vision_link", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform("/eyes_center_vision_link", "/r_camera_vision_link",
                                     ros::Time(0), r_eye_transform);*/
            listener.waitForTransform("/l_camera_vision_link", "/r_camera_vision_link", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform("/l_camera_vision_link", "/r_camera_vision_link",
                                     ros::Time(0), r_l_eye_transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            exit(-1);
        }

        /*double roll, pitch, yaw;
        tf::Matrix3x3(tf::Quaternion(
                          l_eye_transform.getRotation().getX(),
                          l_eye_transform.getRotation().getY(),
                          l_eye_transform.getRotation().getZ(),
                          l_eye_transform.getRotation().getW()
                          )).getRPY(roll, pitch, yaw);
        double l_eye_angle=-yaw;

        tf::Matrix3x3(tf::Quaternion(
                          r_eye_transform.getRotation().getX(),
                          r_eye_transform.getRotation().getY(),
                          r_eye_transform.getRotation().getZ(),
                          r_eye_transform.getRotation().getW()
                          )).getRPY(roll, pitch, yaw);
        double r_eye_angle=-yaw;
        //ROS_INFO_STREAM("l_eye_angle:"<<l_eye_angle <<"   r_eye_angle:"<<r_eye_angle);

        // 3. calibrate given angles
        stereo_calibration->calibrate(left_image_mat,
                                      right_image_mat,
                                      l_eye_angle,
                                      r_eye_angle);
        */

        cv::Mat left_to_center = Mat::eye(4,4,CV_64F);
        left_to_center.at<double>(0,0) = l_eye_transform.getBasis().getColumn(0)[0];
        left_to_center.at<double>(1,0) = l_eye_transform.getBasis().getColumn(0)[1];
        left_to_center.at<double>(2,0) = l_eye_transform.getBasis().getColumn(0)[2];
        left_to_center.at<double>(0,1) = l_eye_transform.getBasis().getColumn(1)[0];
        left_to_center.at<double>(1,1) = l_eye_transform.getBasis().getColumn(1)[1];
        left_to_center.at<double>(2,1) = l_eye_transform.getBasis().getColumn(1)[2];
        left_to_center.at<double>(0,2) = l_eye_transform.getBasis().getColumn(2)[0];
        left_to_center.at<double>(1,2) = l_eye_transform.getBasis().getColumn(2)[1];
        left_to_center.at<double>(2,2) = l_eye_transform.getBasis().getColumn(2)[2];
        left_to_center.at<double>(0,3) = l_eye_transform.getOrigin()[0];
        left_to_center.at<double>(1,3) = l_eye_transform.getOrigin()[1];
        left_to_center.at<double>(2,3) = l_eye_transform.getOrigin()[2];

        stereo_calib_data scd;//=stereo_calibration->get_calibrated_transformations(l_eye_angle,r_eye_angle);
        scd.R_left_cam_to_right_cam=Mat(3,3,CV_64F);
        scd.t_left_cam_to_right_cam=Mat(3,1,CV_64F);

        scd.R_left_cam_to_right_cam.at<double>(0,0)=r_l_eye_transform.getBasis().getColumn(0)[0];
        scd.R_left_cam_to_right_cam.at<double>(1,0)=r_l_eye_transform.getBasis().getColumn(0)[1];
        scd.R_left_cam_to_right_cam.at<double>(2,0)=r_l_eye_transform.getBasis().getColumn(0)[2];
        scd.R_left_cam_to_right_cam.at<double>(0,1)=r_l_eye_transform.getBasis().getColumn(1)[0];
        scd.R_left_cam_to_right_cam.at<double>(1,1)=r_l_eye_transform.getBasis().getColumn(1)[1];
        scd.R_left_cam_to_right_cam.at<double>(2,1)=r_l_eye_transform.getBasis().getColumn(1)[2];
        scd.R_left_cam_to_right_cam.at<double>(0,2)=r_l_eye_transform.getBasis().getColumn(2)[0];
        scd.R_left_cam_to_right_cam.at<double>(1,2)=r_l_eye_transform.getBasis().getColumn(2)[1];
        scd.R_left_cam_to_right_cam.at<double>(2,2)=r_l_eye_transform.getBasis().getColumn(2)[2];

        scd.t_left_cam_to_right_cam.at<double>(0,0) = r_l_eye_transform.getOrigin()[0];
        scd.t_left_cam_to_right_cam.at<double>(1,0) = r_l_eye_transform.getOrigin()[1];
        scd.t_left_cam_to_right_cam.at<double>(2,0) = r_l_eye_transform.getOrigin()[2];

        StereoData stereo_data=ego_sphere->computeStereo(left_image_mat,
                                                         right_image_mat,
                                                         scd.R_left_cam_to_right_cam,
                                                         scd.t_left_cam_to_right_cam,
                                                         stereo_calibration->csc.LeftCalibMat,
                                                         stereo_calibration->csc.RightCalibMat,
                                                         left_to_center
                                                         );//*/

        double total_elapsed = (ros::WallTime::now() - startTime).toSec();

        ROS_INFO(" TOTAL TIME STEREO:  %f sec", total_elapsed);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", stereo_data.disparity_image).toImageMsg();
        image_pub_.publish(msg);
        publishPointClouds(stereo_data, left_image->header.stamp);
        publishCovarianceMatrices(stereo_data, left_image->header.stamp);

    }

    void publishPointClouds(StereoData & sdd, const ros::Time & time)
    {
        sensor_msgs::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(sdd.point_cloud, point_cloud_msg);
        point_cloud_msg.is_dense=false;
        point_cloud_msg.header.stamp=time;
        point_cloud_publisher.publish(point_cloud_msg);

        for(int i=0; i<sdd.sigma_point_clouds.size();++i)
        {
            sensor_msgs::PointCloud2 point_cloud_msg;

            pcl::toROSMsg(sdd.sigma_point_clouds[i],point_cloud_msg);
            point_cloud_msg.is_dense=false;
            point_cloud_msg.header.stamp=time;

            sigma_point_clouds_publishers[i].publish(point_cloud_msg);
        }
    }

    void publishCovarianceMatrices(StereoData & sdd, const ros::Time & time)
    {
        visualization_msgs::MarkerArray marker_array;

        for(int r=0; r<sdd.cov_3d.size();++r)
        {
            for(int c=0; c<sdd.cov_3d[r].size();++c)
            {
                if(sdd.point_cloud_cartesian.at<cv::Vec3d>(r,c)[2]>3.0)
                    continue;

                if(cv::norm(sdd.cov_3d[r][c])<0.1 || cv::norm(sdd.cov_3d[r][c])>1000.0)
                    continue;

                if(!cv::checkRange(sdd.cov_3d[r][c])||!cv::checkRange(sdd.mean_3d.at<cv::Vec3d>(r,c)))
                {
                    std::cout << "yaicks" << std::endl;
                    std::cout << sdd.cov_3d[r][c] << std::endl;
                    continue;
                }

                if(sdd.mean_3d.at<cv::Vec3d>(r,c)[2]<0)
                    continue;

                cv::Mat eigen_values;
                cv::Mat eigen_vectors;

                cv::eigen(sdd.cov_3d[r][c],  eigen_values,  eigen_vectors);
                /*std::cout << sdd.cov_3d[r][c] << std::endl;
                std::cout << eigen_values.at<double>(0) << " " <<
                             eigen_values.at<double>(1) << " " <<
                             eigen_values.at<double>(2) << std::endl;
                exit(-1);*/
                visualization_msgs::Marker marker;

                // Set the frame ID and timestamp.  See the TF tutorials for information on these.
                marker.header.frame_id = "eyes_center_vision_link";
                //marker.header.stamp = ros::Time::now();

                marker.ns = "covariances";
                marker.id = c+r*sdd.cov_3d[r].size();

                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;

                // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
                marker.pose.position.x = sdd.mean_3d.at<cv::Vec3d>(r,c)[0];
                marker.pose.position.y = sdd.mean_3d.at<cv::Vec3d>(r,c)[1];
                marker.pose.position.z = sdd.mean_3d.at<cv::Vec3d>(r,c)[2];
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;

                // Set the scale of the marker -- 1x1x1 here means 1m on a side
                //marker.scale.x = 0.1*eigen_values.at<double>(0);
                //marker.scale.y = 0.1*eigen_values.at<double>(1);
                //marker.scale.z = 0.1*eigen_values.at<double>(2);

                marker.scale.x = 0.1;
                marker.scale.y = 0.1;
                marker.scale.z = 0.1;

                // Set the color -- be sure to set alpha to something non-zero!
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0;

                marker.lifetime = ros::Duration();
                marker_array.markers.push_back(marker);
            }
        }
        marker_pub.publish(marker_array);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_node");

    ros::NodeHandle nh;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");

    // Declare variables that can be modified by launch file or command line.
    int rate;

    private_node_handle_.param("rate", rate, 100);

    int sectors;
    int rings;
    double min_radius;
    int interp;
    int full;
    int sp;
    bool high_pass;
    double pole;
    double occ_likelihood;
    double sigma;
    int disparities;
    int min_disparity;
    int spherical_angle_bins;
    double shell_radius;

    private_node_handle_.param("sectors", sectors, 100);
    private_node_handle_.param("rings", rings, 100);
    private_node_handle_.param("min_radius", min_radius, 1.0);
    private_node_handle_.param("interp", interp, 1);
    private_node_handle_.param("sp", sp, 0);
    private_node_handle_.param("full", full, 1);
    private_node_handle_.param("high_pass", high_pass, false);
    private_node_handle_.param("pole", pole, 0.8);
    private_node_handle_.param("occ_likelihood", occ_likelihood, 0.1);
    private_node_handle_.param("sigma", sigma, 3.0);
    private_node_handle_.param("disparities", disparities, 60);
    private_node_handle_.param("min_disparity", min_disparity, 1);
    private_node_handle_.param("spherical_angle_bins", spherical_angle_bins, 50);
    private_node_handle_.param("shell_radius", shell_radius, 1.0);

    std::vector<double> disparities_vec;

    for(int i=min_disparity; i<disparities; ++i)
    {
        disparities_vec.push_back(i);
    }

    ROS_INFO_STREAM("sectors: "<<sectors);
    ROS_INFO_STREAM("rings: "<<rings);
    ROS_INFO_STREAM("min_radius: "<<min_radius);
    ROS_INFO_STREAM("interp: "<<interp);
    ROS_INFO_STREAM("sp: "<<sp);
    ROS_INFO_STREAM("full: "<<full);
    ROS_INFO_STREAM("high_pass: "<<high_pass);
    ROS_INFO_STREAM("pole: "<<pole);
    ROS_INFO_STREAM("occ_likelihood: "<<occ_likelihood);
    ROS_INFO_STREAM("sigma: "<<sigma);
    ROS_INFO_STREAM("disparities: "<<disparities);
    ROS_INFO_STREAM("min_disparity: "<<min_disparity);
    ROS_INFO_STREAM("spherical_angle_bins: "<<spherical_angle_bins);
    ROS_INFO_STREAM("shell_radius: "<<shell_radius);

    FoveatedStereoNode ego_sphere(nh,
                                  rings,
                                  min_radius,
                                  interp,
                                  full,
                                  sectors,
                                  sp,
                                  disparities_vec,
                                  sigma,
                                  occ_likelihood,
                                  pole,
                                  spherical_angle_bins,
                                  shell_radius,
                                  high_pass
                                  );

    // Tell ROS how fast to run this node.
    ros::Rate r(rate);

    // Main loop.
    while (nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
} // end


