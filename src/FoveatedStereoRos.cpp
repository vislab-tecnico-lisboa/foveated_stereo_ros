#include "FoveatedStereoRos.h"
using namespace sensor_msgs;

using namespace message_filters;

FoveatedStereoRos::~FoveatedStereoRos()
{}

FoveatedStereoRos::FoveatedStereoRos(ros::NodeHandle & nh_, ros::NodeHandle & private_node_handle_) : StereoRos(nh_, private_node_handle_)
{
    std::string left_camera_info_topic;
    private_node_handle.param<std::string>("left_camera_info_topic", left_camera_info_topic, "left_camera_frame");
    left_camera_info_sub=nh_.subscribe(left_camera_info_topic,1,&FoveatedStereoRos::cameraInfoCallback, this);
    return;
}

void FoveatedStereoRos::cameraInfoCallback(const sensor_msgs::CameraInfoPtr & left_camera_info)
{
    left_camera_info_sub.shutdown();

    cameraInfoCommon(left_camera_info);

    stereo=boost::shared_ptr<FovealStereo> (new FovealStereo(left_cam_intrinsic,
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
                                                     ego_frame,
                                                     information_lower_bound,
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

    initTopics();
    ROS_INFO_STREAM("done");
}

void FoveatedStereoRos::callback(const ImageConstPtr& left_image,
                                 const ImageConstPtr& right_image,
                                 const geometry_msgs::TransformStampedConstPtr& left_to_right_transform_,
                                 const geometry_msgs::TransformStampedConstPtr& left_to_center_transform_)
{
    /*complete_stereo_calib_data scd;//=stereo_calibration->get_calibrated_transformations(l_eye_angle,r_eye_angle);
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
    scd.t_left_cam_to_right_cam.at<double>(2,0) = r_l_eye_transform.getOrigin()[2];*/
    ROS_INFO("Stereo callback...");

    ros::WallTime startTime = ros::WallTime::now();
    callbackCommon(left_image, right_image, left_to_right_transform_,left_to_center_transform_);
    StereoData stereo_data=stereo->computeStereo(left_image_mat,
                                                 right_image_mat,
                                                 R_left_cam_to_right_cam,
                                                 t_left_cam_to_right_cam,
                                                 transformation_left_cam_to_baseline_center
                                                 );//*/

    double total_elapsed = (ros::WallTime::now() - startTime).toSec();

    ROS_INFO(" TOTAL TIME STEREO:  %f sec", total_elapsed);

    //sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "mono8", csdd.disparity_image).toImageMsg();
    //disparity_image_publishernuno.publish(msg2);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", stereo_data.disparity_image).toImageMsg();
    disparity_image_publisher.publish(msg);


    publishStereoData(stereo_data, left_image->header.stamp);
    //publishCovarianceMatrices(stereo_data, left_image->header.stamp);

    total_elapsed = (ros::WallTime::now() - startTime).toSec();

    ROS_INFO(" TOTAL TIME AFTER SENDING DATA:  %f sec", total_elapsed);
}

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

    FoveatedStereoRos stereo_ros(nh, private_node_handle_);

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


