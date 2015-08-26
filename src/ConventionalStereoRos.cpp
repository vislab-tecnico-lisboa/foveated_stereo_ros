#include "ConventionalStereoRos.h"
using namespace sensor_msgs;

using namespace message_filters;

ConventionalStereoRos::~ConventionalStereoRos()
{}

ConventionalStereoRos::ConventionalStereoRos(ros::NodeHandle & nh_, ros::NodeHandle & private_node_handle_) : StereoRos(nh_,private_node_handle_)
{
    std::string left_camera_info_topic;
    private_node_handle.param<std::string>("left_camera_info_topic", left_camera_info_topic, "left_camera_frame");
    ROS_INFO_STREAM("TOPIC:"<<left_camera_info_topic);
    left_camera_info_sub=nh_.subscribe(left_camera_info_topic,1,&ConventionalStereoRos::cameraInfoCallback, this);
    return;
}

void ConventionalStereoRos::cameraInfoCallback(const sensor_msgs::CameraInfoPtr & left_camera_info)
{
    left_camera_info_sub.shutdown();
    cameraInfoCommon(left_camera_info);
    std::cout << "width:" <<width << std::endl;
    std::cout << "height:" <<height << std::endl;

    stereo=boost::shared_ptr<ConventionalStereo> (new ConventionalStereo(left_cam_intrinsic,
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

void ConventionalStereoRos::callback(const ImageConstPtr& left_image,
                                     const ImageConstPtr& right_image,
                                     const geometry_msgs::TransformStampedConstPtr& left_to_right_transform_,
                                     const geometry_msgs::TransformStampedConstPtr& left_to_center_transform_)
{
    ROS_INFO("Stereo callback...");

    ros::WallTime startTime = ros::WallTime::now();
    callbackCommon(left_image, right_image, left_to_right_transform_,left_to_center_transform_);
    StereoData stereo_data=stereo->computeStereo(left_image_mat,
                                                 right_image_mat,
                                                 R_left_cam_to_right_cam,
                                                 t_left_cam_to_right_cam,
                                                 transformation_left_cam_to_baseline_center
                                                 );

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


/*void ConventionalStereoRos::callback(const ImageConstPtr& left_image,
                                     const ImageConstPtr& right_image)
{
    ROS_INFO("Stereo callback...");
    ros::WallTime startTime = ros::WallTime::now();

    // 1. Get uncalibrated color images
    cv::Mat left_image_mat =cv_bridge::toCvCopy(left_image, "bgr8")->image;
    cv::Mat right_image_mat =cv_bridge::toCvCopy(right_image, "bgr8")->image;

    // 2. Get eye angles with respect to eyes center
    try
    {
        listener->waitForTransform(ego_frame, left_camera_frame, ros::Time(0), ros::Duration(1.0));
        listener->lookupTransform(ego_frame, left_camera_frame,
                                  ros::Time(0), l_eye_transform);
        listener->waitForTransform(right_camera_frame, left_camera_frame, ros::Time(0), ros::Duration(1.0));
        listener->lookupTransform(right_camera_frame, left_camera_frame,
                                  ros::Time(0), r_l_eye_transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }


    Mat stereo_encoders = Mat::zeros(6,1,CV_64F);

    // 3. calibrate given angles
    ROS_INFO("Calibrate stereo...");
    stereo_calibration->calibrate(left_image_mat,
                                  right_image_mat,
                                  stereo_encoders);

    ROS_INFO("Done.");

    //get the calibrated transformations between the two cameras
    complete_stereo_calib_data scd;
    scd =  stereo_calibration->get_calibrated_transformations(stereo_encoders);



    StereoData stereo_data=stereo->computeStereo(left_image_mat,
                                                 right_image_mat,
                                                 scd.R_left_cam_to_right_cam,
                                                 scd.t_left_cam_to_right_cam,
                                                 scd.transformation_left_cam_to_baseline_center
                                                 );//
	
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
}*/

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

    ConventionalStereoRos stereo_ros(nh, private_node_handle_);

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


