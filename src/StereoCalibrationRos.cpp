#include "StereoCalibrationRos.h"

StereoCalibrationRos::StereoCalibrationRos()
{
}
StereoCalibrationRos::StereoCalibrationRos(ros::NodeHandle & nh_, ros::NodeHandle & private_node_handle_) : nh(nh_),private_node_handle(private_node_handle_)
{
    left_image_sub=boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > (new message_filters::Subscriber<sensor_msgs::Image>(nh, "left_image", 10));
    right_image_sub=boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > (new message_filters::Subscriber<sensor_msgs::Image>(nh, "right_image", 10));
    joint_state_sub=boost::shared_ptr<message_filters::Subscriber<sensor_msgs::JointState> > (new message_filters::Subscriber<sensor_msgs::JointState>(nh, "joint_state", 10));

    sync=boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > (new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *left_image_sub, *right_image_sub, *joint_state_sub));
    sync->registerCallback(boost::bind(&StereoCalibrationRos::callback, this, _1, _2, _3));

    right_to_left_pub=nh.advertise<geometry_msgs::TransformStamped>("right_to_left_tf", 1);
}

void StereoCalibrationRos::callback(const sensor_msgs::ImageConstPtr& left_image,
                                    const sensor_msgs::ImageConstPtr& right_image,
                                    const sensor_msgs::JointStateConstPtr& joint_states)
{
    ROS_INFO("Stereo calibration callback...");
    ros::WallTime startTime = ros::WallTime::now();

    // 1. Get uncalibrated color images
    cv::Mat left_image_mat =cv_bridge::toCvCopy(left_image, "bgr8")->image;
    cv::Mat right_image_mat =cv_bridge::toCvCopy(right_image, "bgr8")->image;

    cv::Mat stereo_encoders = cv::Mat::zeros(6,1,CV_64F);

    // 3. calibrate given angles
    ROS_INFO("Calibrate stereo...");
    stereo_calibration->calibrate(left_image_mat,
                                  right_image_mat,
                                  stereo_encoders);

    ROS_INFO("Done.");

    //get the calibrated transformations between the two cameras
    complete_stereo_calib_data scd;
    scd =  stereo_calibration->get_calibrated_transformations(stereo_encoders);

    geometry_msgs::TransformStamped transform_stamped;

    cv::Mat t_right_cam_to_left_cam_quaternion=Quaternion(scd.R_right_cam_to_left_cam);

    transform_stamped.transform.rotation.w=t_right_cam_to_left_cam_quaternion.at<double>(0,0);
    transform_stamped.transform.rotation.x=t_right_cam_to_left_cam_quaternion.at<double>(1,0);
    transform_stamped.transform.rotation.y=t_right_cam_to_left_cam_quaternion.at<double>(2,0);
    transform_stamped.transform.rotation.z=t_right_cam_to_left_cam_quaternion.at<double>(3,0);
    transform_stamped.transform.translation.x=scd.t_right_cam_to_left_cam.at<double>(0,0);
    transform_stamped.transform.translation.y=scd.t_right_cam_to_left_cam.at<double>(1,0);
    transform_stamped.transform.translation.z=scd.t_right_cam_to_left_cam.at<double>(2,0);

    right_to_left_pub.publish(transform_stamped);
    double total_elapsed = (ros::WallTime::now() - startTime).toSec();

    ROS_INFO(" TOTAL TIME STEREO CALIBRATION:  %f sec", total_elapsed);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "stereo_calibration");

    ros::NodeHandle nh;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle("~");

    // Declare variables that can be modified by launch file or command line.
    int rate;

    private_node_handle.param("rate", rate, 100);

    StereoCalibrationRos stereo_ros(nh,private_node_handle);

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
