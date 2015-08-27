#include "PerfectStereoCalibrationRos.h"

PerfectStereoCalibrationRos::PerfectStereoCalibrationRos()
{
}

PerfectStereoCalibrationRos::PerfectStereoCalibrationRos(ros::NodeHandle & nh_, ros::NodeHandle & private_node_handle_) :
    nh(nh_),
    private_node_handle(private_node_handle_),
    listener(new tf::TransformListener(ros::Duration(5.0)))

{
    private_node_handle.param<std::string>("ego_frame", ego_frame, "ego_frame");
    private_node_handle.param<std::string>("left_camera_frame", left_camera_frame, "left_camera_frame");
    private_node_handle.param<std::string>("right_camera_frame", right_camera_frame, "right_camera_frame");

}

void PerfectStereoCalibrationRos::streamTF()
{
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

}

