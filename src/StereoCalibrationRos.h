#ifndef STEREOCALIBRATIONROS_H
#define STEREOCALIBRATIONROS_H

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <complete_stereo_calib_lib.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

class StereoCalibrationRos
{
    ros::NodeHandle nh;
    ros::NodeHandle private_node_handle;
    boost::shared_ptr<tf::TransformListener> listener;
    boost::shared_ptr<complete_stereo_calib> stereo_calibration;
    image_transport::Publisher image_pub_;

public:
    StereoCalibrationRos();

    void callback(const sensor_msgs::ImageConstPtr& left_image,
                                         const sensor_msgs::ImageConstPtr& right_image)
    {
        ROS_INFO("Stereo calibration callback...");
        ros::WallTime startTime = ros::WallTime::now();

        // 1. Get uncalibrated color images
        cv::Mat left_image_mat =cv_bridge::toCvCopy(left_image, "bgr8")->image;
        cv::Mat right_image_mat =cv_bridge::toCvCopy(right_image, "bgr8")->image;

        // 2. Get eye angles with respect to eyes center
        /*try
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
        }*/


        cv::Mat stereo_encoders = cv::Mat::zeros(6,1,CV_64F);
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
        */
        // 3. calibrate given angles
        ROS_INFO("Calibrate stereo...");
        stereo_calibration->calibrate(left_image_mat,
                                      right_image_mat,
                                      stereo_encoders);

        ROS_INFO("Done.");

        //get the calibrated transformations between the two cameras
        complete_stereo_calib_data scd;
        scd =  stereo_calibration->get_calibrated_transformations(stereo_encoders);

        //obtain and show the disparity map
        complete_stereo_disparity_data csdd = stereo_calibration->complete_stereo_calib::get_disparity_map(left_image_mat, right_image_mat, stereo_encoders);



         //    imshow("disparity", csdd.disparity_image);
        //waitKey(1);
        /*Eigen::Affine3d left_to_center_eigen;

        tf::transformTFToEigen (l_eye_transform, left_to_center_eigen );
        cv::Mat left_to_center = Mat::eye(4,4,CV_64F);
        cv::eigen2cv(left_to_center_eigen.matrix(),left_to_center);

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

        //obtain and show the disparity map
        /*complete_stereo_disparity_data csdd = scd.complete_stereo_calib::get_disparity_map(left_rz, right_rz, stereo_encoders);
            imshow("disparity", csdd.disparity_image);
        waitKey(1)*/

        //show the transformation between the left and right images
        //cout << "Transformation from left to right camera: " << cscd.transformation_left_cam_to_right_cam << endl;



        double total_elapsed = (ros::WallTime::now() - startTime).toSec();

        ROS_INFO(" TOTAL TIME STEREO CALIBRATION:  %f sec", total_elapsed);
    }


};

#endif // STEREOCALIBRATIONROS_H
