#include "StereoRos.h"
using namespace sensor_msgs;

using namespace message_filters;

StereoRos::~StereoRos()
{}

StereoRos::StereoRos(ros::NodeHandle & nh_, ros::NodeHandle & private_node_handle_) :
    nh(nh_),
    private_node_handle(private_node_handle_),
    it_(nh_),
    listener(new tf::TransformListener(ros::Duration(2.0)))
{
    return;
}

/*stereo_calib_params StereoRos::fillStereoCalibParams(float & baseline)
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
}*/

void StereoRos::publishStereoData(StereoData & sdd, const ros::Time & time)
{
    sensor_msgs::PointCloud2 rgb_point_cloud_msg;
    pcl::toROSMsg(sdd.point_cloud, rgb_point_cloud_msg);
    rgb_point_cloud_msg.is_dense=false;
    rgb_point_cloud_msg.header.stamp=time;
    rgb_point_cloud_publisher.publish(rgb_point_cloud_msg);

    sensor_msgs::PointCloud2 mean_point_cloud_msg;
    pcl::toROSMsg(sdd.mean_point_cloud, mean_point_cloud_msg);
    mean_point_cloud_msg.is_dense=false;
    mean_point_cloud_msg.header.stamp=time;
    mean_point_cloud_publisher.publish(mean_point_cloud_msg);

    sensor_msgs::PointCloud2 uncertainty_point_cloud_viz_msg;
    pcl::toROSMsg(sdd.point_cloud_uncertainty_viz, uncertainty_point_cloud_viz_msg);
    uncertainty_point_cloud_viz_msg.is_dense=false;
    uncertainty_point_cloud_viz_msg.header.stamp=time;
    uncertainty_point_cloud_publisher.publish(uncertainty_point_cloud_viz_msg);

    sensor_msgs::PointCloud2 uncertainty_point_cloud_msg;
    pcl::toROSMsg(sdd.point_cloud_uncertainty, uncertainty_point_cloud_msg);
    uncertainty_point_cloud_msg.is_dense=false;
    uncertainty_point_cloud_msg.header.stamp=time;

    foveated_stereo_ros::StereoData stereo_msg;
    stereo_msg.point_clouds.rgb_point_cloud=rgb_point_cloud_msg;
    stereo_msg.header=rgb_point_cloud_msg.header;
    stereo_msg.point_clouds.uncertainty_point_cloud=uncertainty_point_cloud_msg;
    cv::Mat informations_determinants=sdd.getInformationsDeterminants();

    for(unsigned int r=0; r<sdd.information_3d.size(); ++r)
    {
        for(unsigned int c=0; c<sdd.information_3d[r].size();++c)
        {
            if(isnan(informations_determinants.at<double>(r,c))||isnan(log(informations_determinants.at<double>(r,c)))||log(informations_determinants.at<double>(r,c))<information_lower_bound)
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



    /*for(int i=0; i<sdd.sigma_point_clouds.size();++i)
    {
        sensor_msgs::PointCloud2 point_cloud_msg;

        pcl::toROSMsg(sdd.sigma_point_clouds[i],point_cloud_msg);
        point_cloud_msg.is_dense=false;
        point_cloud_msg.header.stamp=time;

        sigma_point_clouds_publishers[i].publish(point_cloud_msg);
    }*/


    stereo_data_publisher.publish(stereo_msg);
}
void StereoRos::publishCovarianceMatrices(StereoData & sdd, const ros::Time & time)
{

    visualization_msgs::MarkerArray marker_array;
    double scale=1.0;
    int jump=5;

    cv::Mat informations_determinants=sdd.getInformationsDeterminants();

    for(int r=0; r<sdd.information_3d.size();r=r+jump)
    {
        for(int c=ignore_border_left; c<sdd.information_3d[r].size();c=c+jump)
        {
            if(isnan(informations_determinants.at<double>(r,c))||isnan(log(informations_determinants.at<double>(r,c)))||log(informations_determinants.at<double>(r,c))<information_lower_bound)
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
            marker.header.frame_id = ego_frame;
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



