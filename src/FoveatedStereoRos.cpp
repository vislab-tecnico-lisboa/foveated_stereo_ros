#include "FoveatedStereoRos.h"
using namespace sensor_msgs;

using namespace message_filters;

FoveatedStereoNode::~FoveatedStereoNode()
{}

FoveatedStereoNode::FoveatedStereoNode(ros::NodeHandle & nh_, ros::NodeHandle & private_node_handle_) :
    nh(nh_),
    private_node_handle(private_node_handle_),
    it_(nh_)
{

    left_camera_info_sub=nh_.subscribe("/vizzy/l_camera/camera_info",1,&FoveatedStereoNode::cameraInfoCallback, this);

    return;
}

void FoveatedStereoNode::cameraInfoCallback(const sensor_msgs::CameraInfoPtr & left_camera_info)
{
    left_camera_info_sub.shutdown();

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

    private_node_handle.param("sectors", sectors, 100);
    private_node_handle.param("rings", rings, 100);
    private_node_handle.param("min_radius", min_radius, 1.0);
    private_node_handle.param("interp", interp, 1);
    private_node_handle.param("sp", sp, 0);
    private_node_handle.param("full", full, 1);
    private_node_handle.param<std::string>("ego_frame", ego_frame, "ego_frame");
    private_node_handle.param<std::string>("left_camera_frame", left_camera_frame, "left_camera_frame");
    private_node_handle.param<std::string>("right_camera_frame", right_camera_frame, "right_camera_frame");
    private_node_handle.param("uncertainty_lower_bound", uncertainty_lower_bound, 0.0);
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
    ROS_INFO_STREAM("uncertainty_lower_bound: "<<uncertainty_lower_bound);
    ROS_INFO_STREAM("L: "<<L);
    ROS_INFO_STREAM("alpha: "<<alpha);
    ROS_INFO_STREAM("beta: "<<beta);
    ROS_INFO_STREAM("ki: "<<ki);
    ROS_INFO_STREAM("scaling_factor: "<<scaling_factor);

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

    left_image_sub=boost::shared_ptr<message_filters::Subscriber<Image> > (new message_filters::Subscriber<Image>(nh, "left_image", 10));
    right_image_sub=boost::shared_ptr<message_filters::Subscriber<Image> > (new message_filters::Subscriber<Image>(nh, "right_image", 10));

    sync=boost::shared_ptr<Synchronizer<MySyncPolicy> > (new Synchronizer<MySyncPolicy>(MySyncPolicy(10), *left_image_sub, *right_image_sub));
    sync->registerCallback(boost::bind(&FoveatedStereoNode::callback, this, _1, _2));

    ROS_INFO("Getting cameras' parameters");
    sensor_msgs::CameraInfoConstPtr right_camera_info=ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/vizzy/r_camera/camera_info", ros::Duration(30));

    //set the camera intrinsic parameters
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
    int width=(int)left_camera_info->width;
    int height=(int)left_camera_info->height;

    //stereo_calibration=boost::shared_ptr<stereo_calib> (new stereo_calib(fillStereoCalibParams(baseline)));


    std::cout <<"width:" << width << std::endl;
    std::cout << "height:"<< height << std::endl;
    ego_sphere=boost::shared_ptr<FovealStereo> (new FovealStereo(left_cam_intrinsic,
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
                                                     uncertainty_lower_bound,
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
    image_pub_ = it_.advertise("/vizzy/disparity", 3);

    point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("stereo", 10);
    mean_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("mean_pcl", 10);
    point_cloud_uncertainty_publisher = nh.advertise<sensor_msgs::PointCloud2>("uncertainty_pcl", 10);

    /*for(int i=0; i<9; ++i)
    {
        std::stringstream ss;
        ss << i;
        std::string str = ss.str();
        //sigma_point_clouds_publishers.push_back(nh.advertise<sensor_msgs::PointCloud2>("sigma_point_clouds_"+str, 10));
    }*/
    stereo_data_publisher = nh.advertise<foveated_stereo_ros::Stereo>("stereo_data", 1);

    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("covariances", 1);


    ROS_INFO_STREAM("done");
}

/*stereo_calib_params FoveatedStereoNode::fillStereoCalibParams(float & baseline)
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

void FoveatedStereoNode::callback(const ImageConstPtr& left_image,
                                  const ImageConstPtr& right_image)
{
    ros::WallTime startTime = ros::WallTime::now();

    // 1. Get uncalibrated color images
    cv::Mat left_image_mat =cv_bridge::toCvCopy(left_image, "bgr8")->image;
    cv::Mat right_image_mat =cv_bridge::toCvCopy(right_image, "bgr8")->image;

    // 2. Get eye angles with respect to eyes center
    try
    {
        listener.waitForTransform(ego_frame, left_camera_frame, ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform(ego_frame, left_camera_frame,
                                 ros::Time(0), l_eye_transform);
        /*listener.waitForTransform(ego_frame, "/r_camera_vision_link", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform(ego_frame, "/r_camera_vision_link",
                                     ros::Time(0), r_eye_transform);*/
        listener.waitForTransform(right_camera_frame, left_camera_frame, ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform(right_camera_frame, left_camera_frame,
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
    Eigen::Affine3d left_to_center_eigen;

    tf::transformTFToEigen (l_eye_transform, left_to_center_eigen );
    cv::Mat left_to_center = Mat::eye(4,4,CV_64F);
    cv::eigen2cv(left_to_center_eigen.matrix(),left_to_center);

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
                                                     left_to_center
                                                     );//*/

    double total_elapsed = (ros::WallTime::now() - startTime).toSec();

    ROS_INFO(" TOTAL TIME STEREO:  %f sec", total_elapsed);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", stereo_data.disparity_image).toImageMsg();
    image_pub_.publish(msg);
    publishStereoData(stereo_data, left_image->header.stamp);
    publishPointClouds(stereo_data, left_image->header.stamp);
    publishCovarianceMatrices(stereo_data, left_image->header.stamp);

}

void FoveatedStereoNode::publishPointClouds(StereoData & sdd, const ros::Time & time)
{
    sensor_msgs::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(sdd.point_cloud, point_cloud_msg);
    point_cloud_msg.is_dense=false;
    point_cloud_msg.header.stamp=time;
    point_cloud_publisher.publish(point_cloud_msg);

    pcl::toROSMsg(sdd.mean_point_cloud, point_cloud_msg);
    point_cloud_msg.is_dense=false;
    point_cloud_msg.header.stamp=time;
    mean_point_cloud_publisher.publish(point_cloud_msg);

    pcl::toROSMsg(sdd.point_cloud_uncertainty, point_cloud_msg);
    point_cloud_msg.is_dense=false;
    point_cloud_msg.header.stamp=time;
    point_cloud_uncertainty_publisher.publish(point_cloud_msg);

    /*for(int i=0; i<sdd.sigma_point_clouds.size();++i)
    {
        sensor_msgs::PointCloud2 point_cloud_msg;

        pcl::toROSMsg(sdd.sigma_point_clouds[i],point_cloud_msg);
        point_cloud_msg.is_dense=false;
        point_cloud_msg.header.stamp=time;

        sigma_point_clouds_publishers[i].publish(point_cloud_msg);
    }*/
}

void FoveatedStereoNode::publishCovarianceMatrices(StereoData & sdd, const ros::Time & time)
{

    visualization_msgs::MarkerArray marker_array;
    double scale=1.0;
    int jump=5;

    cv::Mat information_norms=sdd.getInformationsNorms();

    for(int r=0; r<sdd.cov_3d.size();r=r+jump)
    {
        for(int c=ignore_border_left; c<sdd.cov_3d[r].size();c=c+jump)
        {
            if(log(information_norms.at<double>(r,c))<uncertainty_lower_bound || isnan(log(information_norms.at<double>(r,c))) || isnan(sdd.mean_3d.at<cv::Vec3d>(r,c)[0]))
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

void FoveatedStereoNode::publishStereoData(StereoData & sdd, const ros::Time & time)
{
    foveated_stereo_ros::Stereo stereo_msg;
    sensor_msgs::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(sdd.point_cloud, point_cloud_msg);
    point_cloud_msg.is_dense=false;
    point_cloud_msg.header.stamp=time;
    stereo_msg.point_cloud=point_cloud_msg;
    stereo_msg.header=point_cloud_msg.header;
    for(unsigned int r=0; r<sdd.cov_3d.size(); ++r)
    {
        for(unsigned int c=0; c<sdd.cov_3d[r].size();++c)
        {
            if(isnan(sdd.mean_3d.at<cv::Vec3d>(r,c)[0])) // Outliers
                continue;

            Eigen::Matrix<double,3,3> information_eigen;
            cv2eigen(sdd.information_3d[r][c],information_eigen);

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(information_eigen.transpose()*information_eigen); // last one is the greatest eigen value

            //double norm_=(information_matrix.norm()); // L2 norm
            double norm_=sqrt(eig.eigenvalues()(2)); // SPECTRAL NORM

            if(log(norm_)<uncertainty_lower_bound || norm_!=norm_)
            {
                continue;
            }

            foveated_stereo_ros::Covariance covariance_msg;
            foveated_stereo_ros::Information information_msg;

            for(int i=0; i<3; ++i)
            {
                for(int j=0; j<3; ++j)
                {
                    int index=j+i*3;
                    covariance_msg.covariance[index]=sdd.cov_3d[r][c].at<double>(i,j);
                    information_msg.information[index]=sdd.information_3d[r][c].at<double>(i,j);
                }
            }

            stereo_msg.covariances.push_back(covariance_msg);
            stereo_msg.informations.push_back(information_msg);
        }
    }

    stereo_data_publisher.publish(stereo_msg);
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

    FoveatedStereoNode ego_sphere(nh,
                                  private_node_handle_);

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


