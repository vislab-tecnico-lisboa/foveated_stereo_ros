#include "ConventionalStereoRos.h"
using namespace sensor_msgs;

using namespace message_filters;

ConventionalStereoRos::~ConventionalStereoRos()
{}

ConventionalStereoRos::ConventionalStereoRos(ros::NodeHandle & nh_, ros::NodeHandle & private_node_handle_) : StereoRos(nh_,private_node_handle_)
{
    left_camera_info_sub=nh_.subscribe("/vizzy/l_camera/camera_info",1,&ConventionalStereoRos::cameraInfoCallback, this);
    return;
}

void ConventionalStereoRos::cameraInfoCallback(const sensor_msgs::CameraInfoPtr & left_camera_info)
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
    image_pub_ = it_.advertise("/vizzy/disparity", 3);

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

    sync=boost::shared_ptr<Synchronizer<MySyncPolicy> > (new Synchronizer<MySyncPolicy>(MySyncPolicy(10), *left_image_sub, *right_image_sub));
    sync->registerCallback(boost::bind(&ConventionalStereoRos::callback, this, _1, _2));
    ROS_INFO_STREAM("done");
}

void ConventionalStereoRos::callback(const ImageConstPtr& left_image,
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

    StereoData stereo_data=stereo->computeStereo(left_image_mat,
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


