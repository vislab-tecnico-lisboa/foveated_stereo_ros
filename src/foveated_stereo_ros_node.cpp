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
#include <logpolar.hpp>
#include <stereo_calib_lib.h>


#include <image_transport/image_transport.h>

using namespace sensor_msgs;

using namespace message_filters;

class FoveatedStereoNode
{
    tf::TransformListener listener;
    tf::StampedTransform l_eye_transform;
    tf::StampedTransform r_eye_transform;

    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;

public:
    typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

    ros::NodeHandle nh;
    ros::Publisher point_cloud_publisher;

    boost::shared_ptr<message_filters::Subscriber<Image> > left_image_sub;
    boost::shared_ptr<message_filters::Subscriber<Image> > right_image_sub;

    boost::shared_ptr<Synchronizer<MySyncPolicy> >sync;

    boost::shared_ptr<stereo_calib> stereo_calibration;
    boost::shared_ptr<LogPolar> foveated_stereo;

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
                       bool high_pass_) : nh(nh_), it_(nh_)
    {

        left_image_sub=boost::shared_ptr<message_filters::Subscriber<Image> > (new message_filters::Subscriber<Image>(nh, "left_image", 10));
        right_image_sub=boost::shared_ptr<message_filters::Subscriber<Image> > (new message_filters::Subscriber<Image>(nh, "right_image", 10));

        image_pub_ = it_.advertise("/vizzy/disparity", 3);

        sync=boost::shared_ptr<Synchronizer<MySyncPolicy> > (new Synchronizer<MySyncPolicy>(MySyncPolicy(10), *left_image_sub, *right_image_sub));
        sync->registerCallback(boost::bind(&FoveatedStereoNode::callback, this, _1, _2));
        stereo_calibration=boost::shared_ptr<stereo_calib> (new stereo_calib(fillStereoCalibParams()));

        sensor_msgs::CameraInfoConstPtr left_camera_info=ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/vizzy/l_camera/camera_info", ros::Duration(30));

        foveated_stereo=boost::shared_ptr<LogPolar> (new LogPolar((int)left_camera_info->width,
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
                                                                  high_pass_)
                                                     );

        point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("stereo", 10);
        return;
    }



    stereo_calib_params fillStereoCalibParams()
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

        tf::StampedTransform transform;

        try
        {
            listener.waitForTransform("/l_eye_vergence_link", "/r_eye_vergence_link", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform("/l_eye_vergence_link", "/r_eye_vergence_link",
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            exit(-1);
        }
        tf::Vector3 origin=transform.getOrigin();

        params.baseline=origin.length(); // meters
        ROS_INFO_STREAM("baseline:"<<params.baseline);
        params.encoders_measurements_noise=0.0000000175;
        params.encoders_state_noise=1.0;
        params.encoders_transition_noise=0.005;
        params.features_measurements_noise=25;
        params.matching_threshold=0.35;
        params.max_number_of_features=250;
        params.min_number_of_features=2;
        params.number_fixed_state_params=4;
        params.number_measurements=6;

        return params;
    }

    void callback(const ImageConstPtr& left_image,
                  const ImageConstPtr& right_image)
    {
        // Solve all of perception here...
        cv::Mat left_image_mat =cv_bridge::toCvCopy(left_image, left_image->encoding)->image;
        cv::Mat right_image_mat =cv_bridge::toCvCopy(right_image, right_image->encoding)->image;

        // Get angle
        try
        {
            listener.waitForTransform("/eyes_center_link", "/l_eye_vergence_link", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform("/eyes_center_link", "/l_eye_vergence_link",
                                     ros::Time(0), l_eye_transform);
            listener.waitForTransform("/eyes_center_link", "/r_eye_vergence_link", ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform("/eyes_center_link", "/r_eye_vergence_link",
                                     ros::Time(0), r_eye_transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            exit(-1);
        }

        double roll, pitch, yaw;
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
        ROS_INFO_STREAM("l_eye_angle:"<<l_eye_angle <<"   r_eye_angle:"<<r_eye_angle);

        // Calibrate
        stereo_calibration->calibrate(left_image_mat,
                                      right_image_mat,
                                      l_eye_angle,
                                      r_eye_angle);

        stereo_calib_data scd=stereo_calibration->get_calibrated_transformations(l_eye_angle,r_eye_angle);

        cv::Mat cortical_disparity_map=foveated_stereo->getDisparityMap(left_image_mat,
                                                       right_image_mat,
                                                       scd.R_left_cam_to_right_cam,
                                                       scd.t_left_cam_to_right_cam,
                                                       stereo_calibration->csc.LeftCalibMat,
                                                       stereo_calibration->csc.RightCalibMat
                                                       );

        cv::Mat cartesian_disparity_map=foveated_stereo->to_cortical(cortical_disparity_map);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", cartesian_disparity_map).toImageMsg();
        image_pub_.publish(msg);

        //publishPointCloud2StupidROSConvention(sdd);

    }

    /*void publishPointCloud2StupidROSConvention(stereo_disparity_data & sdd)
    {
        pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
        for(int r=0; r<sdd.point_cloud_xyz.rows; ++r)
        {
            for (int c=0; c<sdd.point_cloud_xyz.cols; ++c)
            {
                if(sdd.point_cloud_xyz.at<cv::Vec3d>(r,c)[2]>0.1 && sdd.point_cloud_xyz.at<cv::Vec3d>(r,c)[2]<10.0)
                {
                    pcl::PointXYZRGB point;

                    point.data[0] = sdd.point_cloud_xyz.at<cv::Vec3d>(r,c)[2];
                    point.data[1] = -sdd.point_cloud_xyz.at<cv::Vec3d>(r,c)[0];
                    point.data[2] = -sdd.point_cloud_xyz.at<cv::Vec3d>(r,c)[1];
                    point.r=sdd.point_cloud_rgb.at<cv::Vec3b>(r,c)[2];
                    point.g=sdd.point_cloud_rgb.at<cv::Vec3b>(r,c)[1];
                    point.b=sdd.point_cloud_rgb.at<cv::Vec3b>(r,c)[0];
                    point_cloud.push_back(point);
                }
            }
        }

        sensor_msgs::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(point_cloud,point_cloud_msg);

        point_cloud_msg.header.frame_id="eyes_center_link";
        point_cloud_publisher.publish(point_cloud_msg);

    }*/


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

    boost::shared_ptr<FoveatedStereoNode> foveated_stereo= boost::shared_ptr<FoveatedStereoNode>(new FoveatedStereoNode(nh,
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
                                                                                                                        high_pass
                                                                                                                        ));

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


