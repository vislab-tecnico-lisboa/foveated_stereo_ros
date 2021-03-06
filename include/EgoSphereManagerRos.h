#ifndef EGOSPHEREMANAGERROS_H
#define EGOSPHEREMANAGERROS_H

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
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <Stereo.h>
//#include <spherical_multiple_filter_stereo_calib_lib.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf/message_filter.h>


#include <visualization_msgs/MarkerArray.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <vizzy_msgs/GazeAction.h>
#include <foveated_stereo_ros/EgoData.h>
#include "DecisionMaking.h"
#include "structures.h"
#include "EgoSphere.h"
#include "foveated_stereo_ros/StereoData.h"
#include <rosbag/bag.h>
#include "ExpectedImprovement.h"
#include "ProbabilityOfImprovement.h"
using namespace sensor_msgs;
using namespace message_filters;

class EgoSphereManagerRos
{
    rosbag::Bag bag;
    boost::shared_ptr<AcquisitionFunction> acquisition_function;
private:
    bool update_mode;
    double field_of_view;
    double sensory_filtering_sphere_radius;

    Eigen::Matrix4f sensorToWorld;
    Eigen::Matrix4f sensorToEgo;
    Eigen::Matrix4f worldToEgo;
    Eigen::Matrix4f sensorToBase;
    Eigen::Matrix4f egoToBase;
    Eigen::Matrix4f egoToWorld;
    Eigen::Matrix4f egoTransform;
    Eigen::Matrix4f previousEgoToWorld;
    Eigen::Vector4d current_fixation_point;
    Eigen::Vector4d next_fixation_point;
    int current_fixation_point_index;
    int next_fixation_point_index;
    std::string world_frame_id;
    std::string ego_frame_id;
    std::string eyes_center_frame_id;
    std::string base_frame_id;
    boost::shared_ptr<tf::TransformListener> listener;
    tf::StampedTransform l_eye_transform;

    image_transport::Publisher image_pub_;

    message_filters::Subscriber<foveated_stereo_ros::StereoData>* stereo_data_subscriber_;
    tf::MessageFilter<foveated_stereo_ros::StereoData>* tf_filter_;

    ros::Timer update_timer;

    actionlib::SimpleActionClient<vizzy_msgs::GazeAction> ac;
    bool active_vision;
    ros::Time last_update_time;

    cv::Mat perturb_standard_deviation_mat;

    double fixation_point_error_tolerance;
    double gaze_timeout;
    bool relative_update;
    bool resample;
public:
    typedef pcl::PointCloud<pcl::PointXYZRGB> PCLPointCloud;

    ros::NodeHandle nh;

    ros::Publisher rgb_point_cloud_publisher;
    ros::Publisher uncertainty_point_cloud_publisher;
    ros::Publisher point_clouds_publisher;
    ros::Publisher marker_pub;
    ros::Publisher sensor_direction_pub;
    ros::Publisher information_publisher;
    ros::Publisher ses_structure_pub;

    boost::shared_ptr<SphericalShell<std::vector<boost::shared_ptr<MemoryPatch> > > > ego_sphere;
    boost::shared_ptr<DecisionMaking> decision_making;
    EgoSphereManagerRos(ros::NodeHandle & nh_,ros::NodeHandle & private_node_handle_);

    ~EgoSphereManagerRos();
    void updateEgoSphereRelative(const ros::TimerEvent&);
    void updateEgoSphereAbsolute(const ros::TimerEvent&);

    void insertCloudCallback(const foveated_stereo_ros::StereoData::ConstPtr& stereo_data);

    void publishAll(const foveated_stereo_ros::StereoDataConstPtr& stereo_data);
    void publishCovarianceMatrices();
    void insertScan(const PCLPointCloud& point_cloud, const std::vector<Eigen::Matrix3d> & covariances);
    Eigen::Vector3d perturb(const Eigen::Vector4d & fixation_point, const cv::Mat & scale);

    ros::Time last;

    void publishEgoStructure();
};


#endif // EGOSPHEREMANAGERROS_H
