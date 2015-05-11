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

#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <Stereo.h>
#include <stereo_calib_lib.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf/message_filter.h>
#include <pcl_ros/transforms.h>
#include "structures.h"
#include "EgoSphere.h"

using namespace sensor_msgs;
using namespace message_filters;

class EgoSphereManagerRos
{
    std::string world_frame_id;

    tf::TransformListener listener;
    tf::StampedTransform l_eye_transform;

    image_transport::Publisher image_pub_;

    message_filters::Subscriber<sensor_msgs::PointCloud2>* point_cloud_subscriber_;
    tf::MessageFilter<sensor_msgs::PointCloud2>* tf_filter_;

public:
    typedef pcl::PointCloud<pcl::PointXYZRGB> PCLPointCloud;
    //typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

    ros::NodeHandle nh;

    ros::Publisher point_cloud_publisher;

    boost::shared_ptr<SphericalShell<std::vector<boost::shared_ptr<MemoryPatch> > > > ego_sphere;

    EgoSphereManagerRos(ros::NodeHandle & nh_, const unsigned int & egosphere_nodes, const unsigned int & angle_bins, std::string & world_frame_id_);

    ~EgoSphereManagerRos();

    void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

    void publishAll(const ros::Time& rostime);

    void insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& point_cloud);

    ros::Time last;
};


#endif // EGOSPHEREMANAGERROS_H
