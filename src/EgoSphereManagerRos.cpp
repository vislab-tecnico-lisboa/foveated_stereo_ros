#include "EgoSphereManagerRos.h"

EgoSphereManagerRos::EgoSphereManagerRos(ros::NodeHandle & nh_, const unsigned int & egosphere_nodes, const unsigned int & angle_bins, std::string & world_frame_id_) :
    nh(nh_),
    world_frame_id(world_frame_id_)//, listener(nh_, ros::Duration(50.))
{
    tf::StampedTransform sensorToWorldTf;

    while(1)
    {
        try
        {
            listener.lookupTransform("eyes_center_vision_link",world_frame_id, ros::Time::now(), sensorToWorldTf);
        } catch(tf::TransformException& ex){
            //ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
            continue;
        }
        break;
    }

    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

    ego_sphere=boost::shared_ptr<SphericalShell<std::vector< boost::shared_ptr<MemoryPatch> > > > (new SphericalShell<std::vector< boost::shared_ptr<MemoryPatch> > > (egosphere_nodes, angle_bins, sensorToWorld));
    point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("ego_sphere", 10);

    point_cloud_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh, "stereo", 5);
    tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (*point_cloud_subscriber_, listener, world_frame_id, 5);
    tf_filter_->registerCallback(boost::bind(&EgoSphereManagerRos::insertCloudCallback, this, _1));
    last=ros::Time::now();
    return;
}

EgoSphereManagerRos::~EgoSphereManagerRos()
{
    if (tf_filter_){
        delete tf_filter_;
        tf_filter_ = NULL;
    }

    if (point_cloud_subscriber_){
        delete point_cloud_subscriber_;
        point_cloud_subscriber_ = NULL;
    }
}

void EgoSphereManagerRos::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud)
{
    //ROS_INFO_STREAM("Ego Sphere new data in.");

    ros::WallTime startTime = ros::WallTime::now();

    // 1. get transform from world to ego-sphere frame
    tf::StampedTransform sensorToWorldTf;
    try
    {
        listener.lookupTransform(cloud->header.frame_id, world_frame_id, cloud->header.stamp, sensorToWorldTf);
    } catch(tf::TransformException& ex){
        ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }

    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

    ros::WallTime transform_time = ros::WallTime::now();

    ROS_INFO_STREAM(" 1. transform time: " <<  (transform_time - startTime).toSec());




    // 1. Update or insert

    if(ego_sphere->update(sensorToWorld))
    {
        ///////////////////////
        // Update ego-sphere //
        ///////////////////////
        ros::WallTime update_time = ros::WallTime::now();
        ROS_INFO_STREAM(" 2. update time: " <<  (update_time - transform_time).toSec());

        return;
    }
    else
    {
        /////////////////////////////////////////
        // Insert new data into the ego-sphere //
        /////////////////////////////////////////

        PCLPointCloud pc; // input cloud for filtering and ground-detection
        pcl::fromROSMsg(*cloud, pc);

        // set up filter for height range, also removes NANs:
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());

        // directly transform to map frame:
        //pcl::transformPointCloud(pc, pc, sensorToWorld);

        // just filter height range:
        pass.setInputCloud(pc.makeShared());
        pass.filter(pc);

        ros::WallTime update_time = ros::WallTime::now();

        ros::WallTime filtering_time = ros::WallTime::now();

        ROS_INFO_STREAM(" 2. filtering time: " <<  (filtering_time - transform_time).toSec());


        insertScan(sensorToWorldTf.getOrigin(), pc);

        ros::WallTime insert_time = ros::WallTime::now();
        ROS_INFO_STREAM(" 2. insertion time: " <<  (insert_time - filtering_time).toSec());

        ROS_INFO_STREAM("   total points inserted: " <<   pc.size());

    }
    publishAll(cloud->header.stamp);
    double total_elapsed = (ros::WallTime::now() - startTime).toSec();

    ROS_INFO(" TOTAL TIME:  %f sec", total_elapsed);

}

void EgoSphereManagerRos::insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& point_cloud)
{
    //ego_sphere->insert(point_cloud);
    ego_sphere->insertHashTable(point_cloud);

}

void EgoSphereManagerRos::publishAll(const ros::Time& rostime)
{
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud=ego_sphere->getPointCloud();
    point_cloud.header.frame_id="eyes_center_vision_link";

    sensor_msgs::PointCloud2 point_cloud_msg;

    pcl::toROSMsg(point_cloud,point_cloud_msg);

    point_cloud_msg.is_dense=false;
    point_cloud_msg.header.stamp=rostime;

    point_cloud_publisher.publish(point_cloud_msg);
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
    int egosphere_nodes;
    int spherical_angle_bins;

    private_node_handle_.param("rate", rate, 100);
    private_node_handle_.param("egosphere_nodes", egosphere_nodes, 1000);
    private_node_handle_.param("spherical_angle_bins", spherical_angle_bins, 1000);

    ROS_INFO_STREAM("rate: "<<rate);
    ROS_INFO_STREAM("egosphere_nodes: "<<egosphere_nodes);
    ROS_INFO_STREAM("spherical_angle_bins: "<<spherical_angle_bins);

    std::string world_frame_id="base_link";

    EgoSphereManagerRos ego_sphere(nh,
                                   egosphere_nodes,
                                   spherical_angle_bins,
                                   world_frame_id
                                   );

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




