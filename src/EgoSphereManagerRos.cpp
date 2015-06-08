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

    ego_sphere=boost::shared_ptr<SphericalShell<std::vector< boost::shared_ptr<MemoryPatch> > > > (new SphericalShell<std::vector< boost::shared_ptr<MemoryPatch> > > (egosphere_nodes, angle_bins, sensorToWorld.cast <double> ()));
    point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("ego_sphere", 10);
    point_cloud_uncertainty_publisher  = nh.advertise<sensor_msgs::PointCloud2>("ego_sphere_uncertainty", 10);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("covariances_debug", 1);


    stereo_data_subscriber_ = new message_filters::Subscriber<foveated_stereo_ros::Stereo> (nh, "stereo_data", 10);
    tf_filter_ = new tf::MessageFilter<foveated_stereo_ros::Stereo> (*stereo_data_subscriber_, listener, world_frame_id, 5);
    tf_filter_->registerCallback(boost::bind(&EgoSphereManagerRos::insertCloudCallback, this, _1));
    last=ros::Time::now();
    return;
}

EgoSphereManagerRos::~EgoSphereManagerRos()
{
    if (tf_filter_)
    {
        delete tf_filter_;
        tf_filter_ = NULL;
    }

    if (stereo_data_subscriber_)
    {
        delete stereo_data_subscriber_;
        stereo_data_subscriber_ = NULL;
    }
}

void EgoSphereManagerRos::insertCloudCallback(const foveated_stereo_ros::Stereo::ConstPtr& stereo_data)
{
    //ROS_INFO_STREAM("Ego Sphere new data in.");

    ros::WallTime startTime = ros::WallTime::now();

    // 1. get transform from world to ego-sphere frame
    tf::StampedTransform sensorToWorldTf;
    try
    {
        listener.lookupTransform(stereo_data->point_cloud.header.frame_id, world_frame_id, stereo_data->point_cloud.header.stamp, sensorToWorldTf);
    } catch(tf::TransformException& ex){
        ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }

    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

    ros::WallTime transform_time = ros::WallTime::now();

    ROS_INFO_STREAM(" 1. transform time: " <<  (transform_time - startTime).toSec());

    // 1. Update or insert
    if(ego_sphere->update(sensorToWorld.cast <double> ()))
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
        std::vector<Eigen::Matrix3d> covariances;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        covariances.reserve(stereo_data->covariances.size());
        for(int c=0; c<stereo_data->covariances.size();++c)
        {
            Eigen::Matrix3d covariance;
            for(int i=0; i<3; ++i)
            {
                for(int j=0; j<3; ++j)
                {
                    int index=j+i*3;
                    covariance(i,j)=stereo_data->covariances[c].covariance[index];
                }
            }
            if(covariance.norm()>0.0001&&covariance.norm()<3.0)
            {
                inliers->indices.push_back(c);
                covariances.push_back(covariance);
                continue;
            }
        }

        PCLPointCloud pc; // input cloud for filtering and ground-detection
        pcl::fromROSMsg(stereo_data->point_cloud, pc);
        // set up filter for height range, also removes NANs:
        //pcl::PassThrough<pcl::PointXYZRGB> pass;
        //pass.setFilterFieldName("z");
        //pass.setFilterLimits(-std::numeric_limits<double>::max(), std::numeric_limits<double>::max());

        // just filter height range:
        //pass.setInputCloud(pc.makeShared());
        //pass.setIndices(inliers);
        //pass.filter(pc);

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        // Extract the inliers
        extract.setInputCloud (pc.makeShared());
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (pc);
        ros::WallTime filtering_time = ros::WallTime::now();

        ROS_INFO_STREAM(" 2. filtering time: " <<  (filtering_time - transform_time).toSec());


        insertScan(pc,covariances);

        ros::WallTime insert_time = ros::WallTime::now();
        ROS_INFO_STREAM(" 2. insertion time: " <<  (insert_time - filtering_time).toSec());

        ROS_INFO_STREAM("   total points inserted: " <<   pc.size());

    }
    publishCovarianceMatrices(stereo_data->point_cloud.header.stamp);
    publishAll(stereo_data->point_cloud.header.stamp);
    double total_elapsed = (ros::WallTime::now() - startTime).toSec();

    ROS_INFO(" TOTAL TIME:  %f sec", total_elapsed);
}

void EgoSphereManagerRos::insertScan(const PCLPointCloud& point_cloud, const std::vector<Eigen::Matrix3d> & covariances)
{
    //ego_sphere->insert(point_cloud);
    ego_sphere->insertHashTable(point_cloud, covariances);

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

    pcl::PointCloud<pcl::PointXYZI> point_cloud_uncertainty=ego_sphere->getPointCloudUncertainty();
    point_cloud_uncertainty.header.frame_id="eyes_center_vision_link";

    sensor_msgs::PointCloud2 point_cloud_uncertainty_msg;

    pcl::toROSMsg(point_cloud_uncertainty,point_cloud_uncertainty_msg);

    point_cloud_uncertainty_msg.is_dense=false;
    point_cloud_uncertainty_msg.header.stamp=rostime;

    point_cloud_uncertainty_publisher.publish(point_cloud_uncertainty_msg);
}


void EgoSphereManagerRos::publishCovarianceMatrices(const ros::Time & time)
{

    visualization_msgs::MarkerArray marker_array;
    double scale=3.0;
    int index=0;
    for(std::vector<boost::shared_ptr<MemoryPatch> >::iterator structure_it = ego_sphere->structure.begin(); structure_it != ego_sphere->structure.end(); ++structure_it)
    {
        double norm_=(*structure_it)->sensory_data.position.inv_cov.norm();
        if(norm_<0.01)
        {
            continue;
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig((*structure_it)->sensory_data.position.cov);
        Eigen::Quaternion<double> q(-eig.eigenvectors());
        q.normalize();

        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "eyes_center_vision_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "covariances";
        marker.id = index++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = (*structure_it)->sensory_data.position.mean[0];
        marker.pose.position.y = (*structure_it)->sensory_data.position.mean[1];
        marker.pose.position.z = (*structure_it)->sensory_data.position.mean[2];
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
    marker_pub.publish(marker_array);
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




