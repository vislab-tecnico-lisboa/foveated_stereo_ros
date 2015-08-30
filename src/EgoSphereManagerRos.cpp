#include "EgoSphereManagerRos.h"

EgoSphereManagerRos::EgoSphereManagerRos(ros::NodeHandle & nh_, ros::NodeHandle & private_node_handle_) :
    nh(nh_),
    ac("gaze", true),
    fixation_point(Eigen::Vector4d::Constant(std::numeric_limits<double>::max()))

  //world_frame_id("map")
{
    fixation_point(3)=1; // Homogeneous coordinates

    // Declare variables that can be modified by launch file or command line.
    int egosphere_nodes;
    int spherical_angle_bins;
    double uncertainty_lower_bound;
    double mahalanobis_distance_threshold;
    double closest_point_bound;
    double sigma_scale_upper_bound;
    private_node_handle_.param<std::string>("world_frame",world_frame_id,"base_link");
    private_node_handle_.param<std::string>("ego_frame",ego_frame_id,"eyes_center_vision_link");

    private_node_handle_.param("egosphere_nodes", egosphere_nodes, 1000);
    private_node_handle_.param("spherical_angle_bins", spherical_angle_bins, 1000);
    private_node_handle_.param("uncertainty_lower_bound", uncertainty_lower_bound, 0.0);
    private_node_handle_.param("active_vision",active_vision,false);
    private_node_handle_.param("closest_point_bound",closest_point_bound,1.0);
    private_node_handle_.param("sigma_scale_upper_bound",sigma_scale_upper_bound,1.0);

    private_node_handle_.param("mahalanobis_distance_threshold",mahalanobis_distance_threshold,std::numeric_limits<double>::max());
    XmlRpc::XmlRpcValue mean_list;
    private_node_handle_.getParam("mean", mean_list);
    ROS_ASSERT(mean_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    cv::Mat mean_mat(3, 1, CV_64F);

    for (int32_t i = 0; i < mean_list.size(); ++i)
    {
        ROS_ASSERT(mean_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        mean_mat.at<double>(i,0)=static_cast<double>(mean_list[i]);
    }

    XmlRpc::XmlRpcValue std_dev_list;
    private_node_handle_.getParam("standard_deviation", std_dev_list);
    ROS_ASSERT(std_dev_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    cv::Mat standard_deviation_mat(3, 1, CV_64F);

    for (int32_t i = 0; i < std_dev_list.size(); ++i)
    {
        ROS_ASSERT(std_dev_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        standard_deviation_mat.at<double>(i,0)=static_cast<double>(std_dev_list[i]);
    }

    ROS_INFO_STREAM("egosphere_nodes: "<<egosphere_nodes);
    ROS_INFO_STREAM("spherical_angle_bins: "<<spherical_angle_bins);
    ROS_INFO_STREAM("uncertainty_lower_bound: "<<uncertainty_lower_bound);
    ROS_INFO_STREAM("closest_point_bound: "<<closest_point_bound);

    ROS_INFO_STREAM("mahalanobis_distance_threshold: "<<mahalanobis_distance_threshold);
    ROS_INFO_STREAM("mean: "<<mean_mat);
    ROS_INFO_STREAM("standard_deviation: "<<standard_deviation_mat);
    ROS_INFO_STREAM("active_vision: "<<active_vision);
    ROS_INFO_STREAM("world_frame_id: "<<world_frame_id);
    ROS_INFO_STREAM("ego_frame_id: "<<ego_frame_id);


    ego_sphere = boost::shared_ptr<SphericalShell<std::vector< boost::shared_ptr<MemoryPatch> > > > (new SphericalShell<std::vector< boost::shared_ptr<MemoryPatch> > > (egosphere_nodes, spherical_angle_bins, sensorToWorld.cast <double> (),uncertainty_lower_bound,mahalanobis_distance_threshold,mean_mat,standard_deviation_mat));
    decision_making = boost::shared_ptr<DecisionMaking> (new DecisionMaking(ego_sphere,closest_point_bound,sigma_scale_upper_bound));


    rgb_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("ego_sphere", 1);
    uncertainty_point_cloud_publisher  = nh.advertise<sensor_msgs::PointCloud2>("ego_sphere_uncertainty", 1);
    point_clouds_publisher = nh.advertise<foveated_stereo_ros::EgoData>("ego_point_clouds", 1);

    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("covariances_debug", 1);
    ego_sphere_hash_table  = nh.advertise<sensor_msgs::PointCloud2>("ego_sphere_structure", 2);

    stereo_data_subscriber_ = new message_filters::Subscriber<foveated_stereo_ros::StereoData> (nh, "stereo_data", 1);
    tf_filter_ = new tf::MessageFilter<foveated_stereo_ros::StereoData> (*stereo_data_subscriber_, listener, world_frame_id, 1);
    tf_filter_->registerCallback(boost::bind(&EgoSphereManagerRos::insertCloudCallback, this, _1));
    last=ros::Time::now();

    //GIVE TIME TO MATLAB
    //sleep(5.0);
    return;
}

void EgoSphereManagerRos::publishEgoStructure()
{
    sensor_msgs::PointCloud2 ego_sphere_msg;

    pcl::toROSMsg(*(ego_sphere->structure_cloud),ego_sphere_msg);

    ego_sphere_msg.is_dense=false;
    ego_sphere_msg.header.frame_id=ego_frame_id;
    ego_sphere_hash_table.publish(ego_sphere_msg);
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

void EgoSphereManagerRos::updateEgoSphere()
{
    ros::WallTime start_time = ros::WallTime::now();

    // 1. get transform from world to ego-sphere frame
    tf::StampedTransform worldToEgoTf;
    try
    {
        listener.waitForTransform(ego_frame_id, world_frame_id, ros::Time(0), ros::Duration(10.0) );

        listener.lookupTransform(ego_frame_id, world_frame_id, ros::Time(0), worldToEgoTf);
    } catch(tf::TransformException& ex){
        ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }

    pcl_ros::transformAsMatrix(worldToEgoTf, worldToEgo);

    ros::WallTime transform_time = ros::WallTime::now();

    ROS_INFO_STREAM(" 1. transform time: " <<  (transform_time - start_time).toSec());

    // 1. Update or insert
    if(ego_sphere->transform(worldToEgo.cast <double> ()))
    {
        ///////////////////////
        // Update ego-sphere //
        ///////////////////////
        ros::WallTime update_time = ros::WallTime::now();
        ROS_INFO_STREAM(" 2. Update time: " <<  (update_time - transform_time).toSec());

        return;
    }
}

void EgoSphereManagerRos::insertCloudCallback(const foveated_stereo_ros::StereoData::ConstPtr& stereo_data)
{
    ROS_INFO_STREAM("Ego Sphere new data in.");

    ros::WallTime start_time = ros::WallTime::now();

    /////////////////////////////////////////
    // Insert new data into the ego-sphere //
    /////////////////////////////////////////

    // Filter data
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    PCLPointCloud pc; // input cloud for filtering and ground-detection
    pcl::fromROSMsg(stereo_data->point_clouds.rgb_point_cloud, pc);
    pcl::removeNaNFromPointCloud(pc, pc, inliers->indices);

    std::vector<Eigen::Matrix3d> informations;
    informations.reserve(stereo_data->informations.size());

    for(int inlier_index=0; inlier_index<inliers->indices.size();++inlier_index)
    {
        int c=inliers->indices[inlier_index];
        Eigen::Matrix3d information;
        for(int i=0; i<3; ++i)
        {
            for(int j=0; j<3; ++j)
            {
                int index=j+i*3;
                information(i,j)=stereo_data->informations[c].information[index];
            }
        }
        informations.push_back(information);
    }

    ros::WallTime filtering_time = ros::WallTime::now();

    ROS_INFO_STREAM(" 1. filtering time: " <<  (filtering_time - start_time).toSec());


    tf::StampedTransform sensorToWorldTf;
    while(nh.ok())
    {
        try
        {
            listener.waitForTransform(world_frame_id, stereo_data->point_clouds.rgb_point_cloud.header.frame_id, stereo_data->point_clouds.rgb_point_cloud.header.stamp, ros::Duration(10.0) );
            listener.lookupTransform(world_frame_id, stereo_data->point_clouds.rgb_point_cloud.header.frame_id, stereo_data->point_clouds.rgb_point_cloud.header.stamp, sensorToWorldTf);
        }
        catch(tf::TransformException& ex)
        {
            //ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
            continue;
        }
        break;
    }


    tf::StampedTransform sensorToEgoTf;
    while(nh.ok())
    {
        try
        {
            listener.waitForTransform(ego_frame_id, stereo_data->point_clouds.rgb_point_cloud.header.frame_id, stereo_data->point_clouds.rgb_point_cloud.header.stamp, ros::Duration(10.0) );
            listener.lookupTransform(ego_frame_id, stereo_data->point_clouds.rgb_point_cloud.header.frame_id, stereo_data->point_clouds.rgb_point_cloud.header.stamp, sensorToEgoTf);
        } catch(tf::TransformException& ex)
        {
            //ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
            continue;
        }
        break;
    }

    ///////////////////////////////////////////////////////////////
    // TRANSFORM SENSOR POINTS TO WORLD FRAME FOR FILTERING ON Z //
    ///////////////////////////////////////////////////////////////
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
    pcl::transformPointCloud (pc, pc, sensorToWorld);

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (pc.makeShared());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 5.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (pc);

    //////////////////////////////////////////////////////////
    // TRANSFORM SENSOR POINTS TO EGO FRAME FOR DATA FUSION //
    //////////////////////////////////////////////////////////
    pcl_ros::transformAsMatrix(sensorToEgoTf, sensorToEgo);

    pcl::transformPointCloud (pc, pc, sensorToEgo*sensorToWorld.inverse());
    ros::WallTime transform_data_time = ros::WallTime::now();

    ////////////////////////////////////////////////////////
    // TRANSFORM SENSOR INFORMATION MATRICES TO EGO FRAME //
    ////////////////////////////////////////////////////////
    for(int i=0; i<informations.size();++i)
    {
        informations[i].noalias()=sensorToEgo.block(0,0,3,3).cast <double> ()*informations[i]*sensorToEgo.block(0,0,3,3).transpose().cast <double> ();
    }

    ROS_INFO_STREAM(" 2. transform data time: " <<  (transform_data_time - filtering_time).toSec());

    insertScan(pc,informations);

    ros::WallTime insert_time = ros::WallTime::now();
    ROS_INFO_STREAM(" 3. insertion time: " <<  (insert_time - transform_data_time).toSec());

    ROS_DEBUG_STREAM("   total points inserted: " <<   pc.size());

    /////////
    // ACT //
    /////////

    while(active_vision|| nh.ok())
    {
        //ego_sphere->getClosestPoint();
        Eigen::Vector3d fixation_point_3d=decision_making->getFixationPoint();

        fixation_point(0)=fixation_point_3d(0);
        fixation_point(1)=fixation_point_3d(1);
        fixation_point(2)=fixation_point_3d(2);
        // send a goal to the action
        move_robot_msgs::GazeGoal goal;
        goal.fixation_point.header.frame_id=ego_frame_id;
        goal.fixation_point.header.stamp=ros::Time::now();
        goal.fixation_point.point.x = fixation_point(0);
        goal.fixation_point.point.y = fixation_point(1);
        goal.fixation_point.point.z = fixation_point(2);

        ROS_DEBUG("Waiting for action server to start.");
        // wait for the action server to start
        ac.waitForServer(); //with no duration will wait for infinite time
        ROS_DEBUG("Started.");
        //wait for the action to return
        ac.sendGoal(goal);
        ROS_ERROR("WAIT FOR RESULT...");

        bool finished_before_timeout = ac.waitForResult(ros::Duration(10));
        ROS_ERROR("DONE.");

        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            if(state.SUCCEEDED)
            {
                ROS_INFO("Action finished: %s",state.toString().c_str());
                break;
            }
            else
            {
                ROS_WARN("Point not valid.");
            }
        }
        else
        {
            ROS_ERROR("Action did not finish before the time out.");
        }
    }

    //////////
    // SHOW //
    //////////

    ros::WallTime publish_time_before = ros::WallTime::now();


    publishAll(stereo_data);
    ros::WallTime publish_time_after = ros::WallTime::now();
    ROS_INFO_STREAM(" 4. publish time: " <<  (publish_time_after - publish_time_before).toSec());

    //publishCovarianceMatrices();
    double total_elapsed = (ros::WallTime::now() - start_time).toSec();

    ROS_INFO(" TOTAL TIME:  %f sec", total_elapsed);
    sleep(4.0); /// give time to sensor
}


void EgoSphereManagerRos::insertScan(const PCLPointCloud& point_cloud, const std::vector<Eigen::Matrix3d> & covariances)
{
    //ego_sphere->insert(point_cloud);
    ego_sphere->insertHashTable(point_cloud, covariances);

}

void EgoSphereManagerRos::publishAll(const foveated_stereo_ros::StereoDataConstPtr& stereo_data)
{
    pcl::PointCloud<pcl::PointXYZRGB> rgb_point_cloud=ego_sphere->getPointCloud();
    rgb_point_cloud.header.frame_id=ego_frame_id;
    rgb_point_cloud.is_dense=false;
    pcl_conversions::toPCL(ros::Time::now(), rgb_point_cloud.header.stamp);
    sensor_msgs::PointCloud2 rgb_point_cloud_msg;
    pcl::toROSMsg(rgb_point_cloud,rgb_point_cloud_msg);

    rgb_point_cloud_publisher.publish(rgb_point_cloud_msg);

    pcl::PointCloud<pcl::PointXYZI> point_cloud_uncertainty_viz=ego_sphere->getPointCloudUncertaintyViz();
    point_cloud_uncertainty_viz.header.frame_id=ego_frame_id;
    point_cloud_uncertainty_viz.is_dense=false;
    pcl_conversions::toPCL(ros::Time::now(), point_cloud_uncertainty_viz.header.stamp);
    sensor_msgs::PointCloud2 uncertainty_point_cloud_viz_msg;
    pcl::toROSMsg(point_cloud_uncertainty_viz, uncertainty_point_cloud_viz_msg);
    uncertainty_point_cloud_publisher.publish(uncertainty_point_cloud_viz_msg);


    foveated_stereo_ros::PointClouds ego_point_clouds_msg;

    sensor_msgs::PointCloud2 rgb_point_cloud_msg_base;
    pcl_ros::transformPointCloud(sensorToWorld, rgb_point_cloud_msg, rgb_point_cloud_msg_base);

    pcl::PointCloud<pcl::PointXYZI> point_cloud_uncertainty=ego_sphere->getPointCloudUncertainty();
    point_cloud_uncertainty.header.frame_id=ego_frame_id;
    point_cloud_uncertainty.is_dense=false;
    pcl_conversions::toPCL(ros::Time::now(), point_cloud_uncertainty.header.stamp);
    sensor_msgs::PointCloud2 uncertainty_point_cloud_msg;
    pcl::toROSMsg(point_cloud_uncertainty, uncertainty_point_cloud_msg);


    sensor_msgs::PointCloud2 uncertainty_point_cloud_msg_base;
    pcl_ros::transformPointCloud(sensorToWorld, uncertainty_point_cloud_msg, uncertainty_point_cloud_msg_base);


    ego_point_clouds_msg.rgb_point_cloud=rgb_point_cloud_msg_base;
    ego_point_clouds_msg.uncertainty_point_cloud=uncertainty_point_cloud_msg_base;

    foveated_stereo_ros::EgoData ego_data_msg;

    sensor_msgs::PointCloud2 sensor_rgb_point_cloud_msg_base;
    pcl_ros::transformPointCloud(sensorToWorld, stereo_data->point_clouds.rgb_point_cloud, sensor_rgb_point_cloud_msg_base);

    sensor_msgs::PointCloud2 sensor_uncertainty_point_cloud_msg_base;

    pcl_ros::transformPointCloud(sensorToWorld, stereo_data->point_clouds.uncertainty_point_cloud, sensor_uncertainty_point_cloud_msg_base);

    ego_data_msg.sensor_point_clouds.rgb_point_cloud=sensor_rgb_point_cloud_msg_base;
    ego_data_msg.sensor_point_clouds.uncertainty_point_cloud=sensor_uncertainty_point_cloud_msg_base;
    ego_data_msg.ego_point_clouds=ego_point_clouds_msg;

    Eigen::Vector4f fixation_point_world;
    fixation_point_world=(sensorToWorld*sensorToEgo.inverse())*fixation_point.cast<float>();
    geometry_msgs::PointStamped fixation_point_world_msg;
    fixation_point_world_msg.header.frame_id=world_frame_id;
    fixation_point_world_msg.point.x = fixation_point_world(0);
    fixation_point_world_msg.point.y = fixation_point_world(1);
    fixation_point_world_msg.point.z = fixation_point_world(2);

    ego_data_msg.fixation_point=fixation_point_world_msg;

    point_clouds_publisher.publish(ego_data_msg);
}

void EgoSphereManagerRos::publishCovarianceMatrices()
{
    std::vector<Eigen::Matrix<double, 3, 3> > covs=ego_sphere->getCovariances();
    pcl::PointCloud<pcl::PointXYZ> point_cloud=ego_sphere->getMeans();

    visualization_msgs::MarkerArray marker_array;
    double scale=0.01;
    int jump=1;

    for(int r=0; r<covs.size();r=r+jump)
    {

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covs[r]); // last one is the greatest eigen value

        Eigen::Quaternion<double> q(-eig.eigenvectors());
        q.normalize();

        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = ego_frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "covariances";
        marker.id = r;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = point_cloud.points[r].x;
        marker.pose.position.y = point_cloud.points[r].y;
        marker.pose.position.z = point_cloud.points[r].z;
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

        marker.lifetime = ros::Duration(5.0);
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
    int rate;

    private_node_handle_.param("rate", rate, 100);
    ROS_INFO_STREAM("rate: "<<rate);
    EgoSphereManagerRos ego_sphere(nh,
                                   private_node_handle_
                                   );

    // Tell ROS how fast to run this node.
    ros::Rate r(rate);

    // Main loop.
    while (nh.ok())
    {
        ego_sphere.updateEgoSphere();
        ego_sphere.publishEgoStructure();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
} // end




