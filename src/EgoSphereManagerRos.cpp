#include "EgoSphereManagerRos.h"

EgoSphereManagerRos::EgoSphereManagerRos(ros::NodeHandle & nh_, ros::NodeHandle & private_node_handle_) :
    nh(nh_),
    ac("gaze", false),
    fixation_point(Eigen::Vector3d::Constant(std::numeric_limits<double>::max()))

  //world_frame_id("map")
{
    // Declare variables that can be modified by launch file or command line.
    int egosphere_nodes;
    int spherical_angle_bins;
    double uncertainty_lower_bound;
    double mahalanobis_distance_threshold;
    private_node_handle_.param<std::string>("world_frame",world_frame_id,"base_link");
    private_node_handle_.param<std::string>("ego_frame",ego_frame_id,"eyes_center_vision_link");

    private_node_handle_.param("egosphere_nodes", egosphere_nodes, 1000);
    private_node_handle_.param("spherical_angle_bins", spherical_angle_bins, 1000);
    private_node_handle_.param("uncertainty_lower_bound", uncertainty_lower_bound, 0.0);
    private_node_handle_.param("active_vision",active_vision,false);

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
    ROS_INFO_STREAM("mahalanobis_distance_threshold: "<<mahalanobis_distance_threshold);
    ROS_INFO_STREAM("mean: "<<mean_mat);
    ROS_INFO_STREAM("standard_deviation: "<<standard_deviation_mat);
    ROS_INFO_STREAM("active_vision: "<<active_vision);
    ROS_INFO_STREAM("world_frame_id: "<<world_frame_id);
    ROS_INFO_STREAM("ego_frame_id: "<<ego_frame_id);

    tf::StampedTransform sensorToWorldTf;
    ROS_INFO_STREAM("Waiting for transform from world ("<<world_frame_id<<") to ego frame ("<<ego_frame_id<<")...");

    while(ros::ok())
    {
        try
        {
            listener.waitForTransform(ego_frame_id, world_frame_id, ros::Time(0), ros::Duration(10.0) );
            listener.lookupTransform(ego_frame_id, world_frame_id, ros::Time(0), sensorToWorldTf);
        } catch(tf::TransformException& ex){
            ROS_DEBUG_STREAM( "Transform lookup failed: " << ex.what());
            continue;
        }
        break;
    }
    ROS_INFO_STREAM("Done.");

    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

    ego_sphere=boost::shared_ptr<SphericalShell<std::vector< boost::shared_ptr<MemoryPatch> > > > (new SphericalShell<std::vector< boost::shared_ptr<MemoryPatch> > > (egosphere_nodes, spherical_angle_bins, sensorToWorld.cast <double> (),uncertainty_lower_bound,mahalanobis_distance_threshold,mean_mat,standard_deviation_mat));
    rgb_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("ego_sphere", 2);
    uncertainty_point_cloud_publisher  = nh.advertise<sensor_msgs::PointCloud2>("ego_sphere_uncertainty", 2);
    point_clouds_publisher = nh.advertise<foveated_stereo_ros::EgoData>("ego_point_clouds", 10);

    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("covariances_debug", 1);
    ego_sphere_hash_table  = nh.advertise<sensor_msgs::PointCloud2>("ego_sphere_structure", 2);

    stereo_data_subscriber_ = new message_filters::Subscriber<foveated_stereo_ros::StereoData> (nh, "stereo_data", 2);
    tf_filter_ = new tf::MessageFilter<foveated_stereo_ros::StereoData> (*stereo_data_subscriber_, listener, world_frame_id, 2);
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
    ego_sphere_msg.header.frame_id="eyes_center_vision_link";
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

void EgoSphereManagerRos::insertCloudCallback(const foveated_stereo_ros::StereoData::ConstPtr& stereo_data)
{
    ROS_INFO_STREAM("Ego Sphere new data in.");

    ros::WallTime startTime = ros::WallTime::now();

    // 1. get transform from world to ego-sphere frame
    tf::StampedTransform sensorToWorldTf;
    try
    {
        listener.waitForTransform(ego_frame_id, world_frame_id, stereo_data->point_clouds.rgb_point_cloud.header.stamp, ros::Duration(10.0) );
        listener.lookupTransform(ego_frame_id, world_frame_id, stereo_data->point_clouds.rgb_point_cloud.header.stamp, sensorToWorldTf);
    } catch(tf::TransformException& ex){
        ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }

    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

    ros::WallTime transform_time = ros::WallTime::now();

    ROS_INFO_STREAM(" 1. transform time: " <<  (transform_time - startTime).toSec());

    // 1. Update or insert
    if(ego_sphere->transform(sensorToWorld.cast <double> ()))
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

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        // Extract the inliers
        extract.setInputCloud (pc.makeShared());
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (pc);

        ros::WallTime filtering_time = ros::WallTime::now();

        ROS_INFO_STREAM(" 2. filtering time: " <<  (filtering_time - transform_time).toSec());

        tf::StampedTransform sensorToEgoTf;
        try
        {
            listener.waitForTransform(ego_frame_id, stereo_data->point_clouds.rgb_point_cloud.header.frame_id, stereo_data->point_clouds.rgb_point_cloud.header.stamp, ros::Duration(10.0) );
            listener.lookupTransform(ego_frame_id, stereo_data->point_clouds.rgb_point_cloud.header.frame_id, stereo_data->point_clouds.rgb_point_cloud.header.stamp, sensorToEgoTf);
        } catch(tf::TransformException& ex){
            ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
            return;
        }


        Eigen::Matrix4f sensorToEgo;
        pcl_ros::transformAsMatrix(sensorToEgoTf, sensorToEgo);

        pcl::transformPointCloud (pc, pc, sensorToEgo);
        ros::WallTime transform_data_time = ros::WallTime::now();

        for(int i=0; i<informations.size();++i)
        {
            informations[i].noalias()=sensorToEgo.block(0,0,3,3).cast <double> ()*informations[i]*sensorToEgo.block(0,0,3,3).transpose().cast <double> ();
        }


        ROS_INFO_STREAM(" 3. transform data time: " <<  (transform_data_time - filtering_time).toSec());

        std::cout << pc.points.size() << std::endl;
        std::cout << informations.size() << std::endl;

        insertScan(pc,informations);

        ros::WallTime insert_time = ros::WallTime::now();
        ROS_INFO_STREAM(" 4. insertion time: " <<  (insert_time - transform_data_time).toSec());

        ROS_INFO_STREAM("   total points inserted: " <<   pc.size());

        publishAll(stereo_data);

        if(active_vision)
        {
            //ego_sphere->getClosestPoint();
            ROS_INFO("Waiting for action server to start.");
            // wait for the action server to start
            ac.waitForServer(ros::Duration(2.0)); //will wait for infinite time
            ROS_INFO("Started.");

            // send a goal to the action
            move_robot_msgs::GazeGoal goal;
            goal.fixation_point.header.frame_id=ego_frame_id;
            goal.fixation_point.header.stamp=ros::Time::now();

            if(ego_sphere->new_closest_point)
            {
                fixation_point(0)=ego_sphere->closest_point(0);
                fixation_point(1)=ego_sphere->closest_point(1);
                fixation_point(2)=ego_sphere->closest_point(2);

                goal.fixation_point.point.x = fixation_point(0);
                goal.fixation_point.point.y = fixation_point(1);
                goal.fixation_point.point.z = fixation_point(2);
            }
            else
            {
                Eigen::Vector3d random_point;
                Mat aux(1, 1, CV_64F);

                // Generate random patch on the sphere surface
                cv::randn(aux, 0, 0.1);
                random_point(0,0)=aux.at<double>(0,0);

                cv::randn(aux, 0, 0.1);
                random_point(1,0)=aux.at<double>(0,0);

                cv::randn(aux, 0, 0.1);
                random_point(2,0)=aux.at<double>(0,0);
                fixation_point(0)=ego_sphere->closest_point(0)+random_point(0,0);
                fixation_point(1)=ego_sphere->closest_point(1)+random_point(1,0);
                fixation_point(2)=ego_sphere->closest_point(2)+random_point(2,0);

                goal.fixation_point.point.x = fixation_point(0);
                goal.fixation_point.point.y = fixation_point(1);
                goal.fixation_point.point.z = fixation_point(2);
            }


            ac.sendGoal(goal);

            //wait for the action to return
            bool finished_before_timeout = ac.waitForResult(ros::Duration(7.0));

            if (finished_before_timeout)
            {
                actionlib::SimpleClientGoalState state = ac.getState();
                ROS_INFO("Action finished: %s",state.toString().c_str());
            }
            else
            {
                ROS_ERROR("Action did not finish before the time out.");
            }
        }
    }

    double total_elapsed = (ros::WallTime::now() - startTime).toSec();

    ROS_INFO(" TOTAL TIME:  %f sec", total_elapsed);
}

void EgoSphereManagerRos::insertScan(const PCLPointCloud& point_cloud, const std::vector<Eigen::Matrix3d> & covariances)
{
    //ego_sphere->insert(point_cloud);
    ego_sphere->insertHashTable(point_cloud, covariances);
}

void EgoSphereManagerRos::publishAll(const foveated_stereo_ros::StereoDataConstPtr& stereo_data)
{
    pcl::PointCloud<pcl::PointXYZRGB> point_cloud=ego_sphere->getPointCloud();
    point_cloud.header.frame_id=ego_frame_id;
    sensor_msgs::PointCloud2 rgb_point_cloud_msg;
    pcl::toROSMsg(point_cloud,rgb_point_cloud_msg);
    rgb_point_cloud_msg.is_dense=false;
    rgb_point_cloud_msg.header.stamp=stereo_data->header.stamp;
    rgb_point_cloud_publisher.publish(rgb_point_cloud_msg);

    pcl::PointCloud<pcl::PointXYZI> point_cloud_uncertainty_viz=ego_sphere->getPointCloudUncertaintyViz();
    point_cloud_uncertainty_viz.header.frame_id=ego_frame_id;
    sensor_msgs::PointCloud2 uncertainty_point_cloud_viz_msg;
    pcl::toROSMsg(point_cloud_uncertainty_viz, uncertainty_point_cloud_viz_msg);
    uncertainty_point_cloud_viz_msg.header=stereo_data->point_clouds.uncertainty_point_cloud.header;
    uncertainty_point_cloud_publisher.publish(uncertainty_point_cloud_viz_msg);


    foveated_stereo_ros::PointClouds ego_point_clouds_msg;

    sensor_msgs::PointCloud2 rgb_point_cloud_msg_base;
    pcl_ros::transformPointCloud(world_frame_id, rgb_point_cloud_msg, rgb_point_cloud_msg_base, listener);

    pcl::PointCloud<pcl::PointXYZI> point_cloud_uncertainty=ego_sphere->getPointCloudUncertainty();
    point_cloud_uncertainty.header.frame_id=ego_frame_id;
    sensor_msgs::PointCloud2 uncertainty_point_cloud_msg;
    pcl::toROSMsg(point_cloud_uncertainty, uncertainty_point_cloud_msg);
    uncertainty_point_cloud_msg.header=stereo_data->point_clouds.uncertainty_point_cloud.header;

    sensor_msgs::PointCloud2 uncertainty_point_cloud_msg_base;
    pcl_ros::transformPointCloud(world_frame_id, uncertainty_point_cloud_msg, uncertainty_point_cloud_msg_base, listener);

    ego_point_clouds_msg.rgb_point_cloud=rgb_point_cloud_msg_base;
    ego_point_clouds_msg.uncertainty_point_cloud=uncertainty_point_cloud_msg_base;

    foveated_stereo_ros::EgoData ego_data_msg;

    sensor_msgs::PointCloud2 sensor_rgb_point_cloud_msg_base;
    sensor_msgs::PointCloud2 sensor_uncertainty_point_cloud_msg_base;

    pcl_ros::transformPointCloud(world_frame_id, stereo_data->point_clouds.rgb_point_cloud, sensor_rgb_point_cloud_msg_base, listener);
    pcl_ros::transformPointCloud(world_frame_id, stereo_data->point_clouds.uncertainty_point_cloud, sensor_uncertainty_point_cloud_msg_base, listener);

    ego_data_msg.sensor_point_clouds.rgb_point_cloud=sensor_rgb_point_cloud_msg_base;
    ego_data_msg.sensor_point_clouds.uncertainty_point_cloud=sensor_uncertainty_point_cloud_msg_base;
    ego_data_msg.ego_point_clouds=ego_point_clouds_msg;

    geometry_msgs::PointStamped fixation_point_ego;
    fixation_point_ego.header.frame_id=ego_frame_id;
    fixation_point_ego.point.x = fixation_point(0);
    fixation_point_ego.point.y = fixation_point(1);
    fixation_point_ego.point.z = fixation_point(2);
    geometry_msgs::PointStamped fixation_point_world;

    try
    {
        listener.waitForTransform(world_frame_id, ego_frame_id, ros::Time(0), ros::Duration(10.0) );
        listener.transformPoint(world_frame_id, ros::Time(0),fixation_point_ego,ego_frame_id,fixation_point_world);
    } catch(tf::TransformException& ex){
        ROS_DEBUG_STREAM( "Transform lookup failed: " << ex.what());
        exit(-1);
    }

    ego_data_msg.fixation_point=fixation_point_world;

    point_clouds_publisher.publish(ego_data_msg);
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
        ego_sphere.publishEgoStructure();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
} // end




