#include "EgoSphereManagerRos.h"
#include <rosbag/bag.h>
bool fileExists(const std::string& filename)
{
    struct stat buf;
    if (stat(filename.c_str(), &buf) != -1)
    {
        return true;
    }
    return false;
}

static int iterations_=0;
EgoSphereManagerRos::EgoSphereManagerRos(ros::NodeHandle & nh_, ros::NodeHandle & private_node_handle_) :
    nh(nh_),
    ac("gaze", true),
    current_fixation_point(Eigen::Vector4d::Constant(std::numeric_limits<double>::max())),
    next_fixation_point(Eigen::Vector4d::Constant(std::numeric_limits<double>::max())),
    listener(new tf::TransformListener(ros::Duration(30.0))),
    current_fixation_point_index(10000000)
{
    std::cout << '\a';
    current_fixation_point(3)=1; // Homogeneous coordinates
    next_fixation_point(3)=1; // Homogeneous coordinates

    // Declare variables that can be modified by launch file or command line.
    int egosphere_nodes;
    double mahalanobis_distance_threshold;
    double sigma_scale_upper_bound;
    double neighbour_angle_threshold;
    double update_frequency;
    double init_scale_mean;
    double init_scale_information;
    double pan_abs_limit;
    double tilt_abs_limit;

    std::string data_folder;
    private_node_handle_.param<std::string>("world_frame",world_frame_id,"world");
    private_node_handle_.param<std::string>("ego_frame",ego_frame_id,"eyes_center_vision_link");
    private_node_handle_.param<std::string>("eyes_center_frame", eyes_center_frame_id, "eyes_center_frame");
    private_node_handle_.param<std::string>("base_frame", base_frame_id, "base_link");
    private_node_handle_.param<std::string>("data_folder", data_folder, "data_folder");

    private_node_handle_.param("egosphere_nodes", egosphere_nodes, 1000);
    private_node_handle_.param("active_vision",active_vision,false);
    private_node_handle_.param("sigma_scale_upper_bound",sigma_scale_upper_bound,1.0);
    private_node_handle_.param("mahalanobis_distance_threshold",mahalanobis_distance_threshold,std::numeric_limits<double>::max());
    private_node_handle_.param("neighbour_angle_threshold",neighbour_angle_threshold,1.0);
    private_node_handle_.param("fixation_point_error_tolerance",fixation_point_error_tolerance,0.01);
    private_node_handle_.param("gaze_timeout",gaze_timeout,1.0);
    private_node_handle_.param("field_of_view",field_of_view,1.0);
    private_node_handle_.param("update_mode",update_mode, true);
    private_node_handle_.param("relative_update",relative_update, true);
    private_node_handle_.param("update_frequency",update_frequency, 20.0);
    private_node_handle_.param("sensory_filtering_sphere_radius",sensory_filtering_sphere_radius, 20.0);
    private_node_handle_.param("init_scale_mean",init_scale_mean, 50.0);
    private_node_handle_.param("init_scale_information",init_scale_information, 0.0001);

    private_node_handle_.param("pan_abs_limit",pan_abs_limit, 1.0);
    private_node_handle_.param("tilt_abs_limit",tilt_abs_limit, 1.0);

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

    XmlRpc::XmlRpcValue perturb_std_dev_list;
    private_node_handle_.getParam("perturb_standard_deviation", perturb_std_dev_list);
    ROS_ASSERT(std_dev_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    perturb_standard_deviation_mat=cv::Mat(3, 1, CV_64F);

    for (int32_t i = 0; i < std_dev_list.size(); ++i)
    {
        ROS_ASSERT(perturb_std_dev_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        perturb_standard_deviation_mat.at<double>(i,0)=static_cast<double>(perturb_std_dev_list[i]);
    }


    ROS_INFO_STREAM("egosphere_nodes: "<<egosphere_nodes);

    ROS_INFO_STREAM("mahalanobis_distance_threshold: "<<mahalanobis_distance_threshold);
    ROS_INFO_STREAM("mean: "<<mean_mat);
    ROS_INFO_STREAM("standard_deviation: "<<standard_deviation_mat);
    ROS_INFO_STREAM("perturb_standard_deviation: "<<perturb_standard_deviation_mat);

    ROS_INFO_STREAM("active_vision: "<<active_vision);
    ROS_INFO_STREAM("world_frame_id: "<<world_frame_id);
    ROS_INFO_STREAM("ego_frame_id: "<<ego_frame_id);
    ROS_INFO_STREAM("eyes_center_frame_id: "<<eyes_center_frame_id);
    ROS_INFO_STREAM("base_frame_id: "<<base_frame_id);
    ROS_INFO_STREAM("neighbour_angle_threshold: "<<neighbour_angle_threshold);
    ROS_INFO_STREAM("fixation_point_error_tolerance: "<<fixation_point_error_tolerance);
    ROS_INFO_STREAM("gaze_timeout: "<<gaze_timeout);
    ROS_INFO_STREAM("field_of_view: "<<field_of_view);
    ROS_INFO_STREAM("update_mode: "<<update_mode);
    ROS_INFO_STREAM("relative_update: "<<relative_update);
    ROS_INFO_STREAM("update_frequency: "<<update_frequency);
    ROS_INFO_STREAM("sensory_filtering_sphere_radius: "<<sensory_filtering_sphere_radius);
    ROS_INFO_STREAM("init_scale_mean: "<<init_scale_mean);
    ROS_INFO_STREAM("init_scale_information: "<<init_scale_information);
    ROS_INFO_STREAM("pan_abs_limit: "<<pan_abs_limit);
    ROS_INFO_STREAM("tilt_abs_limit: "<<tilt_abs_limit);

    ROS_DEBUG("Waiting for action server to start.");

    ac.waitForServer();

    ROS_DEBUG("Move to home position.");
    move_robot_msgs::GazeGoal goal;
    goal.fixation_point.header.frame_id=ego_frame_id;
    goal.type=move_robot_msgs::GazeGoal::HOME;
    goal.mode=move_robot_msgs::GazeGoal::JOINT;
    goal.fixation_point.header.stamp=ros::Time::now();
    ac.sendGoal(goal);

    if(!ac.waitForResult(ros::Duration(gaze_timeout)))
    {
        ROS_ERROR_STREAM("COULD NOT MOVE HOME! EXITING...");
        exit(-1);
    }

    // GET TRANSFORM FROM EGO FRAME TO SENSOR FRAME

    sleep(5.0); //GIVE TIME TO TF
    tf::StampedTransform transform;
    tf::StampedTransform egoToWorldTf;

    while(nh.ok())
    {
        try
        {
            listener->waitForTransform(eyes_center_frame_id, ego_frame_id, ros::Time(0), ros::Duration(10.0) );
            listener->lookupTransform(eyes_center_frame_id, ego_frame_id, ros::Time(0), transform);

            listener->waitForTransform(world_frame_id, ego_frame_id, ros::Time(0), ros::Duration(1.0) );
            listener->lookupTransform(world_frame_id, ego_frame_id, ros::Time(0), egoToWorldTf); // ABSOLUTE EGO TO WORLD
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            continue;
        }
        break;
    }

    pcl_ros::transformAsMatrix(egoToWorldTf, previousEgoToWorld);

    std::string ego_file_name;
    // 1. with Boost
    ego_file_name = data_folder+"/ego_sphere_nodes_" + boost::lexical_cast<std::string>(egosphere_nodes) + "_nn_angle_threshold_"+boost::lexical_cast<std::string>(neighbour_angle_threshold);
    ROS_INFO_STREAM("ego_file_name:"<<ego_file_name);

    if(fileExists(ego_file_name))
    {
        ROS_INFO("LOAD EGOSPHERE");
        // create and open an archive for input
        std::ifstream ifs(ego_file_name.c_str());
        boost::archive::binary_iarchive ia(ifs);
        // read class state from archive
        ego_sphere =  boost::shared_ptr<SphericalShell<std::vector< boost::shared_ptr<MemoryPatch> > > > (new SphericalShell<std::vector< boost::shared_ptr<MemoryPatch> > > ());
        ia >> ego_sphere;
    }
    else
    {
        ROS_INFO("CREATE NEW EGO SPHERE");

        ego_sphere = boost::shared_ptr<SphericalShell<std::vector< boost::shared_ptr<MemoryPatch> > > > (new SphericalShell<std::vector< boost::shared_ptr<MemoryPatch> > > (egosphere_nodes,
                                                                                                                                                                             mahalanobis_distance_threshold,
                                                                                                                                                                             mean_mat,
                                                                                                                                                                             standard_deviation_mat,
                                                                                                                                                                             transform.getOrigin().getY(),
                                                                                                                                                                             neighbour_angle_threshold,
                                                                                                                                                                             init_scale_mean,
                                                                                                                                                                             init_scale_information,
                                                                                                                                                                             pan_abs_limit,
                                                                                                                                                                             tilt_abs_limit
                                                                                                                                                                             ));
        std::ofstream ofs(ego_file_name.c_str());
        ROS_INFO("SAVE EGOSPHERE");
        // save data to archive
        {
            boost::archive::binary_oarchive oa(ofs);
            // write class instance to archive
            oa << ego_sphere;

            // archive and stream closed when destructors are called
        }
        ROS_INFO("DONE");
    }
    current_fixation_point_index=ego_sphere->init_fixation_point_index;

    // archive and stream closed when destructors are called
    // ... some time later restore the class instance to its orginal state
    ROS_INFO("DONE");

    decision_making = boost::shared_ptr<DecisionMaking> (new DecisionMaking(ego_sphere,sigma_scale_upper_bound));

    rgb_point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("ego_sphere", 1);
    uncertainty_point_cloud_publisher  = nh.advertise<sensor_msgs::PointCloud2>("ego_sphere_uncertainty", 1);
    point_clouds_publisher = nh.advertise<foveated_stereo_ros::EgoData>("ego_point_clouds", 1);
    information_publisher = nh.advertise<std_msgs::Float64>("total_information", 1);

    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("covariances_debug", 1);
    ses_structure_pub  = nh.advertise<sensor_msgs::PointCloud2>("ego_sphere_structure", 2);
    sensor_direction_pub  = nh.advertise<geometry_msgs::PointStamped>("direction_arrow", 2);

    stereo_data_subscriber_ = new message_filters::Subscriber<foveated_stereo_ros::StereoData> (nh, "stereo_data", 1);
    tf_filter_ = new tf::MessageFilter<foveated_stereo_ros::StereoData> (*stereo_data_subscriber_, *listener, world_frame_id, 1);
    tf_filter_->registerCallback(boost::bind(&EgoSphereManagerRos::insertCloudCallback, this, _1));
    last=ros::Time::now();

    if(relative_update)
    {
        update_timer = nh.createTimer(ros::Duration(1.0/update_frequency), &EgoSphereManagerRos::updateEgoSphereRelative, this);
    }
    else
    {
        update_timer = nh.createTimer(ros::Duration(1.0/update_frequency), &EgoSphereManagerRos::updateEgoSphereAbsolute, this);
    }

    ROS_INFO("DONE INIT");

    return;
}

void EgoSphereManagerRos::publishEgoStructure()
{
    sensor_msgs::PointCloud2 ego_sphere_msg;

    pcl::toROSMsg(*(ego_sphere->structure_cloud),ego_sphere_msg);

    sensor_msgs::PointCloud2 ego_sphere_msg_world;
    pcl_ros::transformPointCloud(egoToWorld, ego_sphere_msg, ego_sphere_msg_world);
    ego_sphere_msg_world.is_dense=false;
    ego_sphere_msg_world.header.frame_id=world_frame_id;
    ego_sphere_msg_world.header.stamp=last_update_time;
    ses_structure_pub.publish(ego_sphere_msg_world);
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
    ac.cancelAllGoals();
}

void EgoSphereManagerRos::updateEgoSphereRelative(const ros::TimerEvent&)
{
    ROS_INFO_STREAM("Updating Ego Sphere (relative):");

    ros::Time current_time;

    // 1. get transform from world to ego-sphere frame
    tf::StampedTransform EgoTransformTf;
    tf::StampedTransform egoToWorldTf;

    while(nh.ok())
    {
        try
        {
            current_time = ros::Time::now();
            listener->waitForTransform(ego_frame_id, current_time, ego_frame_id, last_update_time, world_frame_id, ros::Duration(1.0) );
            listener->lookupTransform(ego_frame_id, current_time, ego_frame_id, last_update_time, world_frame_id, EgoTransformTf); // DELTA EGO

            listener->waitForTransform(world_frame_id, ego_frame_id, current_time, ros::Duration(1.0) );
            listener->lookupTransform(world_frame_id, ego_frame_id, current_time, egoToWorldTf); // ABSOLUTE EGO TO WORLD
        }
        catch(tf::TransformException& ex)
        {
            ROS_ERROR_STREAM( "Transform error: " << ex.what() << ", quitting callback");
            continue;
        }
        break;
    }
    last_update_time=current_time;

    pcl_ros::transformAsMatrix(EgoTransformTf, egoTransform);
    pcl_ros::transformAsMatrix(egoToWorldTf, egoToWorld);

    ros::Time transform_time = ros::Time::now();

    ROS_INFO_STREAM(" 1. transform time: " <<  (transform_time - current_time).toSec());

    ///////////////////////
    // Update ego-sphere //
    ///////////////////////

    ego_sphere->transform(egoTransform.cast <double> (), update_mode, sensory_filtering_sphere_radius);

    ros::Time update_time = ros::Time::now();
    ROS_INFO_STREAM(" 2. Update time: " <<  (update_time - transform_time).toSec());

    ///////////////////////
    // Publish structure //
    ///////////////////////

    publishEgoStructure();

    ros::Time publish_time = ros::Time::now();
    ROS_INFO_STREAM(" 3. Publish time: " <<  (publish_time - update_time).toSec());

    ROS_INFO_STREAM("Done. Total update time:"<< (publish_time - current_time).toSec());
}


void EgoSphereManagerRos::updateEgoSphereAbsolute(const ros::TimerEvent&)
{
    ROS_INFO_STREAM("Updating Ego Sphere (absolute):");

    ros::Time current_time;

    // 1. get transform from world to ego-sphere frame
    tf::StampedTransform egoToWorldTf;

    while(nh.ok())
    {
        try
        {
            current_time = ros::Time::now();
            listener->waitForTransform(world_frame_id, ego_frame_id, current_time, ros::Duration(1.0) );
            listener->lookupTransform(world_frame_id, ego_frame_id, current_time, egoToWorldTf);
        }
        catch(tf::TransformException& ex)
        {
            ROS_ERROR_STREAM( "Transform error: " << ex.what() << ", quitting callback");
            continue;
        }
        break;
    }
    last_update_time=current_time;

    pcl_ros::transformAsMatrix(egoToWorldTf, egoToWorld);

    ros::Time transform_time = ros::Time::now();

    ROS_INFO_STREAM(" 1. transform time: " <<  (transform_time - current_time).toSec());

    ///////////////////////
    // Update ego-sphere //
    ///////////////////////
    ego_sphere->transform( (egoToWorld.inverse()*previousEgoToWorld).cast <double> (), update_mode, sensory_filtering_sphere_radius);

    previousEgoToWorld=egoToWorld;

    ros::Time update_time = ros::Time::now();
    ROS_INFO_STREAM(" 2. Update time: " <<  (update_time - transform_time).toSec());

    ///////////////////////
    // Publish structure //
    ///////////////////////

    publishEgoStructure();

    ros::Time publish_time = ros::Time::now();
    ROS_INFO_STREAM(" 3. Publish time: " <<  (publish_time - update_time).toSec());

    ROS_INFO_STREAM("Done. Total update time:"<< (publish_time - current_time).toSec());
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
            ros::Time current_time = ros::Time::now();

            listener->waitForTransform(world_frame_id, last_update_time, stereo_data->point_clouds.rgb_point_cloud.header.frame_id, stereo_data->point_clouds.rgb_point_cloud.header.stamp, world_frame_id, ros::Duration(10.0) );
            listener->lookupTransform(world_frame_id, last_update_time, stereo_data->point_clouds.rgb_point_cloud.header.frame_id, stereo_data->point_clouds.rgb_point_cloud.header.stamp, world_frame_id, sensorToWorldTf);
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
            listener->waitForTransform(ego_frame_id, last_update_time, stereo_data->point_clouds.rgb_point_cloud.header.frame_id, stereo_data->point_clouds.rgb_point_cloud.header.stamp, world_frame_id, ros::Duration(10.0) );
            listener->lookupTransform (ego_frame_id, last_update_time, stereo_data->point_clouds.rgb_point_cloud.header.frame_id, stereo_data->point_clouds.rgb_point_cloud.header.stamp, world_frame_id, sensorToEgoTf);
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
    pcl::PointIndices removed_indices;
    pass.getRemovedIndices(removed_indices);

    // WHAT ABOUT FILTERING THE INFORMATIONS?!
    //ROS_ERROR_STREAM("pc size:"<<pc.size());
    //ROS_ERROR_STREAM("informations size before:"<< informations.size());

    informations=erase_indices<Eigen::Matrix3d>(informations, removed_indices.indices);
    //ROS_ERROR_STREAM("informations size after:"<< informations.size());

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

    //////////
    // SHOW //
    //////////

    ros::WallTime publish_time_before = ros::WallTime::now();

    publishAll(stereo_data);
    ros::WallTime publish_time_after = ros::WallTime::now();
    ROS_INFO_STREAM(" 6. publish time: " <<  (publish_time_after - publish_time_before).toSec());

    publishCovarianceMatrices();

    /////////
    // ACT //
    /////////
    next_fixation_point_index=decision_making->getFixationPoint(sensory_filtering_sphere_radius);

    /*if(current_fixation_point_index==next_fixation_point_index)
    {
        ego_sphere->structure[next_fixation_point_index]->sensory_data.position.reset();
    }*/
    current_fixation_point_index=next_fixation_point_index;
    Eigen::Vector3d fixation_point_3d=ego_sphere->structure[next_fixation_point_index]->sensory_data.position.mean;
    ros::WallTime decision_time = ros::WallTime::now();
    ROS_INFO_STREAM(" 4. decision making time: " <<  (decision_time - insert_time).toSec());



    next_fixation_point(0)=fixation_point_3d(0);
    next_fixation_point(1)=fixation_point_3d(1);
    next_fixation_point(2)=fixation_point_3d(2);
    // send a goal to the action
    move_robot_msgs::GazeGoal goal;
    goal.fixation_point.header.frame_id=ego_frame_id;
    goal.type=move_robot_msgs::GazeGoal::FIXATION_POINT;
    goal.mode=move_robot_msgs::GazeGoal::CARTESIAN;

    goal.fixation_point.header.stamp=ros::Time::now();
    goal.fixation_point.point.x = next_fixation_point(0);
    goal.fixation_point.point.y = next_fixation_point(1);
    goal.fixation_point.point.z = next_fixation_point(2);
    goal.fixation_point_error_tolerance = fixation_point_error_tolerance;

    ROS_DEBUG("Waiting for action server to start.");
    // wait for the action server to start
    if(ac.waitForServer()&&active_vision&&nh.ok()) //with no duration will wait for infinite time
    {
        //wait for the action to return
        ac.sendGoal(goal);
        bool finished_before_timeout = ac.waitForResult(ros::Duration(gaze_timeout));
        actionlib::SimpleClientGoalState state = ac.getState();

        if (finished_before_timeout)
        {
            if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Action succeeded: %s",state.toString().c_str());
            }
            else
            {
                ROS_ERROR("Action failed1: %s",state.toString().c_str());

                Eigen::Vector3d fixation_point_perturb;
                do
                {
                    fixation_point_perturb=perturb(next_fixation_point, perturb_standard_deviation_mat);
                    // send a goal to the action
                    goal.fixation_point.header.stamp=ros::Time::now();
                    goal.fixation_point.point.x = fixation_point_perturb(0);
                    goal.fixation_point.point.y = fixation_point_perturb(1);
                    goal.fixation_point.point.z = fixation_point_perturb(2);

                    ac.sendGoal(goal);
                    bool finished_before_timeout = ac.waitForResult(ros::Duration(gaze_timeout));
                    state = ac.getState();
                    if (finished_before_timeout)
                    {
                        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
                        {
                            ROS_INFO("Action succeeded: %s",state.toString().c_str());
                            break;
                        }
                        else
                        {
                            ROS_ERROR("Action failed2: %s",state.toString().c_str());
                            ac.cancelAllGoals();
                        }
                    }
                    else
                    {
                        ROS_ERROR("Action did not finish before the time out.");
                        ac.cancelAllGoals();
                        break;
                    }
                }
                while(nh.ok());
            }
        }
        else
        {
            ROS_ERROR("Action failed due to timeout: %s",state.toString().c_str());
            ac.cancelAllGoals();
        }
    }
    ros::TimerEvent aux;
    updateEgoSphereAbsolute(aux);
    ros::WallTime moving_time = ros::WallTime::now();
    ROS_INFO_STREAM(" 5. saccade time: " <<  (moving_time - decision_time).toSec());

    double total_elapsed = (ros::WallTime::now() - start_time).toSec();

    ROS_INFO_STREAM("Done. Total insertion time:"<< total_elapsed);

    ROS_ERROR_STREAM("ITERATION:"<<++iterations_);

    if(iterations_>=100)
        std::cout << '\a';
}


void EgoSphereManagerRos::insertScan(const PCLPointCloud& point_cloud, const std::vector<Eigen::Matrix3d> & covariances)
{
    //ego_sphere->insert(point_cloud);
    //ego_sphere->insertHashTable(point_cloud, covariances);
    Eigen::Matrix3d sensor_to_normal;
    sensor_to_normal <<  0, 0, 1,
            -1, 0, 0,
            0,-1, 0;

    Eigen::Vector3d sensor_direction;//

    //sensor_direction=sensor_to_normal.transpose()*sensorToWorld.block<3,3>(0,0).cast<double>()*Eigen::Vector3d::UnitZ();

    sensor_direction=sensorToEgo.block<3,3>(0,0).cast<double>()*Eigen::Vector3d::UnitZ();
    //std::cout << "sensor_direction:"<< sensor_direction.transpose() << std::endl;
    //std::cout << sensorToEgo.block<3,3>(0,0).cast<double>() << std::endl;

    geometry_msgs::PointStamped sensor_direction_msg;

    sensor_direction_msg.header.frame_id=ego_frame_id;
    sensor_direction_msg.header.stamp=ros::Time::now();
    sensor_direction_msg.point.x=sensor_direction(0);
    sensor_direction_msg.point.y=sensor_direction(1);
    sensor_direction_msg.point.z=sensor_direction(2);

    sensor_direction_pub.publish(sensor_direction_msg);
    //sensor_direction=Eigen::Vector3d::UnitZ();
    ego_sphere->insertKdTree(point_cloud, covariances, sensor_direction.normalized(), field_of_view, sensory_filtering_sphere_radius);
}

void EgoSphereManagerRos::publishAll(const foveated_stereo_ros::StereoDataConstPtr& stereo_data)
{
    pcl::PointCloud<pcl::PointXYZRGB> rgb_point_cloud=ego_sphere->getPointCloud();
    pcl::transformPointCloud (ego_sphere->getPointCloud(), rgb_point_cloud, egoToWorld);
    rgb_point_cloud.header.frame_id=world_frame_id;
    rgb_point_cloud.is_dense=false;
    //pcl_conversions::toPCL(last_update_time, rgb_point_cloud.header.stamp);
    rgb_point_cloud.header.stamp=last_update_time.toNSec() / 1000ull;  // Convert from ns to us
    sensor_msgs::PointCloud2 rgb_point_cloud_msg;
    pcl::toROSMsg(rgb_point_cloud,rgb_point_cloud_msg);

    rgb_point_cloud_publisher.publish(rgb_point_cloud_msg);

    pcl::PointCloud<pcl::PointXYZI> point_cloud_uncertainty_viz;
    pcl::transformPointCloud (ego_sphere->getPointCloudUncertaintyViz(), point_cloud_uncertainty_viz, egoToWorld);
    point_cloud_uncertainty_viz.header.frame_id=world_frame_id;
    point_cloud_uncertainty_viz.is_dense=false;
    //pcl_conversions::toPCL(ros::Time::now(), point_cloud_uncertainty_viz.header.stamp);
    point_cloud_uncertainty_viz.header.stamp=ros::Time::now().toNSec() / 1000ull;  // Convert from ns to us
    sensor_msgs::PointCloud2 uncertainty_point_cloud_viz_msg;
    pcl::toROSMsg(point_cloud_uncertainty_viz, uncertainty_point_cloud_viz_msg);
    uncertainty_point_cloud_publisher.publish(uncertainty_point_cloud_viz_msg);

    pcl::PointCloud<pcl::PointXYZI> point_cloud_uncertainty;
    pcl::transformPointCloud (ego_sphere->getPointCloudUncertainty(), point_cloud_uncertainty, egoToWorld);
    point_cloud_uncertainty.header.frame_id=world_frame_id;
    point_cloud_uncertainty.is_dense=false;
    //pcl_conversions::toPCL(ros::Time::now(), point_cloud_uncertainty.header.stamp);
    point_cloud_uncertainty.header.stamp=last_update_time.toNSec() / 1000ull;  // Convert from ns to us
    sensor_msgs::PointCloud2 uncertainty_point_cloud_msg;
    pcl::toROSMsg(point_cloud_uncertainty, uncertainty_point_cloud_msg);


    foveated_stereo_ros::PointClouds ego_point_clouds_msg;
    ego_point_clouds_msg.rgb_point_cloud=rgb_point_cloud_msg;
    ego_point_clouds_msg.uncertainty_point_cloud=uncertainty_point_cloud_msg;

    foveated_stereo_ros::EgoData ego_data_msg;

    sensor_msgs::PointCloud2 sensor_rgb_point_cloud_msg_world;
    pcl_ros::transformPointCloud(sensorToWorld, stereo_data->point_clouds.rgb_point_cloud, sensor_rgb_point_cloud_msg_world);

    sensor_msgs::PointCloud2 sensor_uncertainty_point_cloud_msg_world;

    pcl_ros::transformPointCloud(sensorToWorld, stereo_data->point_clouds.uncertainty_point_cloud, sensor_uncertainty_point_cloud_msg_world);

    ego_data_msg.sensor_point_clouds.rgb_point_cloud=sensor_rgb_point_cloud_msg_world;
    ego_data_msg.sensor_point_clouds.uncertainty_point_cloud=sensor_uncertainty_point_cloud_msg_world;
    ego_data_msg.ego_point_clouds=ego_point_clouds_msg;

    Eigen::Vector3d current_fixation_point_3d=ego_sphere->structure[current_fixation_point_index]->sensory_data.position.mean;
    current_fixation_point(0)=current_fixation_point_3d(0);
    current_fixation_point(1)=current_fixation_point_3d(1);
    current_fixation_point(2)=current_fixation_point_3d(2);

    Eigen::Vector4f fixation_point_world=(egoToWorld)*current_fixation_point.cast<float>();
    geometry_msgs::PointStamped fixation_point_world_msg;
    fixation_point_world_msg.header.frame_id=world_frame_id;
    fixation_point_world_msg.point.x = fixation_point_world(0);
    fixation_point_world_msg.point.y = fixation_point_world(1);
    fixation_point_world_msg.point.z = fixation_point_world(2);

    ego_data_msg.fixation_point=fixation_point_world_msg;
    ego_data_msg.fixation_point_index=current_fixation_point_index;
    point_clouds_publisher.publish(ego_data_msg);

    double total_uncertainty=0;
    for(pcl::PointCloud<pcl::PointXYZI>::iterator pcl_it = point_cloud_uncertainty.begin(); pcl_it != point_cloud_uncertainty.end(); ++pcl_it)
    {
        double uncert=log((double)pcl_it->intensity);
        if(isinf(uncert)||isnan(uncert))
        {
            continue;
        }
        total_uncertainty+=uncert;
    }
    std_msgs::Float64 total_uncertainty_msg;
    total_uncertainty_msg.data=total_uncertainty/point_cloud_uncertainty.points.size();
    information_publisher.publish(total_uncertainty_msg);
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
Eigen::Vector3d EgoSphereManagerRos::perturb(const Eigen::Vector4d & fixation_point, const cv::Mat & perturb_standard_deviation_mat_)
{
    cv::Mat aux(1, 1, CV_64F);
    Eigen::Vector3d fixation_point_perturb;
    // Generate random patch on the sphere surface
    cv::randn(aux, 0, perturb_standard_deviation_mat_.at<double>(0,0));
    fixation_point_perturb(0,0)=fixation_point.x()+aux.at<double>(0,0);

    cv::randn(aux, 0, perturb_standard_deviation_mat_.at<double>(1,0));
    fixation_point_perturb(1,0)=fixation_point.y()+aux.at<double>(0,0);

    cv::randn(aux, 0, perturb_standard_deviation_mat_.at<double>(2,0));
    fixation_point_perturb(2,0)=fixation_point.z()+aux.at<double>(0,0);
    //cv::randn(aux, 0, 0.1);
    //fixation_point_perturb= fixation_point_normalized*aux.at<double>(0,0)+fixation_point;

    return fixation_point_perturb;
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

        ros::spinOnce();
        r.sleep();
    }

    return 0;
} // end




