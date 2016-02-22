#include "BayesianFilterNode.h"

using namespace MatrixWrapper;
using namespace BFL;
using namespace std;

BayesianFilterNode::BayesianFilterNode(const ros::NodeHandle &nh) :
    nh_(nh),
    private_node_handle_("~"),
    odom_initialized(false),
    listener(new tf::TransformListener(ros::Duration(2.0)))
{
    odom_sub = nh_.subscribe("odom", 1, &BayesianFilterNode::odomCallback, this);
    stereo_sub = nh_.subscribe("stereo", 1, &BayesianFilterNode::stereoCallback, this);
    pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/pose_pf",1);
    particle_pub = nh_.advertise<sensor_msgs::PointCloud2>("/particle_cloud",1);
    dt = 0.0;

    double prior_mean;
    double prior_std_dev;
    int particles_number;
    // Initial estimate of position and orientation
    private_node_handle_.param("prior_mean",prior_mean, 1.0);
    private_node_handle_.param("prior_std_dev",prior_std_dev, 0.01);
    private_node_handle_.param("particles_number",particles_number, 100);

    ROS_INFO_STREAM("prior_mean: "<<prior_mean);

    createParticleFilter(prior_mean,
                         prior_std_dev,
                         particles_number);
}

BayesianFilterNode::~BayesianFilterNode()
{}

void BayesianFilterNode::createParticleFilter(const double & prior_mean,
                                              const double & prior_std_dev,
                                              const int & particles_number)
{
    /****************************
     * NonLinear system model   *
     ***************************/
    // create gaussian
    ColumnVector sys_noise_Mu(STATE_SIZE);
    sys_noise_Mu(1) = 0.0;
    sys_noise_Mu(2) = 0.0;
    sys_noise_Mu(3) = 0.0;

    SymmetricMatrix sys_noise_Cov(STATE_SIZE);
    sys_noise_Cov = 0.0;
    sys_noise_Cov(1,1) = 0.001;
    sys_noise_Cov(1,2) = 0.0;
    sys_noise_Cov(1,3) = 0.0;
    sys_noise_Cov(2,1) = 0.0;
    sys_noise_Cov(2,2) = 0.001;
    sys_noise_Cov(2,3) = 0.0;
    sys_noise_Cov(3,1) = 0.0;
    sys_noise_Cov(3,2) = 0.0;
    sys_noise_Cov(3,3) = 0.001;

    Gaussian system_Uncertainty(sys_noise_Mu, sys_noise_Cov);

    // create the nonlinear system model
    sys_pdf = boost::shared_ptr<BFL::NonlinearSystemPdf> (new  BFL::NonlinearSystemPdf(system_Uncertainty));
    sys_model = boost::shared_ptr<BFL::SystemModel<ColumnVector> > (new BFL::SystemModel<ColumnVector> (sys_pdf.get()));

    /*********************************
     * NonLinear Measurement model   *
     ********************************/
    // Construct the measurement noise (a scalar in this case)
    ColumnVector meas_noise_Mu(MEAS_SIZE);
    meas_noise_Mu(1) = 0.0;
    meas_noise_Mu(2) = 0.0;
    meas_noise_Mu(3) = 0.0;
    SymmetricMatrix meas_noise_Cov(MEAS_SIZE);
    meas_noise_Cov(1,1) = 0.001;
    meas_noise_Cov(1,2) = 0.0;
    meas_noise_Cov(1,3) = 0.0;
    meas_noise_Cov(2,1) = 0.0;
    meas_noise_Cov(2,2) = 0.001;
    meas_noise_Cov(2,3) = 0.0;
    meas_noise_Cov(3,1) = 0.0;
    meas_noise_Cov(3,2) = 0.0;
    meas_noise_Cov(3,3) = 0.001;

    Gaussian measurement_Uncertainty(meas_noise_Mu, meas_noise_Cov);

    Matrix H(MEAS_SIZE,STATE_SIZE);
    H = 0.0;
    H(1,1) = 1.0;
    H(2,2) = 1.0;
    H(3,3) = 1.0;
    meas_pdf = boost::shared_ptr<BFL::LinearAnalyticConditionalGaussian>(new BFL::LinearAnalyticConditionalGaussian(H,measurement_Uncertainty));
    meas_model = boost::shared_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty>(new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(meas_pdf.get()));

    /****************************
     * Linear prior DENSITY     *
     ***************************/
    // Continuous Gaussian prior (for Kalman filters)
    ColumnVector prior_Mu(STATE_SIZE);
    prior_Mu(1) = 0;
    prior_Mu(2) = 0;
    prior_Mu(3) = prior_mean;
    SymmetricMatrix prior_Cov(STATE_SIZE);
    prior_Cov(1,1) = prior_std_dev;
    prior_Cov(1,2) = 0.0;
    prior_Cov(1,3) = 0.0;
    prior_Cov(2,1) = 0.0;
    prior_Cov(2,2) = prior_std_dev;
    prior_Cov(2,3) = 0.0;
    prior_Cov(3,1) = 0.0;
    prior_Cov(3,2) = 0.0;
    prior_Cov(3,3) = prior_std_dev;
    Gaussian prior_cont(prior_Mu,prior_Cov);

    // Discrete prior for Particle filter (using the continuous Gaussian prior)
    vector<Sample<ColumnVector> > prior_samples(particles_number);
    MCPdf<ColumnVector> prior_discr(particles_number,STATE_SIZE);
    prior_cont.SampleFrom(prior_samples,particles_number,CHOLESKY,NULL);
    prior_discr.ListOfSamplesSet(prior_samples);

    /******************************
     * Construction of the Filter *
     ******************************/
    filter=boost::shared_ptr<CustomParticleFilter> (new CustomParticleFilter(&prior_discr, 0, particles_number/4.0));
}

void BayesianFilterNode::stereoCallback(const StereoConstPtr & msg)
{
    tf::StampedTransform transform;
    while(nh.ok())
    {
        try
        {
            listener->waitForTransform(msg->header.frame_id, "ego_frame", ros::Time(0), ros::Duration(10.0) );
            listener->lookupTransform(msg->header.frame_id, "ego_frame", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            continue;
        }
        break;
    }

    pcl::PointCloud<pcl::PointXYZRGB> pc; // input cloud
    pcl::fromROSMsg(msg->point_clouds.rgb_point_cloud, pc);

    // Filter data
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    pcl::fromROSMsg(msg->point_clouds.rgb_point_cloud, pc);
    pcl::removeNaNFromPointCloud(pc, pc, inliers->indices);

    std::vector<Eigen::Matrix3d> covariances;
    covariances.reserve(msg->covariances.size());

    for(int inlier_index=0; inlier_index<inliers->indices.size();++inlier_index)
    {
        int c=inliers->indices[inlier_index];
        Eigen::Matrix3d covariance;
        for(int i=0; i<3; ++i)
        {
            for(int j=0; j<3; ++j)
            {
                int index=j+i*3;
                covariance(i,j)=msg->covariances[c].covariance[index];
            }
        }
        covariances.push_back(covariance);
    }

    ColumnVector measurement(3);
    measurement(1) = pc[0].x;
    measurement(2) = pc[0].y;
    measurement(3) = pc[0].z;
    ROS_INFO_STREAM("Measurement: "<<measurement);

    filter->Update(meas_model.get(), measurement);
    PublishParticles();
    PublishPose();
}

void BayesianFilterNode::odomCallback(const OdomConstPtr & msg)
{
    ros::Time odom_init_time = ros::Time::now();

    // Get transform from base footprint to ego_centric frame (proprioception / maybe some uncertainty here?)
    tf::StampedTransform ego_to_base_footprint_tf;
    try
    {
        listener->waitForTransform("base_footprint", "ego_frame",  msg->header.stamp, ros::Duration(0.05) );
        listener->lookupTransform("base_footprint", "ego_frame", msg->header.stamp, ego_to_base_footprint_tf); // ABSOLUTE EGO TO WORLD
    }
    catch (tf::TransformException &ex)
    {
        ROS_DEBUG("%s",ex.what());
        return;
    }

    geometry_msgs::Pose ego_to_base_footprint_msg;
    tf::poseTFToMsg(ego_to_base_footprint_tf, ego_to_base_footprint_msg);
    boost::shared_ptr<nav_msgs::Odometry> msg_ego(new nav_msgs::Odometry());
    /*nav_msgs::Odometry msg_aux(*msg);
    msg_aux.pose.covariance[0]=0.001;
    msg_aux.pose.covariance[7]=0.001;
    msg_aux.pose.covariance[14]=0.000;*/

    pose_cov_ops::compose(msg->pose, ego_to_base_footprint_msg, msg_ego->pose);

    if(!odom_initialized)
    {
        odom_initialized=true;
        last_odom_msg=msg_ego;
        return;
    }

    //if(!last_odom_msg->header.stamp.isZero()) dt = (msg->header.stamp - last_odom_msg->header.stamp).toSec();

    geometry_msgs::PoseWithCovariance delta_transf;
    pose_cov_ops::inverseCompose(last_odom_msg->pose, msg_ego->pose, delta_transf);

    tf::Pose new_odom_transform;
    tf::poseMsgToTF(msg_ego->pose.pose, new_odom_transform);
    tf::Pose old_odom_transform;
    tf::poseMsgToTF(last_odom_msg->pose.pose, old_odom_transform);

    tf::Pose delta_transform;
    tf::poseMsgToTF(delta_transf.pose, delta_transform);

    BFL::Matrix transf_matrix(4,4);
    decomposeTransform(delta_transform,transf_matrix);

    last_odom_msg = msg_ego;

    ColumnVector input(16);

    for(unsigned int row=1; row<=4; ++row)
    {
        for(unsigned int col=1; col<=4; ++col)
        {
            unsigned int index=col+(row-1)*4;
            input(index)=transf_matrix(row,col);
        }
    }

    //std::cout << delta_transf.covariance[0] << std::endl;
    SymmetricMatrix odom_cov(STATE_SIZE);
    odom_cov(1,1) = 0.001;
    odom_cov(1,2) = 0.0;
    odom_cov(1,3) = 0.0;
    odom_cov(2,1) = 0.0;
    odom_cov(2,2) = 0.001;
    odom_cov(2,3) = 0.0;
    odom_cov(3,1) = 0.0;
    odom_cov(3,2) = 0.0;
    odom_cov(3,3) = 0.001;
    sys_pdf->AdditiveNoiseSigmaSet(odom_cov);
    filter->Update(sys_model.get(),input);
    PublishParticles();

    ros::Time odom_end_time = ros::Time::now();
    ROS_INFO_STREAM(" prediction_time: " <<  (odom_end_time - odom_init_time).toSec());
}

void BayesianFilterNode::decomposeTransform(const tf::Pose& transform, BFL::Matrix& transf_matrix)
{
    transf_matrix=0.0;
    transf_matrix(4,4)=1.0;

    for(int row=0; row<3;++row)
    {
        // rotation
        for(int col=0; col<3; ++col)
        {
            transf_matrix(row+1,col+1)=transform.getBasis().getRow(row)[col];
        }

        // translation
        transf_matrix(row+1,4)=transform.getOrigin()[row];
    }
}

void BayesianFilterNode::PublishParticles()
{
    pcl::PointCloud<pcl::PointXYZ> point_cloud;

    point_cloud.header.frame_id = "/ego_frame";
    pcl_conversions::toPCL(ros::Time::now(), point_cloud.header.stamp);
    vector<WeightedSample<ColumnVector> >::iterator sample_it;
    vector<WeightedSample<ColumnVector> > samples;

    samples = filter->getNewSamples();

    for(sample_it = samples.begin(); sample_it<samples.end(); sample_it++)
    {
        ColumnVector sample = (*sample_it).ValueGet();

        //std::cout << sample << std::endl;
        pcl::PointXYZ point;
        point.x=sample(1);
        point.y=sample(2);
        point.z=sample(3);
        point_cloud.push_back(point);
    }
    sensor_msgs::PointCloud2 point_cloud_msg;
    pcl::toROSMsg(point_cloud, point_cloud_msg);
    particle_pub.publish(point_cloud_msg);//*/
}

void BayesianFilterNode::PublishPose()
{
    /*Pdf<ColumnVector> * posterior = filter->PostGet();
    ColumnVector pose = posterior->ExpectedValueGet();
    SymmetricMatrix pose_cov = posterior->CovarianceGet();

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "/map";

    pose_msg.pose.position.x = pose(1);
    pose_msg.pose.position.y = pose(2);
    pose_msg.pose.position.z = pose(3);

    pose_pub.publish(pose_msg);*/
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "BayesianFilterNode");
    ros::NodeHandle nh;
    BayesianFilterNode pfNode(nh);
    ros::spin();
    return 0;
}
