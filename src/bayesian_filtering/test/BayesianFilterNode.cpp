#include "BayesianFilterNode.h"
using namespace MatrixWrapper;
using namespace BFL;
using namespace std;

BayesianFilterNode::BayesianFilterNode(const ros::NodeHandle &nh_) :
    nh(nh_),
    odom_initialized(false)
{
    odom_sub = nh.subscribe("odom", 1, &BayesianFilterNode::odomCallback, this);
    ranges_sub = nh.subscribe("ranges", 1, &BayesianFilterNode::stereoCallback, this);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/pose_pf",1);
    particle_pub = nh.advertise<geometry_msgs::PoseArray>("/particle_cloud",1);
    dt = 0.0;

    CreateParticleFilter();

}

BayesianFilterNode::~BayesianFilterNode()
{}

void BayesianFilterNode::CreateParticleFilter()
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
    sys_noise_Cov(1,1) = 1.0;
    sys_noise_Cov(1,2) = 0.0;
    sys_noise_Cov(1,3) = 0.0;
    sys_noise_Cov(2,1) = 0.0;
    sys_noise_Cov(2,2) = 1.0;
    sys_noise_Cov(2,3) = 0.0;
    sys_noise_Cov(3,1) = 0.0;
    sys_noise_Cov(3,2) = 0.0;
    sys_noise_Cov(3,3) = 1.0;

    Gaussian system_Uncertainty(sys_noise_Mu, sys_noise_Cov);

    // create the nonlinear system model
    sys_pdf = boost::shared_ptr<BFL::NonlinearSystemPdf> (new  BFL::NonlinearSystemPdf(system_Uncertainty));
    sys_model = boost::shared_ptr<BFL::SystemModel<ColumnVector> > (new BFL::SystemModel<ColumnVector> (sys_pdf.get()));

    /*********************************
     * NonLinear Measurement model   *
     ********************************/

    // Construct the measurement noise (a scalar in this case)
    ColumnVector meas_noise_Mu(MEAS_SIZE);
    meas_noise_Mu(1) = MU_MEAS_NOISE;
    meas_noise_Mu(2) = MU_MEAS_NOISE;
    meas_noise_Mu(3) = MU_MEAS_NOISE;
    SymmetricMatrix meas_noise_Cov(MEAS_SIZE);
    meas_noise_Cov(1,1) = 1.0;
    meas_noise_Cov(1,2) = 0.0;
    meas_noise_Cov(1,3) = 0.0;
    meas_noise_Cov(2,1) = 0.0;
    meas_noise_Cov(2,2) = 1.0;
    meas_noise_Cov(2,3) = 0.0;
    meas_noise_Cov(3,1) = 0.0;
    meas_noise_Cov(3,2) = 0.0;
    meas_noise_Cov(3,3) = 1.0;

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
    prior_Mu(1) = PRIOR_MU_X;
    prior_Mu(2) = PRIOR_MU_Y;
    prior_Mu(3) = PRIOR_MU_THETA;
    SymmetricMatrix prior_Cov(STATE_SIZE);
    prior_Cov(1,1) = PRIOR_COV_X;
    prior_Cov(1,2) = 0.0;
    prior_Cov(1,3) = 0.0;
    prior_Cov(2,1) = 0.0;
    prior_Cov(2,2) = PRIOR_COV_Y;
    prior_Cov(2,3) = 0.0;
    prior_Cov(3,1) = 0.0;
    prior_Cov(3,2) = 0.0;
    prior_Cov(3,3) = PRIOR_COV_THETA;
    Gaussian prior_cont(prior_Mu,prior_Cov);

    // Discrete prior for Particle filter (using the continuous Gaussian prior)
    vector<Sample<ColumnVector> > prior_samples(NUM_SAMPLES);
    MCPdf<ColumnVector> prior_discr(NUM_SAMPLES,STATE_SIZE);
    prior_cont.SampleFrom(prior_samples,NUM_SAMPLES,CHOLESKY,NULL);
    prior_discr.ListOfSamplesSet(prior_samples);

    /******************************
     * Construction of the Filter *
     ******************************/
    filter=boost::shared_ptr<BFL::BootstrapFilter<BFL::ColumnVector,BFL::ColumnVector> > (new BFL::BootstrapFilter<BFL::ColumnVector,BFL::ColumnVector>(&prior_discr, 0, NUM_SAMPLES/4.0));
}

void BayesianFilterNode::stereoCallback(const StereoConstPtr & msg)
{
    /*ColumnVector measurement(3);
    measurement(1) = msg.range_front/100;
    measurement(2) = msg.range_left/100;
    measurement(3) = msg.range_right/100;
    //ROS_INFO("Measurement: %f",measurement(1));
    if (LastNavDataMsg.state==3 || LastNavDataMsg.state==7 || LastNavDataMsg.state==4)
    {
        filter->Update(meas_model, measurement);
        PublishParticles();
        PublishPose();
    }*/
}

void BayesianFilterNode::odomCallback(const OdomConstPtr & msg)
{
    if(!odom_initialized)
    {
        odom_initialized=true;
        last_odom_msg=msg;
        return;
    }

    if(!last_odom_msg->header.stamp.isZero()) dt = (msg->header.stamp - last_odom_msg->header.stamp).toSec();
    last_odom_msg = msg;

    BFL::Matrix  transf_matrix(4,4);
    tf::Pose new_transform;
    tf::poseMsgToTF(msg->pose.pose, new_transform);
    tf::Pose old_transform;
    tf::poseMsgToTF(last_odom_msg->pose.pose, old_transform);
    new_transform=new_transform*old_transform.inverse();
    decomposeTransform(new_transform,transf_matrix);
    //std::cout << "transf_matrix:"<< transf_matrix << std::endl;

    ColumnVector input(16);
    for(unsigned int row=1; row<=4; ++row)
    {
        for(unsigned int col=1; col<=4; ++col)
        {
            unsigned int index=col+(row-1)*4;
            //std::cout << index << std::endl;
            input(index)=transf_matrix(row,col);
        }
    }

    filter->Update(sys_model.get(),input);
}

void BayesianFilterNode::PublishParticles()
{
    /*   geometry_msgs::PoseArray particles_msg;
    particles_msg.header.stamp = ros::Time::now();
    particles_msg.header.frame_id = "/map";

    vector<WeightedSample<ColumnVector> >::iterator sample_it;
    vector<WeightedSample<ColumnVector> > samples;

    samples = filter->getNewSamples();

    for(sample_it = samples.begin(); sample_it<samples.end(); sample_it++)
    {
        geometry_msgs::Pose pose;
        ColumnVector sample = (*sample_it).ValueGet();

        pose.position.x = sample(1);
        pose.position.y = sample(2);
        pose.orientation.z = sample(3);

        particles_msg.poses.insert(particles_msg.poses.begin(), pose);
    }
    particle_pub.publish(particles_msg);*/

}

void BayesianFilterNode::PublishPose()
{
    /*    Pdf<ColumnVector> * posterior = filter->PostGet();
    ColumnVector pose = posterior->ExpectedValueGet();
    SymmetricMatrix pose_cov = posterior->CovarianceGet();

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "/map";

    pose_msg.pose.position.x = pose(1);
    pose_msg.pose.position.y = pose(2);
    pose_msg.pose.orientation.z = pose(3);

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
