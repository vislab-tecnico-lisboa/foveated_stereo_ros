#ifndef BAYESIANFILTERNODE_H
#define BAYESIANFILTERNODE_H
//#include <filter/extendedkalmanfilter.h>
#include <filter/particlefilter.h>
#include "bayesian_filtering/customparticlefilter.h"

#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>

#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include "bayesian_filtering/nonlinearanalyticconditionalgaussian3D.h"//added
#include "bayesian_filtering/nonlinearsystempdf.h"
#include "bayesian_filtering/nonlinearmeasurementpdf.h"

#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <foveated_stereo_ros/StereoData.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>

#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>

#include <tf/transform_listener.h>
#include <pose_cov_ops/pose_cov_ops.h>
// Sizes
#define STATE_SIZE 3 //state: x,y,z
#define MEAS_SIZE 3  // measurment: 3D points

class BayesianFilterNode
{
    typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
    typedef boost::shared_ptr<foveated_stereo_ros::StereoData const> StereoConstPtr;

    ros::NodeHandle nh_, private_node_handle_;
    ros::Subscriber odom_sub;
    ros::Subscriber stereo_sub;
    ros::Publisher pose_pub;
    ros::Publisher particle_pub;
    boost::shared_ptr<tf::TransformListener> listener;

    boost::shared_ptr<CustomParticleFilter> filter;

    boost::shared_ptr<BFL::NonlinearSystemPdf> sys_pdf;
    boost::shared_ptr<BFL::SystemModel<BFL::ColumnVector> > sys_model;
    boost::shared_ptr<BFL::LinearAnalyticConditionalGaussian> meas_pdf;
    boost::shared_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> meas_model;
    boost::shared_ptr<BFL::MCPdf<BFL::ColumnVector> > prior_discr;

    double dt;
    OdomConstPtr last_odom_msg;

    bool odom_initialized;
public:
    BayesianFilterNode(const ros::NodeHandle &nh_);

    ~BayesianFilterNode();
    void createParticleFilter(const double & prior_mean,
                              const double & prior_std_dev,
                              const int & particles_number);
    void stereoCallback(const StereoConstPtr & msg);
    void odomCallback(const OdomConstPtr & msg);

    void PublishParticles();
    void PublishPose();

    void decomposeTransform(const tf::Pose& transform, BFL::Matrix& transf_matrix);
};

#endif // BAYESIANFILTERNODE_H
