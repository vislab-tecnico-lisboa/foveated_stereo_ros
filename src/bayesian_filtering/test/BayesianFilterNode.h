#ifndef BAYESIANFILTERNODE_H
#define BAYESIANFILTERNODE_H
//#include <filter/extendedkalmanfilter.h>
#include <filter/particlefilter.h>
#include <filter/bootstrapfilter.h>

#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>

#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include "bayesian_filtering/nonlinearanalyticconditionalgaussian3D.h"//added
#include "bayesian_filtering/nonlinearsystempdf.h"
#include "bayesian_filtering/nonlinearmeasurementpdf.h"

#include <iostream>
#include <fstream>

// Include file with properties
#include "mobile_robot_wall_cts.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <foveated_stereo_ros/StereoData.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>

class BayesianFilterNode
{
    typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
    typedef boost::shared_ptr<foveated_stereo_ros::StereoData const> StereoConstPtr;
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Subscriber ranges_sub;
    ros::Publisher pose_pub;
    ros::Publisher particle_pub;

    boost::shared_ptr<BFL::BootstrapFilter<BFL::ColumnVector,BFL::ColumnVector> > filter;

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
    void CreateParticleFilter();
    void stereoCallback(const StereoConstPtr & msg);
    void odomCallback(const OdomConstPtr & msg);

    void PublishParticles();
    void PublishPose();

    void decomposeTransform(const tf::Pose& transform, BFL::Matrix& transf_matrix)
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
};

#endif // BAYESIANFILTERNODE_H
