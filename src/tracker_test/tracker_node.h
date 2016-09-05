/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

#ifndef TRACKER_NODE_H
#define TRACKER_NODE_H

// ros stuff
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "tracker.h"
//#include <robot_pose_ekf/GetStatus.h>

// messages
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <foveated_stereo_ros/StereoData.h>
#include <boost/thread/mutex.hpp>

// log files
#include <fstream>

// pcl stuff
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>

namespace estimation
{

/** \mainpage
 *  \htmlinclude manifest.html
 *
 * <b>Package Summary</b>
 * This package provides two main classes:
 *  1) OdomEstimation performs all sensor fusion operations, and
 *  2) MappingNode provides a ROS wrapper around OdomEstimation
*/

typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
typedef boost::shared_ptr<foveated_stereo_ros::StereoData const> StereoConstPtr;
typedef boost::shared_ptr<geometry_msgs::Twist const> VelConstPtr;

class MappingNode
{
public:
    typedef pcl::PointCloud<pcl::PointXYZRGB> PCLPointCloud;
    /// constructor
    MappingNode();

    /// destructor
    virtual ~MappingNode();

private:
    /// the mail filter loop that will be called periodically
    void spin(const ros::TimerEvent& e);

    /// callback function for odo data
    void odomCallback(const OdomConstPtr& odom);

    /// callback function for odo data
    void stereoCallback(const StereoConstPtr& stereo_msg);

    /// get the status of the filter
    //bool getStatus(robot_pose_ekf::GetStatus::Request& req, robot_pose_ekf::GetStatus::Response& resp);

    ros::NodeHandle node_;
    ros::Timer timer_;
    ros::Publisher pose_pub_;
    ros::Subscriber odom_sub_, stereo_sub_;
    ros::ServiceServer state_srv_;

    // mapping filter
    Mapping my_filter_;

    // estimated robot pose message to send
    geometry_msgs::PoseWithCovarianceStamped  output_;

    // robot state
    tf::TransformListener    robot_state_;
    tf::TransformBroadcaster odom_broadcaster_;

    // vectors
    //tf::Transform odom_meas_, stereo_meas_;
    tf::StampedTransform camera_base_;
    ros::Time odom_time_, stereo_time_;
    ros::Time odom_stamp_, stereo_stamp_, filter_stamp_;
    ros::Time odom_init_stamp_,  stereo_init_stamp_;
    bool odom_active_, stereo_active_;
    bool odom_used_, stereo_used_;
    bool odom_initializing_, stereo_initializing_;
    double timeout_;
    MatrixWrapper::SymmetricMatrix odom_covariance_, stereo_covariance_;
    std::vector<MatrixWrapper::SymmetricMatrix> stereo_covariances_;
    bool debug_, self_diagnose_;
    std::string output_frame_, base_footprint_frame_, egocentric_frame_, tf_prefix_;
    ros::Time filter_time_old_;

    // log files for debugging
    std::ofstream odom_file_, stereo_file_, corr_file_, time_file_, extra_file_;

    // counters
    unsigned int odom_callback_counter_, stereo_callback_counter_, ekf_sent_counter_;

    tf::StampedTransform odom_meas_, odom_meas_old_, stereo_meas_, stereo_meas_old_;

    //MatrixWrapper::ColumnVector vel_desi_, filter_estimate_old_vec_;
    //tf::Transform filter_estimate_old_;


    // decompose Transform into x,y,z,Rx,Ry,Rz
    void decomposeTransform(const tf::StampedTransform& trans, double& x, double& y, double&z, double&Rx, double& Ry, double& Rz)
    {
        x = trans.getOrigin().x();
        y = trans.getOrigin().y();
        z = trans.getOrigin().z();
        trans.getBasis().getEulerYPR(Rz, Ry, Rx);
    }

    // decompose Transform into x,y,z,Rx,Ry,Rz
    void decomposeTransform(const tf::Transform& trans, double& x, double& y, double&z, double&Rx, double& Ry, double& Rz)
    {
        x = trans.getOrigin().x();
        y = trans.getOrigin().y();
        z = trans.getOrigin().z();
        trans.getBasis().getEulerYPR(Rz, Ry, Rx);
    }

    // correct for angle overflow
    void angleOverflowCorrect(double& a, double ref)
    {
        while ((a-ref) >  M_PI) a -= 2*M_PI;
        while ((a-ref) < -M_PI) a += 2*M_PI;
    }



}; // class

} // namespace

#endif // TRACKER_NODE_H
