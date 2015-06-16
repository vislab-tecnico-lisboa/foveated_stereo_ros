#ifndef GAZE_H
#define GAZE_H

#include <moveit_msgs/GetPositionIK.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <std_msgs/Float64.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <foveated_stereo_ros/GazeAction.h>
#include <tf/transform_listener.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
//#include <moveit/planning_scene_monitor/current_state_monitor.h>

class Gaze
{

protected:

    planning_scene_monitor::CurrentStateMonitorPtr state_monitor;
    boost::shared_ptr<tf::TransformListener> tf_listener;

    ros::NodeHandle nh_;
    ros::NodeHandle private_node_handle;

    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<foveated_stereo_ros::GazeAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    foveated_stereo_ros::GazeFeedback feedback_;
    foveated_stereo_ros::GazeResult result_;

    ros::Publisher neck_pan_publisher;
    ros::Publisher neck_tilt_publisher;
    ros::Publisher eyes_tilt_publisher;
    ros::Publisher eyes_vergence_publisher;

    robot_model_loader::RobotModelLoader robot_model_loader;//("robot_description");
    moveit::core::RobotModelPtr robot_model;// = robot_model_loader.getModel();
    const moveit::core::JointModelGroup* joint_model_group;//kinematic_model->getJointModelGroup("head");
    moveit::planning_interface::MoveGroup* group;

    std::vector<double> joint_values;
    std::vector<std::string> joint_names;

public:
    double half_base_line;
    Gaze(std::string name);
    void move(const Eigen::Vector3d &fixation_point);

    void executeCB(const foveated_stereo_ros::GazeGoalConstPtr &goal);
};

#endif // GAZE_H
