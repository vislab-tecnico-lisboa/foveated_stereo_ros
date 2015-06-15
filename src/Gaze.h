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


class Gaze
{
    ros::NodeHandle node_handle;
    ros::Publisher neck_pan_publisher;// = node_handle.advertise<std_msgs::Float64>("/vizzy/neck_pan_position_controller/command", 10);
    ros::Publisher neck_tilt_publisher;// = node_handle.advertise<std_msgs::Float64>("/vizzy/neck_tilt_position_controller/command", 10);
    ros::Publisher eyes_tilt_publisher;// = node_handle.advertise<std_msgs::Float64>("/vizzy/eyes_tilt_position_controller/command", 10);

    robot_model_loader::RobotModelLoader robot_model_loader;//("robot_description");
    moveit::core::RobotModelPtr kinematic_model;// = robot_model_loader.getModel();

    moveit::core::RobotStatePtr kinematic_state;//(new robot_state::RobotState(kinematic_model));
    const moveit::core::JointModelGroup* joint_model_group;//kinematic_model->getJointModelGroup("head");

    std::vector<double> joint_values;
    std::vector<std::string> joint_names;

public:
    Gaze(const ros::NodeHandle & node_handle_);
    void move(const Eigen::Affine3d &end_effector_state);
};

#endif // GAZE_H
