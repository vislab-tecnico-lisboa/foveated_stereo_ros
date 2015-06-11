/*#include "Gaze.h"
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
Gaze::Gaze()
{
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gaze_node");

    // Start a service client
    ros::NodeHandle node_handle;

    ros::Rate r(10);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    // Using the :moveit_core:`RobotModel`, we can construct a
    // :planning_scene:`PlanningScene` that maintains the state of
    // the world (including the robot).
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));


    moveit::core::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("head");

    std::vector<std::string> joint_names=joint_model_group->getJointModelNames();

    while(ros::ok())
    {*/
        /* Compute FK for a set of random joint values*/
        //kinematic_state->setToRandomPositions(joint_model_group);
        //const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("eyes_center_vision_link");

        /* Get the joint values*/
        /*std::vector<double> joint_values;
        kinematic_state->printStatePositions();
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        const double * joint_state_=kinematic_state->getJointPositions("neck_pan_joint");

        std::cout << "test:" << *joint_state_ << std::endl;
        // current joint_values
        for(std::size_t i=0; i < joint_names.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }

        //bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);*/

        /*if(found_ik)
        {
            kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
            for(std::size_t i=0; i < joint_names.size(); ++i)
            {
                ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            }
        }
        else
        {
            ROS_INFO("Did not find IK solution");
        }*/
/*
        r.sleep();
    }
}*/



/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Sachin Chitta */
#include "Gaze.h"
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate r(10);


  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  moveit::core::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("head");

  std::vector<double> joint_values;
  kinematic_state->printStatePositions();
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("eyes_center_vision_link");

  std::vector<std::string> joint_names=joint_model_group->getJointModelNames();

  while(ros::ok())
  {
      /* Compute FK for a set of random joint values*/
      kinematic_state->setToRandomPositions(joint_model_group);
      const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("eyes_center_vision_link");

      /* Get the joint values*/
      std::vector<double> joint_values;
      kinematic_state->printStatePositions();
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      const double * joint_state_=kinematic_state->getJointPositions("neck_pan_joint");

      // current joint_values
      for(std::size_t i=0; i < joint_names.size(); ++i)
      {
          ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }

      bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

      if(found_ik)
      {
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          for(std::size_t i=0; i < joint_names.size(); ++i)
          {
              ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
      }
      else
      {
          ROS_INFO("Did not find IK solution");
      }

      r.sleep();
  }

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroup group("head");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  group.setPoseReferenceFrame("eyes_center_vision_link");
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.

  Eigen::Quaternion<double> test;
  test=Eigen::AngleAxis<double>(0.0, Eigen::Vector3d::UnitY());

  geometry_msgs::PoseStamped target_pose1;
  target_pose1.header.frame_id="eyes_center_vision_link";
  target_pose1.pose.orientation.w = test.w();
  target_pose1.pose.position.x = test.x();
  target_pose1.pose.position.y = test.y();
  target_pose1.pose.position.z = test.z();
  group.setPoseTarget(target_pose1);


  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(2.0);

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // Now that we have a plan we can visualize it in Rviz.  This is not
  // necessary because the group.plan() call we made above did this
  // automatically.  But explicitly publishing plans is useful in cases that we
  // want to visualize a previously created plan.
  if (1)
  {
    ROS_INFO("Visualizing plan 1 (again)");
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(2.0);
  }

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. Note that
  // the pose goal we had set earlier is still active
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.

  /* Uncomment below line when working with a real robot*/
  /* group.move() */

  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // First get the current set of joint values for the group.
  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

  // Now, let's modify one of the joints, plan to the new joint
  // space goal and visualize the plan.
  group_variable_values[0] = -1.0;
  group.setJointValueTarget(group_variable_values);
  success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);

  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.
  /*moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "r_wrist_roll_link";
  ocm.header.frame_id = "base_link";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  group.setPathConstraints(test_constraints);

  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already
  // satisfies the path constraints. So, we need to set the start
  // state to a new pose.
  robot_state::RobotState start_state(*group.getCurrentState());
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.55;
  start_pose2.position.y = -0.05;
  start_pose2.position.z = 0.8;
  const robot_state::JointModelGroup *joint_model_group =
                  start_state.getJointModelGroup(group.getName());
  start_state.setFromIK(joint_model_group, start_pose2);
  group.setStartState(start_state);

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  group.setPoseTarget(target_pose1);
  success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 3 (constraints) %s",success?"":"FAILED");
  sleep(10.0);

  // When done with the path constraint be sure to clear it.
  group.clearPathConstraints();

  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list.
  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose3 = start_pose2;
  target_pose3.position.x += 0.2;
  target_pose3.position.z += 0.2;
  waypoints.push_back(target_pose3);  // up and out

  target_pose3.position.y -= 0.2;
  waypoints.push_back(target_pose3);  // left

  target_pose3.position.z -= 0.2;
  target_pose3.position.y += 0.2;
  target_pose3.position.x -= 0.2;
  waypoints.push_back(target_pose3);  // down and right (back to start)

  // We want the cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively
  // disabling it.
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);

  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);
  sleep(15.0);*/

  ros::shutdown();
  return 0;
}
