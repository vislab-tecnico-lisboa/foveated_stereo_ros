#include "Gaze.h"
Gaze::Gaze(const ros::NodeHandle & node_handle_) :
    node_handle(node_handle_),
    robot_model_loader("robot_description"),
    kinematic_model(robot_model_loader.getModel()),
    kinematic_state(new robot_state::RobotState(kinematic_model)),
    joint_model_group(kinematic_model->getJointModelGroup("head"))

{
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    //kinematic_state->setToDefaultValues();

    neck_pan_publisher = node_handle.advertise<std_msgs::Float64>("/vizzy/neck_pan_position_controller/command", 10);
    neck_tilt_publisher = node_handle.advertise<std_msgs::Float64>("/vizzy/neck_tilt_position_controller/command", 10);
    eyes_tilt_publisher = node_handle.advertise<std_msgs::Float64>("/vizzy/eyes_tilt_position_controller/command", 10);
}

void Gaze::move(const Eigen::Affine3d &end_effector_state)
{
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

    if(found_ik)
    {
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for(std::size_t i=0; i < joint_names.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
        std_msgs::Float64 j_value;
        j_value.data=joint_values[0];
        neck_pan_publisher.publish(j_value);
        j_value.data=joint_values[1];
        neck_tilt_publisher.publish(j_value);
    }
    else
    {
        ROS_INFO("Did not find IK solution");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    Gaze gaze(node_handle);

    ros::Rate r(1.0);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    moveit::core::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("head");

    while(ros::ok())
    {

        /* Compute FK for a set of random joint values*/
        kinematic_state->setToRandomPositions(joint_model_group);
        const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("eyes_center_vision_link");
        gaze.move(end_effector_state);
        r.sleep();
    }

    return 1;
}
