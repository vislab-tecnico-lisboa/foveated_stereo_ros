#include "Gaze.h"
Gaze::Gaze(std::string name) :
    as_(nh_, name, boost::bind(&Gaze::executeCB, this, _1), false),
    private_node_handle("~"),
    action_name_(name),
    robot_model_loader("robot_description"),
    robot_model(robot_model_loader.getModel()),
    joint_model_group(robot_model->getJointModelGroup("head")),
    tf_listener(new tf::TransformListener(ros::Duration(2.0))),
    group(new moveit::planning_interface::MoveGroup("head"))
{
    joint_names=group->getActiveJoints();
    joint_values.resize(joint_names.size());
    std::cout << joint_names[0] << " " << joint_names[1] << " " << joint_names[2] << std::endl;

    state_monitor=boost::shared_ptr<planning_scene_monitor::CurrentStateMonitor>(new planning_scene_monitor::CurrentStateMonitor(robot_model,tf_listener));
    state_monitor->startStateMonitor("/vizzy/joint_states");


    std::string left_camera_frame;
    std::string right_camera_frame;
    private_node_handle.param<std::string>("left_camera_frame", left_camera_frame, "left_camera_frame");
    private_node_handle.param<std::string>("right_camera_frame", right_camera_frame, "right_camera_frame");
    tf::StampedTransform transform;

    try
    {
        tf_listener->waitForTransform(right_camera_frame, left_camera_frame, ros::Time(0), ros::Duration(10.0) );
        tf_listener->lookupTransform(right_camera_frame, left_camera_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        exit(-1);
    }

    tf::Vector3 origin=transform.getOrigin();

    half_base_line=(double)origin.length()/2.0; // meters

    ROS_INFO("Model frame: %s", robot_model->getModelFrame().c_str());
    //kinematic_state->setToDefaultValues();

    neck_pan_publisher = nh_.advertise<std_msgs::Float64>("/vizzy/neck_pan_position_controller/command", 10);
    neck_tilt_publisher = nh_.advertise<std_msgs::Float64>("/vizzy/neck_tilt_position_controller/command", 10);
    eyes_tilt_publisher = nh_.advertise<std_msgs::Float64>("/vizzy/eyes_tilt_position_controller/command", 10);
    eyes_vergence_publisher = nh_.advertise<std_msgs::Float64>("/vizzy/vergence_position_controller/command", 10);

    as_.start();
}

void Gaze::move(const Eigen::Vector3d & fixation_point)
{
    std_msgs::Float64 neck_pan_angle;
    std_msgs::Float64 neck_tilt_angle;
    std_msgs::Float64 vergence_angle;

    Eigen::Vector3d fixation_point_normalized=fixation_point.normalized();
    if(fixation_point_normalized.x()!=fixation_point_normalized.x())
    {
        neck_pan_angle.data=0.0;
        neck_tilt_angle.data=0.0;
        vergence_angle.data=0.0;
    }
    else
    {
        neck_pan_angle.data=asin(fixation_point_normalized.x());
        neck_tilt_angle.data=asin(fixation_point_normalized.y());
        vergence_angle.data=M_PI/2.0-atan2(fixation_point.z(),half_base_line);

        /*std::cout << "Z:" << fixation_point.z() <<std::endl;
        std::cout << "atan2(fixation_point.z(),half_base_line):" << atan2(fixation_point.z(),half_base_line) << std::endl;
        std::cout << "half_base_line" << half_base_line << std::endl;
        std::cout << "fixation_point:" << fixation_point << std::endl;
        std::cout << "fixation_point normalized:" << fixation_point.normalized() << std::endl;
        std::cout << "fixation_point normalized y:" << fixation_point.normalized().y() << std::endl;
        std::cout << "asin fixation_point normalized y:" << asin(fixation_point.normalized().y()) << std::endl;

        std::cout << "neck_pan_angle.data"<<neck_pan_angle.data << std::endl;
        std::cout << "neck_tilt_angle.data"<<neck_tilt_angle.data << std::endl;
        std::cout << "vergence_angle.data"<<vergence_angle.data << std::endl;*/

    }

    joint_values[0] = neck_pan_angle.data;
    joint_values[1] = 0;
    joint_values[2] = neck_tilt_angle.data;

    group->setJointValueTarget(joint_values);
    std::cout << "going to move..." << std::endl;
    group->move();
    std::cout << "done" << std::endl;

    // INSTEAD SHOULD CHECK LIMITS AND MOVE MANUALLY
    //robot_model->getActiveJointModelsBounds()
    //neck_pan_publisher.publish(neck_pan_angle);
    //neck_tilt_publisher.publish(neck_tilt_angle);
    //eyes_tilt_publisher.publish(neck_tilt_angle);
    eyes_vergence_publisher.publish(vergence_angle);

    state_monitor->getCurrentState()->copyJointGroupPositions(joint_model_group, joint_values);

    double error=10000;
    double epsilon=0.1;
    /*do
    {
        error=0.0;

        state_monitor->getCurrentState()->copyJointGroupPositions(joint_model_group, joint_values);

        error=  (joint_values[0]-neck_pan_angle.data)*(joint_values[0]-neck_pan_angle.data)+
                (joint_values[1]-neck_tilt_angle.data)*(joint_values[1]-neck_tilt_angle.data)+
                (joint_values[2]-vergence_angle.data)*(joint_values[2]-vergence_angle.data);
        //std::cout << error << std::endl;
        //std::cout << "joint values["  << "]:" << joint_values[0] << " " << joint_values[1] << " "<<joint_values[2] << std::endl;

    }
    while(error>epsilon);*/
}

void Gaze::executeCB(const foveated_stereo_ros::GazeGoalConstPtr &goal)
{
    // helper variables
    ros::Rate r(1);
    bool success = true;

    Eigen::Vector3d fixation_point_;
    fixation_point_(0)=goal->fixation_point.point.x;
    fixation_point_(1)=goal->fixation_point.point.y;
    fixation_point_(2)=goal->fixation_point.point.z;

    move(fixation_point_);

    // publish info to the console for the user
    /*ROS_INFO("%s: Executing, creating Gaze sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
        // check that preempt has not been requested by the client
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            break;
        }
        feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
        // publish the feedback
        as_.publishFeedback(feedback_);
        // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        r.sleep();
    }*/
    success=true;
    if(success)
    {
        result_.state_reached=true;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }
    else
    {
        result_.state_reached=false;
        as_.setAborted(result_);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gaze");
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    Gaze gaze(ros::this_node::getName());

    ros::Rate r(1.0);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    moveit::core::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("head");

    ros::spin();

    return 1;
}
