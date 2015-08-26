#ifndef STEREOCALIBRATIONROS_H
#define STEREOCALIBRATIONROS_H

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class StereoCalibrationRos
{
    ros::NodeHandle nh;
    ros::NodeHandle private_node_handle;
public:
    StereoCalibrationRos();
};

#endif // STEREOCALIBRATIONROS_H
