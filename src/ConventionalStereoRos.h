#ifndef CONVENTIONALSTEREOROS_H
#define CONVENTIONALSTEREOROS_H

#include "StereoRos.h"
#include "ConventionalStereo.h"

class ConventionalStereoRos : StereoRos
{
private:
    void cameraInfoCallback(const sensor_msgs::CameraInfoPtr & left_camera_info);
public:

    boost::shared_ptr<ConventionalStereo> stereo;

    ~ConventionalStereoRos();

    ConventionalStereoRos(ros::NodeHandle & nh_,
                  ros::NodeHandle & private_node_handle_
                  );

    void callback(const ImageConstPtr& left_image,
                  const ImageConstPtr& right_image);
};

#endif // STEREOROS_H
