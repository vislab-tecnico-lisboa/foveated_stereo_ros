#ifndef FOVEATEDSTEREOROS_H
#define FOVEATEDSTEREOROS_H

#include "StereoRos.h"
#include "FovealStereo.h"

class FoveatedStereoRos : StereoRos
{
private:
    void cameraInfoCallback(const sensor_msgs::CameraInfoPtr & left_camera_info);
public:

    boost::shared_ptr<FovealStereo> stereo;

    ~FoveatedStereoRos();

    FoveatedStereoRos(ros::NodeHandle & nh_, ros::NodeHandle & private_node_handle_);

    void callback(const ImageConstPtr& left_image, const ImageConstPtr& right_image);
};

#endif // FOVEATEDSTEREOROS_H
