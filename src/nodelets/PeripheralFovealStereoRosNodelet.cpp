#include "nodelets/PeripheralFovealStereoRosNodelet.h"

namespace foveated_stereo_ros
{
    void PeripheralFovealStereoRosNodelet::onInit()
    {
        NODELET_INFO("Initializing nodelet");
        inst_.reset(new StereoRos<PeripheralFovealStereo>(getNodeHandle(), getPrivateNodeHandle()));
    }
}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(foveated_stereo_ros::PeripheralFovealStereoRosNodelet,nodelet::Nodelet)
