#include "nodelets/PeripheralStereoRosNodelet.h"

namespace foveated_stereo_ros
{
    void PeripheralStereoRosNodelet::onInit()
    {
        NODELET_INFO("Initializing nodelet");
        inst_.reset(new StereoRos<PeripheralStereo>(getNodeHandle(), getPrivateNodeHandle()));
    }
}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(foveated_stereo_ros::PeripheralStereoRosNodelet,nodelet::Nodelet)
