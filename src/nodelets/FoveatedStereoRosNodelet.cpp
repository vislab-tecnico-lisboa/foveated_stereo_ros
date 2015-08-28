#include "nodelets/FoveatedStereoRosNodelet.h"

namespace foveated_stereo_ros
{
    void FoveatedStereoRosNodelet::onInit()
    {
        NODELET_INFO("Initializing nodelet");
        inst_.reset(new StereoRos<FovealStereo>(getNodeHandle(), getPrivateNodeHandle()));
    }
}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(foveated_stereo_ros::FoveatedStereoRosNodelet,nodelet::Nodelet)
