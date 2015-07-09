#include "StereoRosNodelet.h"

namespace foveated_stereo_ros
{
    void StereoRosNodelet::onInit()
    {
        NODELET_INFO("Initializing nodelet");
        inst_.reset(new StereoRosNode(getNodeHandle(), getPrivateNodeHandle()));
    }
}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(foveated_stereo_ros::FoveatedStereoRosNodelet,nodelet::Nodelet)
