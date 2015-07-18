#include "ConventionalStereoRosNodelet.h"

namespace foveated_stereo_ros
{
    void ConventionalStereoRosNodelet::onInit()
    {
        NODELET_INFO("Initializing nodelet");
        inst_.reset(new ConventionalStereoRos(getNodeHandle(), getPrivateNodeHandle()));
    }
}

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(foveated_stereo_ros::ConventionalStereoRosNodelet,nodelet::Nodelet)
