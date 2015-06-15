#include "EgoSphereManagerRosNodelet.h"

namespace foveated_stereo_ros
{
    void EgoSphereManagerRosNodelet::onInit()
    {
        NODELET_INFO("Initializing nodelet");
        inst_.reset(new EgoSphereManagerRos(getNodeHandle(), getPrivateNodeHandle()));
    }
}
// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(foveated_stereo_ros::EgoSphereManagerRosNodelet,nodelet::Nodelet)

