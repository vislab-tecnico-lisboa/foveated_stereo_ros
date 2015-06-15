#ifndef EGOSPHEREMANAGERROSNODELET_H
#define EGOSPHEREMANAGERROSNODELET_H
#include <nodelet/nodelet.h>
#include "EgoSphereManagerRos.h"

namespace foveated_stereo_ros
{
class EgoSphereManagerRosNodelet: public nodelet::Nodelet
{
public:
    EgoSphereManagerRosNodelet(){}
    ~EgoSphereManagerRosNodelet(){}
    virtual void onInit();
    boost::shared_ptr<EgoSphereManagerRos> inst_;
};
}
#endif // EGOSPHEREMANAGERROSNODELET_H
