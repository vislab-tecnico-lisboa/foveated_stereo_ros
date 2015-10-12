#ifndef PERIPHERALFOVEALSTEREOROSNODELET_H
#define PERIPHERALFOVEALSTEREOROSNODELET_H
#include <nodelet/nodelet.h>
#include "StereoRos.h"
#include "PeripheralFovealStereo.h"

namespace foveated_stereo_ros
{
class PeripheralFovealStereoRosNodelet: public nodelet::Nodelet
{

public:
    PeripheralFovealStereoRosNodelet(){}
    ~PeripheralFovealStereoRosNodelet(){}
    virtual void onInit();
    boost::shared_ptr<StereoRos<PeripheralFovealStereo> > inst_;
};
}

#endif // PERIPHERALFOVEALSTEREOROSNODELET_H
