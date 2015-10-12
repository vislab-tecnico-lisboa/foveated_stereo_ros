#ifndef PERIPHERALSTEREOROSNODELET_H
#define PERIPHERALSTEREOROSNODELET_H
#include <nodelet/nodelet.h>
#include "StereoRos.h"
#include "PeripheralStereo.h"

namespace foveated_stereo_ros
{
class PeripheralStereoRosNodelet: public nodelet::Nodelet
{

public:
    PeripheralStereoRosNodelet(){}
    ~PeripheralStereoRosNodelet(){}
    virtual void onInit();
    boost::shared_ptr<StereoRos<PeripheralStereo> > inst_;

};

}


#endif // PERIPHERALSTEREOROSNODELET_H
