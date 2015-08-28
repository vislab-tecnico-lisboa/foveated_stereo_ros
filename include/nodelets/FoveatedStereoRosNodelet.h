#ifndef FOVEATEDSTEREOROSNODELET_H
#define FOVEATEDSTEREOROSNODELET_H
#include <nodelet/nodelet.h>
#include "StereoRos.h"
#include "FovealStereo.h"

namespace foveated_stereo_ros
{
class FoveatedStereoRosNodelet: public nodelet::Nodelet
{

public:
    FoveatedStereoRosNodelet(){}
    ~FoveatedStereoRosNodelet(){}
    virtual void onInit();
    boost::shared_ptr<StereoRos<FovealStereo> > inst_;

};

}


#endif // FOVEATEDSTEREOROSNODELET_H
