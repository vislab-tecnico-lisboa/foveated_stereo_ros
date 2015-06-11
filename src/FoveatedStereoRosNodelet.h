#ifndef FOVEATEDSTEREOROSNODELET_H
#define FOVEATEDSTEREOROSNODELET_H
#include <nodelet/nodelet.h>
#include "FoveatedStereoRos.h"

namespace foveated_stereo_ros{
class FoveatedStereoRosNodelet: public nodelet::Nodelet
{

public:
    FoveatedStereoRosNodelet(){}
    ~FoveatedStereoRosNodelet(){}
    virtual void onInit();
    boost::shared_ptr<FoveatedStereoNode> inst_;

};

}


#endif // FOVEATEDSTEREOROSNODELET_H
