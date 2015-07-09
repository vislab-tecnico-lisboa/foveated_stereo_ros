#ifndef STEREOROSNODELET_H
#define STEREOROSNODELET_H

#include <nodelet/nodelet.h>
#include "StereoRos.h"

namespace foveated_stereo_ros
{
class StereoRosNodelet: public nodelet::Nodelet
{

public:
    StereoRosNodelet(){}
    ~StereoRosNodelet(){}
    virtual void onInit();
    boost::shared_ptr<StereoRosNode> inst_;

};

}
#endif // STEREOROSNODELET_H
