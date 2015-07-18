#ifndef CONVENTIONALSTEREOROSNODELET_H
#define CONVENTIONALSTEREOROSNODELET_H
#include <nodelet/nodelet.h>
#include "ConventionalStereoRos.h"

namespace foveated_stereo_ros
{
class ConventionalStereoRosNodelet: public nodelet::Nodelet
{

public:
    ConventionalStereoRosNodelet(){}
    ~ConventionalStereoRosNodelet(){}
    virtual void onInit();
    boost::shared_ptr<ConventionalStereoRos> inst_;

};

}


#endif // CONVENTIONALSTEREOROSNODELET_H
