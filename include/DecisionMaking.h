#ifndef DECISIONMAKING_H
#define DECISIONMAKING_H
#include "EgoSphere.h"
class DecisionMaking
{
public:
    double sigma_scale_upper_bound;
    boost::shared_ptr<SphericalShell<std::vector<boost::shared_ptr<MemoryPatch> > > > ego_sphere;

    DecisionMaking(const boost::shared_ptr<SphericalShell<std::vector<boost::shared_ptr<MemoryPatch> > > > & ego_sphere_, const double & sigma_scale_upper_bound_); // HAS DIRECT ACCESS TO EGO SPHERE

    int getFixationPoint();

};

#endif // DECISIONMAKING_H
