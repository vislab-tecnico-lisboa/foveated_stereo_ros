#ifndef DECISIONMAKING_H
#define DECISIONMAKING_H
#include "EgoSphere.h"
class DecisionMaking
{
public:
    double closest_bound;
    double sigma_scale_upper_bound;
    boost::shared_ptr<SphericalShell<std::vector<boost::shared_ptr<MemoryPatch> > > > ego_sphere;

    DecisionMaking(const boost::shared_ptr<SphericalShell<std::vector<boost::shared_ptr<MemoryPatch> > > > & ego_sphere_, const double & closest_bound_, const double & sigma_scale_upper_bound_); // HAS DIRECT ACCESS TO EGO SPHERE

    Eigen::Vector3d getFixationPoint();

};

#endif // DECISIONMAKING_H
