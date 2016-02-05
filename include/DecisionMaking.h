#ifndef DECISIONMAKING_H
#define DECISIONMAKING_H
#include "EgoSphere.h"
#include "UpperConfidenceBound.h"
class DecisionMaking
{
public:
    double sigma_scale_upper_bound;
    boost::shared_ptr<SphericalShell<std::vector<boost::shared_ptr<MemoryPatch> > > > ego_sphere;

    DecisionMaking(const boost::shared_ptr<SphericalShell<std::vector<boost::shared_ptr<MemoryPatch> > > > & ego_sphere_,
                   const boost::shared_ptr<UpperConfidenceBound> acquisition_function_); // HAS DIRECT ACCESS TO EGO SPHERE

    int getFixationPoint(const double & sensory_filtering_radius);
    boost::shared_ptr<UpperConfidenceBound> acquisition_function;
};

#endif // DECISIONMAKING_H
