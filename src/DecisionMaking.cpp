#include "DecisionMaking.h"
#include <ros/rate.h>
DecisionMaking::DecisionMaking(const boost::shared_ptr<SphericalShell<std::vector<boost::shared_ptr<MemoryPatch> > > > & ego_sphere_,
                               const boost::shared_ptr<AcquisitionFunction> acquisition_function_) :
    ego_sphere(ego_sphere_),
    acquisition_function(acquisition_function_)
{}

int DecisionMaking::getFixationPoint(const double & sensory_filtering_radius)
{
    std::vector<double> means_;
    means_.resize(ego_sphere->structure.size());
    std::fill(means_.begin(),means_.end(),-std::numeric_limits<double>::max());

    std::vector<double> sigmas_;
    sigmas_.resize(ego_sphere->structure.size());
    std::fill(sigmas_.begin(),sigmas_.end(),0.0);

    // apply non-linear transformation
    for(int i=0; i< ego_sphere->structure.size(); ++i)
    {
        // Linearize 1/sqrt(x*x + y*y + z*z)
        double mean=ego_sphere->structure[i]->sensory_data.position.mean.norm();
        if(isnan(mean))
        {
            ROS_FATAL("ESTA MAL!!!!");
            continue;
        }
        if(mean<sensory_filtering_radius||ego_sphere->structure[i]->sensory_data.position.mean(2)<0.0)
        {
            continue;
        }
        Eigen::Vector3d jacobian(ego_sphere->structure[i]->sensory_data.position.mean.x()/mean,
                                 ego_sphere->structure[i]->sensory_data.position.mean.y()/mean,
                                 ego_sphere->structure[i]->sensory_data.position.mean.z()/mean);

        double sigma=sqrt( jacobian.transpose()*ego_sphere->structure[i]->sensory_data.position.information.inverse()*jacobian );

        if(isnan(sigma))
        {
            continue;
        }

        means_[i]=-mean;
        sigmas_[i]=sigma;
    }

    return acquisition_function->getArgMax(means_,sigmas_);
}

