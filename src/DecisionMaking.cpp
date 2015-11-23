#include "DecisionMaking.h"
#include <ros/rate.h>
DecisionMaking::DecisionMaking(const boost::shared_ptr<SphericalShell<std::vector<boost::shared_ptr<MemoryPatch> > > > & ego_sphere_, const double & sigma_scale_upper_bound_) :
    ego_sphere(ego_sphere_),
    sigma_scale_upper_bound(sigma_scale_upper_bound_)
{}

int DecisionMaking::getFixationPoint(const double & sensory_filtering_radius)
{
    Eigen::Vector3d fixation_point(Eigen::Vector3d::Constant(std::numeric_limits<double>::max()));
    double closest_point_confidence_dist=std::numeric_limits<double>::max();
    double confidence_dist;
    double best_sigma;
    double best_mean;
    int best_index;
    best_index=0;
    // minimize distance
    for(int i=0; i< ego_sphere->structure.size(); ++i)
    {
        // Linearize 1/sqrt(x*x + y*y + z*z)
        double mean=ego_sphere->structure[i]->sensory_data.position.mean.norm();
        if(isnan(mean))
        {
            ROS_FATAL("ESTA MAL!!!!");
            continue;
        }
        if(mean<sensory_filtering_radius)
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

        confidence_dist=mean-sigma_scale_upper_bound*sigma;

        //std::cout << "sigma:"<< sigma << std::endl;
        if(confidence_dist<closest_point_confidence_dist)
        {
            closest_point_confidence_dist=confidence_dist;
            fixation_point=ego_sphere->structure[i]->sensory_data.position.mean;
            best_sigma=sigma;
            best_mean=mean;
            best_index=i;
        }
    }

    return best_index;
}

