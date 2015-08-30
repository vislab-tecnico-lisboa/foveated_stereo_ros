#include "DecisionMaking.h"

DecisionMaking::DecisionMaking(const boost::shared_ptr<SphericalShell<std::vector<boost::shared_ptr<MemoryPatch> > > > & ego_sphere_, const double & closest_bound_, const double & sigma_scale_upper_bound_) :
    ego_sphere(ego_sphere_),
    closest_bound(closest_bound_),
    sigma_scale_upper_bound(sigma_scale_upper_bound_)
{}

Eigen::Vector3d DecisionMaking::getFixationPoint()
{
    Eigen::Vector3d fixation_point(Eigen::Vector3d::Constant(std::numeric_limits<double>::max()));
    double closest_point_dist=std::numeric_limits<double>::max();
    for(std::vector<boost::shared_ptr<MemoryPatch> >::iterator structure_it = ego_sphere->structure.begin(); structure_it != ego_sphere->structure.end(); ++structure_it)
    {
        // Linearize 1/sqrt(x*x + y*y + z*z)
        double mean_distance=(*structure_it)->sensory_data.position.mean.norm();
        Eigen::Vector3d jacobian((*structure_it)->sensory_data.position.mean.x()/mean_distance,
                                 (*structure_it)->sensory_data.position.mean.y()/mean_distance,
                                 (*structure_it)->sensory_data.position.mean.z()/mean_distance);
        double sigma=sqrt( jacobian.transpose()*(*structure_it)->sensory_data.position.information.inverse()*jacobian );
        double confidence_dist=mean_distance-sigma_scale_upper_bound*sigma;

        if(confidence_dist<closest_point_dist&&confidence_dist>closest_bound)
        {
            closest_point_dist=confidence_dist;
            fixation_point=(*structure_it)->sensory_data.position.mean;
        }
    }

    return fixation_point;
}

