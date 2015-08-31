#include "DecisionMaking.h"

DecisionMaking::DecisionMaking(const boost::shared_ptr<SphericalShell<std::vector<boost::shared_ptr<MemoryPatch> > > > & ego_sphere_, const double & closest_bound_, const double & sigma_scale_upper_bound_) :
    ego_sphere(ego_sphere_),
    closest_bound(1.0/closest_bound_),
    sigma_scale_upper_bound(sigma_scale_upper_bound_)
{}

Eigen::Vector3d DecisionMaking::getFixationPoint()
{
    Eigen::Vector3d fixation_point(Eigen::Vector3d::Constant(std::numeric_limits<double>::max()));
    double closest_point_inverse_dist=0.0;
    for(std::vector<boost::shared_ptr<MemoryPatch> >::iterator structure_it = ego_sphere->structure.begin(); structure_it != ego_sphere->structure.end(); ++structure_it)
    {
        // Linearize 1/sqrt(x*x + y*y + z*z)
        double mean=1.0/(*structure_it)->sensory_data.position.mean.norm();
        double aux=pow((*structure_it)->sensory_data.position.mean.squaredNorm(),-1.5);
        Eigen::Vector3d jacobian(-(*structure_it)->sensory_data.position.mean.x()*aux,
                                 -(*structure_it)->sensory_data.position.mean.y()*aux,
                                 -(*structure_it)->sensory_data.position.mean.z()*aux);
        double sigma=sqrt( jacobian.transpose()*(*structure_it)->sensory_data.position.information.inverse()*jacobian );

        if(isnan(sigma))
            continue;

        double confidence_dist=mean+sigma_scale_upper_bound*sigma;

        //std::cout << "sigma:"<< sigma << std::endl;
        if(confidence_dist>closest_point_inverse_dist&&confidence_dist<closest_bound)
        {
            closest_point_inverse_dist=confidence_dist;
            fixation_point=(*structure_it)->sensory_data.position.mean;

        }
    }
    std::cout << "fixation_point:"<< fixation_point.transpose() << std::endl;

    return fixation_point;
}

