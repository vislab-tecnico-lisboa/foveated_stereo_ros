#include "DecisionMaking.h"
#include <ros/rate.h>
DecisionMaking::DecisionMaking(const boost::shared_ptr<SphericalShell<std::vector<boost::shared_ptr<MemoryPatch> > > > & ego_sphere_, const double & closest_bound_, const double & sigma_scale_upper_bound_) :
    ego_sphere(ego_sphere_),
    sigma_scale_upper_bound(sigma_scale_upper_bound_),
    closest_bound(closest_bound_)
{}

Eigen::Vector3d DecisionMaking::getFixationPoint()
{
    Eigen::Vector3d fixation_point(Eigen::Vector3d::Constant(std::numeric_limits<double>::max()));
    double closest_point_dist=100000.0;
    double confidence_dist;
    double best_sigma;
    double best_mean;
    // minimize distance
    for(std::vector<boost::shared_ptr<MemoryPatch> >::iterator structure_it = ego_sphere->structure.begin(); structure_it != ego_sphere->structure.end(); ++structure_it)
    {
        // Linearize 1/sqrt(x*x + y*y + z*z)
        double mean=(*structure_it)->sensory_data.position.mean.norm();

        Eigen::Vector3d jacobian((*structure_it)->sensory_data.position.mean.x()/mean,
                                 (*structure_it)->sensory_data.position.mean.y()/mean,
                                 (*structure_it)->sensory_data.position.mean.z()/mean);

        double sigma=sqrt( jacobian.transpose()*(*structure_it)->sensory_data.position.information.inverse()*jacobian );

        if(isnan(sigma))
        {
            //ROS_ERROR_STREAM("NAAAAOOOOOOOOOOOOOOOOOOOO volume:"<<(*structure_it)->sensory_data.position.getVolume()<< " mean:"<<  mean);
            continue;
        }

        //confidence_dist=mean;

        //confidence_dist=-sigma;
        confidence_dist=mean-sigma_scale_upper_bound*sigma;

        //std::cout << "sigma:"<< sigma << std::endl;
        if(confidence_dist<closest_point_dist)
        {
            closest_point_dist=confidence_dist;
            fixation_point=(*structure_it)->sensory_data.position.mean;
            best_sigma=sigma;
            best_mean=mean;
            ROS_ERROR_STREAM("SIGMA:"<< sigma);
            ROS_ERROR_STREAM("mean:"<< mean);
            ROS_ERROR_STREAM("confidence_dist:"<< confidence_dist);
        }
    }
    ROS_ERROR_STREAM("UPPER BOUND SCALE:"<<sigma_scale_upper_bound);
    ROS_ERROR_STREAM("BEST fixation_point:"<< fixation_point.transpose());
    ROS_ERROR_STREAM("BEST SIGMA:"<< best_sigma);
    ROS_ERROR_STREAM("BEST MEAN:"<< best_mean);
    ROS_ERROR_STREAM("BEST CONFIDENCE DIST:"<< closest_point_dist);

    return fixation_point;
}

