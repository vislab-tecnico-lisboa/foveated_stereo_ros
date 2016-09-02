#include "ExpectedImprovement.h"

ExpectedImprovement::ExpectedImprovement()
{}

int ExpectedImprovement::getArgMax(const std::vector<double> & means_, const std::vector<double> & standard_deviation_)
{
    double max_value=*std::max_element(means_.begin(),means_.end());

    int argmax=0;
    double max=-std::numeric_limits<double>::max();
    for(int i=0; i< means_.size(); ++i)
    {
        double value_=getValue(means_[i],standard_deviation_[i], max_value);
        if(value_>max)
        {
            max=value_;
            argmax=i;
        }
    }
    std::cout << "max: "<< max <<std::endl;
    return argmax;
}

double ExpectedImprovement::getValue(const double & mean_, const double & standard_deviation_, const double & best_value_)
{
    // standard normal distribution object:
    //std::cout << "standard_deviation_:"<< standard_deviation_ << std::endl;
    std::cout << "mean_:"<< mean_ << std::endl;

    if(standard_deviation_<0.000001)
        return 0;
    boost::math::normal normal_dist(mean_,standard_deviation_);
    // print survival function for x=2.0:
    //double value_=(mean_-best_value_);
    //std::cout << "value_:"<< value_<<std::endl;
    // truncated
    double cumulative=boost::math::cdf(normal_dist, 0)-boost::math::cdf(normal_dist, best_value_);
    double prob;
    if(mean_>0||best_value_>0)
    {
        prob=0;
    }
    else
    {
        std::cout << boost::math::pdf(normal_dist,best_value_) << std::endl;
        prob=boost::math::pdf(normal_dist,best_value_)/cumulative;
    }

    std::cout <<"prob:"<<prob<< std::endl;
    std::cout <<"cumulative:"<<cumulative<< std::endl;

    double expected_improvement= (mean_ - best_value_)*prob + standard_deviation_*cumulative;
    std::cout << "expected imp:"<<expected_improvement<<std::endl;
    return expected_improvement;
}


