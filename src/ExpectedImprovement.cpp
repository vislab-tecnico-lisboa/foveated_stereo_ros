#include "ExpectedImprovement.h"

ExpectedImprovement::ExpectedImprovement()
{

}

double ExpectedImprovement::getValue(const double & mean_, const double & standard_deviation_, const double & best_value_)
{
    // standard normal distribution object:
    boost::math::normal normal_dist(mean_-best_value_,standard_deviation_);
    // print survival function for x=2.0:
    double value_=(mean_-best_value_)/standard_deviation_;
    double cumulative= boost::math::cdf(normal_dist, value_);
    std::cout <<"prob:"<<boost::math::pdf(normal_dist,value_)<< std::endl;
    std::cout <<"cumulative:"<<cumulative<< std::endl;

    double expected_improvement= (mean_ - best_value_)*boost::math::pdf(normal_dist,value_) + standard_deviation_*cumulative;
    return expected_improvement;
}

int main(int argc, char** argv)
{

    ExpectedImprovement expected_improvement;
    double mean=2.0;
    double std_dev=1.0;
    double best_value=2.5;
    expected_improvement.getValue(mean,std_dev,best_value);
    return 0;
}
