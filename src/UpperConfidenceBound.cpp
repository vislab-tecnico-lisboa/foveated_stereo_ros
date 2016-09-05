#include "UpperConfidenceBound.h"

UpperConfidenceBound::UpperConfidenceBound(const double & scale_) : scale(scale_)
{}

int UpperConfidenceBound::getArgMax(const std::vector<double> & means_, const std::vector<double> & standard_deviation_)
{
    int argmax=0;
    double max=-std::numeric_limits<double>::max();
    for(int i=0; i< means_.size(); ++i)
    {
        double value_=getValue(means_[i],standard_deviation_[i]);
        if(value_>max)
        {
            max=value_;
            argmax=i;
        }
    }
    return argmax;
}

double UpperConfidenceBound::getValue(const double & mean_, const double & standard_deviation_)
{
    return mean_+scale*standard_deviation_;
}
