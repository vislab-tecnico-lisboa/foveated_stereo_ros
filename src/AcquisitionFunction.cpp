#include "AcquisitionFunction.h"

AcquisitionFunction::AcquisitionFunction()
{
}

int AcquisitionFunction::getArgMax(const std::vector<double> & means_, const std::vector<double> & standard_deviation_)
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

std::vector<double> AcquisitionFunction::getValues(const std::vector<double> & means_, const std::vector<double> & standard_deviation_)
{
    std::vector<double> values_;
    values_.resize(means_.size());
    for(int i=0; i<means_.size();++i)
    {
        values_[i]=getValue(means_[i],standard_deviation_[i]);
    }
    return values_;
}
