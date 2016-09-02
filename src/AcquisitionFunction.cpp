#include "AcquisitionFunction.h"

AcquisitionFunction::AcquisitionFunction()
{
}

/*
std::vector<double> AcquisitionFunction::getValues(const std::vector<double> & means_, const std::vector<double> & standard_deviation_)
{
    double epsilon=1.0;
    std::vector<double> values_;
    values_.resize(means_.size());
    double eta=0.0;
    double min=std::numeric_limits<double>::max();
    for(int i=0; i<means_.size();++i)
    {
        values_[i]=getValue(means_[i],standard_deviation_[i]);
        if(values_[i]<min)
        {
            min=values_[i];
        }
    }
    min=fabs(min+epsilon);
    for(int i=0; i<means_.size();++i)
    {
        values_[i]+=min;
        eta+=values_[i];
    }

    for(int i=0; i<means_.size();++i)
    {
        values_[i]/=eta;
    }
    return values_;
}
*/

