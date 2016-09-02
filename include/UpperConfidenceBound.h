#ifndef UPPERCONFIDENCEBOUND_H
#define UPPERCONFIDENCEBOUND_H
#include "AcquisitionFunction.h"
class UpperConfidenceBound : public AcquisitionFunction
{
    double scale;
public:
    UpperConfidenceBound(const double & scale_);
    double getValue(const double & mean_, const double & standard_deviation_);
    int getArgMax(const std::vector<double> & means_, const std::vector<double> & standard_deviation_);

};

#endif // UPPERCONFIDENCEBOUND_H
