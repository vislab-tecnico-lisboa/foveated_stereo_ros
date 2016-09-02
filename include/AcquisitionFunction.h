#ifndef ACQUISITIONFUNCTION_H
#define ACQUISITIONFUNCTION_H
#include <vector>
#include <limits>
#include <math.h>
class AcquisitionFunction
{
public:
    AcquisitionFunction();

    //virtual double getValue(const double & mean_, const double & standard_deviation_) = 0;
    virtual int getArgMax(const std::vector<double> & means_, const std::vector<double> & standard_deviation_) = 0;
    //std::vector<double> getValues(const std::vector<double> & means_, const std::vector<double> & standard_deviation_);
};

#endif // ACQUISITIONFUNCTION_H
