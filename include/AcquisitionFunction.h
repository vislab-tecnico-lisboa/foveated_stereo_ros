#ifndef ACQUISITIONFUNCTION_H
#define ACQUISITIONFUNCTION_H
#include <vector>
#include <limits>
class AcquisitionFunction
{
public:
    AcquisitionFunction();

    virtual double getValue(const double & mean_, const double & standard_deviation_) = 0;
    int getArgMax(const std::vector<double> & means_, const std::vector<double> & standard_deviation_);
    std::vector<double> getValues(const std::vector<double> & means_, const std::vector<double> & standard_deviation_);

};

#endif // ACQUISITIONFUNCTION_H
