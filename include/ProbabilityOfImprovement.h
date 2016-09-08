#ifndef PROBABILITYOFIMPROVEMENT_H
#define PROBABILITYOFIMPROVEMENT_H

#include "AcquisitionFunction.h"
#include <boost/math/distributions.hpp>
#include <algorithm>
class ProbabilityOfImprovement : public AcquisitionFunction
{
public:
    ProbabilityOfImprovement();
    double getValue(const double & mean_, const double & standard_deviation_, const double & best_value_);
    int getArgMax(const std::vector<double> & means_, const std::vector<double> & standard_deviation_);

};

#endif // ProbabilityOfImprovement_H
