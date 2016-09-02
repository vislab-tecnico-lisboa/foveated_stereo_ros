#ifndef EXPECTEDIMPROVEMENT_H
#define EXPECTEDIMPROVEMENT_H
#include "AcquisitionFunction.h"
#include <boost/math/distributions.hpp>
#include <algorithm>
class ExpectedImprovement : public AcquisitionFunction
{
public:
    ExpectedImprovement();
    double getValue(const double & mean_, const double & standard_deviation_, const double & best_value_);
    int getArgMax(const std::vector<double> & means_, const std::vector<double> & standard_deviation_);

};

#endif // EXPECTEDIMPROVEMENT_H
