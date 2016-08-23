#ifndef EXPECTEDIMPROVEMENT_H
#define EXPECTEDIMPROVEMENT_H
#include "AcquisitionFunction.h"
#include <boost/math/distributions.hpp>
class ExpectedImprovement
{
public:
    ExpectedImprovement();
    double getValue(const double & mean_, const double & standard_deviation_, const double & best_value_);

};

#endif // EXPECTEDIMPROVEMENT_H
