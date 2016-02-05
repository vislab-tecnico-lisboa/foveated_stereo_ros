#include "UpperConfidenceBound.h"

UpperConfidenceBound::UpperConfidenceBound(const double & scale_) : scale(scale_)
{}

double UpperConfidenceBound::getValue(const double & mean_, const double & standard_deviation_)
{
    return mean_+scale*standard_deviation_;
}
