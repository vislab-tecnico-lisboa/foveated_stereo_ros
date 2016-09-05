#include "ExpectedImprovement.h"
int main(int argc, char** argv)
{

    ExpectedImprovement expected_improvement;
    double mean=-1.0;
    double std_dev=1.0;
    double best_value=-0.5;
    expected_improvement.getValue(mean,std_dev,best_value);
    return 0;
}

