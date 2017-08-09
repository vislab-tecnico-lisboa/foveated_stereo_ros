#include "covSampling.hpp"
#include <iostream>

using namespace Eigen;
using namespace std;
using namespace CovSampler;

int main()
{
	CovSampling covSampling(7.82,5.0);

	Mat T, covMatrix;
	T << 4, 1, 0, -2, 1, 2, 3, -3, 3;
	covMatrix = T*T.transpose();
	Vec m(0, 0, 0);

	covSampling.sample(covMatrix,m);

	for(int i = 0; i < covSampling.getProbs().size(); i++)
	{
		cout << covSampling.getSamples()[i].transpose() << " " <<
				covSampling.getProbs()[i] << endl;
	}

//	cout << covSampling.points.size() << endl;

	return 0;
}

