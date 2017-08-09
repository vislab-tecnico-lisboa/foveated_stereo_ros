#include "covSampling.hpp"

#include <cmath>
#include <iostream>

using namespace Eigen;
using namespace std;

namespace CovSampler {

	CovSampling::CovSampling(Scalar _chi2, Scalar _dx)
	{
		setConf(_chi2);
		setDist(_dx);

		points.reserve(MAX_POINTS);
		prob.reserve(MAX_POINTS);
	}

//	inline void CovSampling::setConf(Scalar _chi2)
//	{
//		chi2 = _chi2;
//	}
//
//	inline void CovSampling::setDist(Scalar _dx)
//	{
//		dx = _dx;
//	}
//
//	inline std::vector<Vec> const &CovSampling::getSamples()
//	{
//		return points;
//	}
//
//	inline std::vector<Scalar> const &CovSampling::getProbs()
//	{
//		return prob;
//	}

	void CovSampling::sample(Mat const &cov, Vec const &mean)
	{
		// INIT:

		// SelfAdjointEigenSolver<Mat> decomp;
		// decomp.computeDirect(cov);
		// //decomp.compute(cov);
		Mat invSigma = cov.inverse();

		Scalar xMax = sqrt(cov(0, 0)*chi2);
		Scalar yMax = sqrt(cov(1, 1)*chi2);
		Scalar zMax = sqrt(cov(2, 2)*chi2);

		int nX = trunc(xMax / dx);
		int nY = trunc(yMax / dx);
		int nZ = trunc(zMax / dx);

cout << "nX = " << nX << ", nY = " << nY << ", nZ = " << nZ << endl;
		// SAMPLE:
		Vec x(0.0, 0.0, 0.0);
		Scalar d, p, aux, sum_p;

		points.push_back(x);
		prob.push_back(1.0);
		sum_p = 1.0;

		Scalar a = invSigma(2,2);

		// line x = 0, y = 0
		while(true)
		{
			x(2) += dx;
			d = x(2)*x(2)*a;
			if( d >= chi2 )
				break;
			p = exp(-0.5*d);
			points.push_back(x);
			prob.push_back(p);
			sum_p += p;
			points.push_back(-x);
			prob.push_back(p);
			sum_p += p;
		}

		// plane x = 0
		for( int j = 1; j <= nY; j++)
		{
			x(1) = dx*j;

			Scalar b = 2.0*x(1)*invSigma(1,2);
			Scalar c = x(1)*x(1)*invSigma(1,1) - chi2;

			if( (aux = b*b-4.0*a*c) < 0 )
				continue;

			x(2) = ceil((-b-sqrt(aux))/(2.0*a*dx))*dx;
			while(true)
			{
				d = x.dot(invSigma*x);
				if( d >= chi2 )
					break;
				p = exp(-0.5*d);
				points.push_back(x);
				prob.push_back(p);
				sum_p += p;
				points.push_back(-x);
				prob.push_back(p);
				sum_p += p;

				x(2) += dx;
			}
		}

		// general case
		for( int k = 1; k <= nX; k++)
		{
			x(0) = dx*k;
			for( int j = -nY; j <= nY; j++)
			{
				x(1) = dx*j;

				Scalar b = 2.0*(x(0)*invSigma(0,2) + x(1)*invSigma(1,2));
				Scalar c = x(0)*x(0)*invSigma(0,0) + x(1)*x(1)*invSigma(1,1) + 2.0*x(0)*x(1)*invSigma(0,1) - chi2;

				if( (aux = b*b-4.0*a*c) < 0 )
					continue;

				x(2) = ceil((-b-sqrt(aux))/(2.0*a*dx))*dx;
				while(true)
				{
					d = x.dot(invSigma*x);
					if( d >= chi2 )
						break;
					p = exp(-0.5*d);
					points.push_back(x);
					prob.push_back(p);
					sum_p += p;
					points.push_back(-x);
					prob.push_back(p);
					sum_p += p;

					x(2) += dx;
				}
			}
		}

		for(int j = 0; j < points.size(); j++)
		{
			points[j] += mean;
			prob[j] /= sum_p;
		}
	}

}
