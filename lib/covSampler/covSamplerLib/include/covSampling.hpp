#ifndef COV_SAMPLER
#define COV_SAMPLER

#include <Eigen/Dense>
#include <vector>

#define MAX_POINTS 10000

namespace CovSampler {

	typedef double Scalar;
	typedef Eigen::Matrix<Scalar, 3, 1> Vec;
	typedef Eigen::Matrix<Scalar, 3, 3> Mat;

	class CovSampling
	{
	public:
		CovSampling(Scalar _chi2 = 7.82, Scalar _dx = 1.0);

		// chi2: 4.64 -> 20%, 6.25 -> 10%, 7.82 -> 5%, 11.34 -> 1%
		void setConf(Scalar _chi2) {
			chi2 = _chi2; }
		void setDist(Scalar _dx) {
			dx = _dx; }
		void sample(Mat const &cov, Vec const &mean);

		std::vector<Vec> const &getSamples() {
			return points; }
		inline std::vector<Scalar> const &getProbs() {
			return prob; }

	protected:
		Scalar chi2;
		Scalar dx;

		std::vector<Vec> points;
		std::vector<Scalar> prob;
//		Eigen::Matrix<Scalar, 3, MAX_POINTS> points;
//		Eigen::Matrix<Scalar, 1, MAX_POINTS> prob;
	};

}

#endif
