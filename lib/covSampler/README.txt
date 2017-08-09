USAGE:

Constructor and sets():
	CovSampling(Scalar chi2, Scalar dx);
	void setConf(Scalar chi2);
	void setDist(Scalar dx)
	
	- chi2 defines the size of the ellipsoid you want to sample, with larger values corresponding to larger ellipsoids. In particular, this is the value of a chi-squared distribuition with 3 degrees of freedom that represents the probability of a sample point missing the region defined by the ellipsoid that represents the covariance matrix of a normal distribution. 
	Typical values for chi2: 4.64 -> 20%, 6.25 -> 10%, 7.82 -> 5%, 11.34 -> 1%
	
	- dx is size of the grid used for sampling the distribution, i.e., a value dx=2 represents a grid sampled every 2 units along each direction.

Sampling:
	void sample(Mat const &cov, Vec const &mean);
	std::vector<Vec> const &getSamples();
	std::vector<Scalar> const &getProbs();
	
	- cov and mean represent the covariance matrix and the mean of the normal distribution to be sampled;
	
	- after sampling you can get a std::vector collection of samples with getSamples() and the corresponding values of the probability density function of the normal distribution at those points with getProbs(), that also returns a std::vector. Note: this pdf values are normalized so that they sum to 1.0.

Check covSamplerTester.cpp for a very simple example.