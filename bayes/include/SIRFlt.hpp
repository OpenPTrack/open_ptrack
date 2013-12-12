#ifndef _BAYES_FILTER_SIR
#define _BAYES_FILTER_SIR

/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id: SIRFlt.hpp 634 2010-08-15 16:39:44Z mistevens $
 */

/*
 * Sampling Importance Resampleing Filter Scheme.
 *  Also know as a weighted Booststrap
 *
 * References
 *  [1] "Novel approach to nonlinear-non-Guassian Bayesian state estimation"
 *   NJ Gordon, DJ Salmond, AFM Smith IEE Proceeding-F Vol.140 No.2 April 1993
 *  [2] Building Robust Simulation-based Filter for Evolving Data Sets"
 *   J Carpenter, P Clifford, P Fearnhead Technical Report Unversity of Oxford
 *
 *  A variety of reampling algorithms can be used for the SIR filter.
 *  There are implementations for two algorihtms:
 *   standard_resample: Standard resample algorithm from [1]
 *   systematic_resample: A Simple stratified resampler from [2]
 *  A virtual 'weighted_resample' provides an standard interface to these and defaults
 *  to the standard_resample.
 *
 * NOTES:
 *  SIR algorithm is sensative to random generator
 *  In particular random uniform must be [0..1) NOT [0..1]
 *  Quantisation in the random number generator must not approach the sample size.
 *  This will result in quantisation of the resampling.
 *  For example if random identically equal to 0 becomes highly probable due to quantisation
 *  this will result in the first sample being selectively draw whatever its likelihood.
 *
 *  Numerics
 *   Resampling requires comparisons of normalised weights. These may
 *   become insignificant if Likelihoods have a large range. Resampling becomes ill conditioned
 *   for these samples.
 */
#include "bayesFlt.hpp"

/* Filter namespace */
namespace Bayesian_filter
{

struct SIR_random
/*
 * Random number generators interface
 *  Helper to allow polymorthic use of random number generators
 */
{
	virtual void normal(FM::DenseVec& v) = 0;
	virtual void uniform_01(FM::DenseVec& v) = 0;
	virtual ~SIR_random () {}
};


class Importance_resampler : public Bayes_base
/*
 * Importance resampler
 *  Represents a function that computes the posterior resampling from importance weights
 *  Polymorphic function object used to parameterise the resampling operation
 */
{
public:
	typedef std::vector<std::size_t> Resamples_t;	// resampling counts

	virtual Float resample (Resamples_t& presamples, std::size_t& uresamples, FM::DenseVec& w, SIR_random& r) const = 0;
	/*
	 * The resampling function
	 *  Weights w are proportional to the posterior Likelihood of a state
	 * Sideeffect
	 *  w becomes a normalised cumulative sum
	 *  Random draws can be made from 'r'
	 *
	 * Exceptions:
	 *  bayes_filter_exception for numerical problems with weights including
	 *   any w < 0, all w == 0
	 * Return
	 *	lcond, smallest normalised weight, represents conditioning of resampling solution
	 *	presamples: posterior resample, the number of times each sample should appear posterior baes on it weight
	 *  uresample: the number of unique resamples in the posterior == number of non zero elements in presamples
	 * Preconditions
	 *  wresample,w must have same size
	 */
};

class Standard_resampler : public Importance_resampler
// Standard resample algorithm from [1]
{
public:
	Float resample (Resamples_t& presamples, std::size_t& uresamples, FM::DenseVec& w, SIR_random& r) const;
};

class Systematic_resampler : public Importance_resampler
// Systematic resample algorithm from [2]
{
public:
	Float resample (Resamples_t& presamples, std::size_t& uresamples, FM::DenseVec& w, SIR_random& r) const;
};


template <class Predict_model>
class Sampled_general_predict_model: public Predict_model, public Sampled_predict_model
/*
 * Generalise a predict model to sampled predict model using and SIR random
 *  To instantiate template std::sqrt is required
 */
{
public:
	Sampled_general_predict_model (std::size_t x_size, std::size_t q_size, SIR_random& random_helper) :
		Predict_model(x_size, q_size),
		Sampled_predict_model(),
		genn(random_helper),
		xp(x_size),
		n(q_size), rootq(q_size)
	{
		first_init = true;
	}

	virtual const FM::Vec& fw(const FM::Vec& x) const
	/*
	 * Definition of sampler for addative noise model given state x
	 *  Generate Gaussian correlated samples
	 * Precond: init_GqG, automatic on first use
	 */
	{
		if (first_init)
			init_GqG();
							// Predict state using supplied functional predict model
		xp = Predict_model::f(x);
							// Additive random noise
		genn.normal(n);				// independant zero mean normal
									// multiply elements by std dev
		for (FM::DenseVec::iterator ni = n.begin(); ni != n.end(); ++ni) {
			*ni *= rootq[ni.index()];
		}
		FM::noalias(xp) += FM::prod(this->G,n);			// add correlated noise
		return xp;
	}

	void init_GqG() const
	/* initialise predict given a change to q,G
	 *  Implementation: Update rootq
	 */
	{
		first_init = false;
		for (FM::Vec::const_iterator qi = this->q.begin(); qi != this->q.end(); ++qi) {
			if (*qi < 0)
				error (Numeric_exception("Negative q in init_GqG"));
			rootq[qi.index()] = std::sqrt(*qi);
		}
	}
private:
	SIR_random& genn;
	mutable FM::Vec xp;
	mutable FM::DenseVec n;
	mutable FM::Vec rootq;		// Optimisation of sqrt(q) calculation, automatic on first use
	mutable bool first_init;	
};

typedef Sampled_general_predict_model<Linear_predict_model> Sampled_LiAd_predict_model;
typedef Sampled_general_predict_model<Linear_invertable_predict_model> Sampled_LiInAd_predict_model;
// Sampled predict model generalisations
//  Names a shortened to first two letters of their model properties



class SIR_scheme : public Sample_filter
/*
 * Sampling Importance Resampleing Filter Scheme.
 *  Implement a general form of SIR filter
 *  Importance resampling is delayed until an update is required. The sampler used
 *  is a parameter of update to allow a wide variety of usage.
 *  A stochastic sample is defined as a sample with a unqiue stochastic history other then roughening
 */
{
	friend class SIR_kalman_scheme;
public:
	std::size_t stochastic_samples;	// Number of stochastic samples in S

	SIR_scheme (std::size_t x_size, std::size_t s_size, SIR_random& random_helper);
	SIR_scheme& operator= (const SIR_scheme&);
	// Optimise copy assignment to only copy filter state

	/* Specialisations for filter algorithm */
	void init_S ();

	Float update_resample ()
	// Default resampling update
	{	return update_resample (Standard_resampler());
	}

	virtual Float update_resample (const Importance_resampler& resampler);
	/* Update: resample particles using weights and then roughen
	 *  Return: lcond
	 */

	void predict (Functional_predict_model& f)
	// Predict samples without noise
	{	Sample_filter::predict (f);
	}
	void predict (Sampled_predict_model& f);
	// Predict samples with noise model

	void observe (Likelihood_observe_model& h, const FM::Vec& z);
	// Weight particles using likelihood model h and z

	void observe_likelihood (const FM::Vec& lw);
	// Observation fusion directly from likelihood weights

	Float rougheningK;			// Current roughening value (0 implies no roughening)
	virtual void roughen()
	// Generalised roughening:  Default to roughen_minmax
	{
		if (rougheningK != 0)
			roughen_minmax (S, rougheningK);
	}

	static void copy_resamples (FM::ColMatrix& P, const Importance_resampler::Resamples_t& presamples);
	// Update P by selectively copying based on presamples 

	SIR_random& random;			// Reference random number generator helper

protected:
	void roughen_minmax (FM::ColMatrix& P, Float K) const;	// roughening using minmax of P distribution
	Importance_resampler::Resamples_t resamples;		// resampling counts
	FM::DenseVec wir;			// resamping weights
	bool wir_update;			// weights have been updated requring a resampling on update
private:
	static const Float rougheningKinit;
	std::size_t x_size;
};


class SIR_kalman_scheme : public SIR_scheme, virtual public Kalman_state_filter
/*
 * SIR implementation of a Kalman filter
 *  Updates Kalman statistics of SIR_filter
 *  These statistics are use to provide a specialised correlated roughening procedure
 */
{
public:
	SIR_kalman_scheme (std::size_t x_size, std::size_t s_size, SIR_random& random_helper);

	/* Specialisations for filter algorithm */

	void init ();

	void update ()
	// Implement Kalman_filter::update identically to SIR_scheme
	{	(void)SIR_scheme::update_resample();
	}

	Float update_resample ()
	// Implement identically to SIR_scheme
	{	return SIR_scheme::update_resample();
	}

	Float update_resample (const Importance_resampler& resampler);
	// Modified SIR_filter update implementation: update mean and covariance of sampled distribution

	void update_statistics ();
	// Update kalman statistics without resampling

	void roughen()
	{	// Specialised correlated roughening
		if (rougheningK != 0)
			roughen_correlated (S, rougheningK);
	}

protected:
	void roughen_correlated (FM::ColMatrix& P, Float K);	// Roughening using covariance of P distribution
	Sampled_LiAd_predict_model roughen_model;		// roughening predict
private:
	static Float scaled_vector_square(const FM::Vec& v, const FM::SymMatrix& S);
	void mean();
};


}//namespace
#endif
