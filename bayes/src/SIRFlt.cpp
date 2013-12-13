/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id: SIRFlt.cpp 634 2010-08-15 16:39:44Z mistevens $
 */

/*
 * Sampling Importance Resampling Filter.
 *
 * Bootstrap filter (Sequential Importance Resampling).
 */
#include "SIRFlt.hpp"
#include "matSup.hpp"
#include "models.hpp"
#include <algorithm>
#include <iterator>
#include <vector>

namespace {

template <class scalar>
inline scalar sqr(scalar x)
// Square 
{
	return x*x;
}
};//namespace


/* Filter namespace */
namespace Bayesian_filter
{
	using namespace Bayesian_filter_matrix;



Standard_resampler::Float
 Standard_resampler::resample (Resamples_t& presamples, std::size_t& uresamples, FM::DenseVec& w, SIR_random& r) const
/* Standard resampler from [1]
 * Algorithm:
 *	A particle is chosen once for each time its cumulative weight intersects with a uniform random draw.
 *	Complexity O(n*log(n))
 *  This complexity is required to sort the uniform random draws made,
 *	this allows comparing of the two ordered lists w(cumulative) and ur (the sort random draws).
 * Output:
 *  presamples number of times this particle should be resampled
 *  uresamples number of unqiue particles (number of non zeros in Presamples)
 *  w becomes a normalised cumulative sum
 * Side effects:
 *  A draw is made from 'r' for each particle
 */
{
	assert (presamples.size() == w.size());
	DenseVec::iterator wi, wi_end = w.end();
						// Normalised cumulative sum of likelihood weights (Kahan algorithm), and find smallest weight
	Float wmin = std::numeric_limits<Float>::max();
	Float wcum = 0;
	{
		Float c = 0, y, t;
		for (wi = w.begin(); wi != wi_end; ++wi) {
			if (*wi < wmin) {
				wmin = *wi;
			}
			y = *wi - c;
			t = wcum + y;
			c = t - wcum - y;
			wcum = *wi = t;
		}
	}
	if (wmin < 0)		// bad weights
		error (Numeric_exception("negative weight"));
	if (wcum <= 0)		// bad cumulative weights (previous check should actually prevent -ve
		error (Numeric_exception("zero cumulative weight sum"));
						// Any numerical failure should cascade into cumulative sum
	if (wcum != wcum)		// inequality due to NaN
		error (Numeric_exception("NaN cumulative weight sum"));

						// Sorted uniform random distribution [0..1) for each resample
	DenseVec ur(w.size());
	r.uniform_01(ur);
	std::sort  (ur.begin(), ur.end());
	assert (ur[0] >= 0 && ur[ur.size()-1] < 1);	// very bad if random is incorrect
						// Scale ur to cumulative sum
	ur *= wcum;
						// Resamples based on cumulative weights from sorted resample random values
	Resamples_t::iterator pri = presamples.begin();
	wi = w.begin();
	DenseVec::iterator ui = ur.begin(), ui_end = ur.end();
	std::size_t unique = 0;

	while (wi != wi_end)
	{
		std::size_t Pres = 0;		// assume P not resampled until find out otherwise
		if (ui != ui_end && *ui < *wi)
		{
			++unique;
			do						// count resamples
			{
				++Pres;
				++ui;
			} while (ui != ui_end && *ui < *wi);
		}
		++wi;
		*pri++ = Pres;
	}
	assert (pri==presamples.end());	// must traverse all of P
	
	if (ui != ui_end)				// resample failed due no non numeric weights
		error (Numeric_exception("weights are not numeric and cannot be resampled"));
	
	uresamples = unique;
	return wmin / wcum;
}


Systematic_resampler::Float
 Systematic_resampler::resample (Resamples_t& presamples, std::size_t& uresamples, FM::DenseVec& w, SIR_random& r) const
/* Systematic resample algorithm from [2]
 * Algorithm:
 *	A particle is chosen once for each time its cumulative weight intersects with an equidistant grid.
 *	A uniform random draw is chosen to position the grid within the cumulative weights
 *	Complexity O(n)
 * Output:
 *  presamples number of times this particle should be resampled
 *  uresamples number of unqiue particles (number of non zeros in Presamples)
 *  w becomes a normalised cumulative sum
 * Sideeffects:
 *  A single draw is made from 'r'
 */
{
	std::size_t nParticles = presamples.size();
	assert (nParticles == w.size());
	DenseVec::iterator wi, wi_end = w.end();
						// Normalised cumulative sum of likelihood weights (Kahan algorithm), and find smallest weight
	Float wmin = std::numeric_limits<Float>::max();
	Float wcum = 0;
	{
		Float c = 0, y, t;
		for (wi = w.begin(); wi != wi_end; ++wi) {
			if (*wi < wmin) {
				wmin = *wi;
			}
			y = *wi - c;
			t = wcum + y;
			c = t - wcum - y;
			wcum = *wi = t;
		}
	}
	if (wmin < 0)		// bad weights
		error (Numeric_exception("negative weight"));
	if (wcum <= 0)		// bad cumulative weights (previous check should actually prevent -ve
		error (Numeric_exception("total likelihood zero"));
						// Any numerical failure should cascade into cumulative sum
	if (wcum != wcum)
		error (Numeric_exception("total likelihood numerical error"));

						// Stratified step
	Float wstep = wcum / Float(nParticles);
						
	DenseVec ur(1);				// single uniform for initialisation
	r.uniform_01(ur);
	assert (ur[0] >= 0 && ur[0] < 1);		// bery bad if random is incorrect

						// Resamples based on cumulative weights
	Importance_resampler::Resamples_t::iterator pri = presamples.begin();
	wi = w.begin();
	std::size_t unique = 0;
	Float s = ur[0] * wstep;		// random initialisation

	while (wi != wi_end)
	{
		std::size_t Pres = 0;			// assume P not resampled until find out otherwise
		if (s < *wi)
		{
			++unique;
			do							// count resamples
			{
				++Pres;
				s += wstep;
			} while (s < *wi);
		}
		++wi;
		*pri++ = Pres;
	}
	assert (pri==presamples.end());	// must traverse all of P

	uresamples = unique;
	return wmin / wcum;
}


/*
 * SIR filter implementation
 */
const SIR_scheme::Float SIR_scheme::rougheningKinit = 1;
		// use 1 std.dev. per sample as default roughening

SIR_scheme::SIR_scheme (std::size_t x_size, std::size_t s_size, SIR_random& random_helper) :
		Sample_state_filter (x_size, s_size),
		Sample_filter (x_size, s_size),
		random (random_helper),
		resamples (s_size), wir (s_size)
/* Initialise filter and set the size of things we know about
 */
{
	SIR_scheme::x_size = x_size;
	rougheningK = rougheningKinit;
}

SIR_scheme& SIR_scheme::operator= (const SIR_scheme& a)
/* Optimise copy assignment to only copy public filter state (particles and weights)
 * random helper is not part of filter state
 * Precond: matrix size conformance
 */
{
	Sample_filter::operator=(a);
	stochastic_samples = a.stochastic_samples;			// Copy weights
	wir_update = a.wir_update;
	return *this;
}


void
 SIR_scheme::init_S ()
/* Initialise sampling
 *  Pre: S
 *  Post: stochastic_samples := samples in S
 */
{
	stochastic_samples = S.size2();
	std::fill (wir.begin(), wir.end(), Float(1));		// Initial uniform weights
	wir_update = false;
}


SIR_scheme::Float
 SIR_scheme::update_resample (const Importance_resampler& resampler)
/* Resample particles using weights and roughen
 * Pre : S represent the predicted distribution
 * Post: S represent the fused distribution, n_resampled from weighted_resample
 * Exceptions:
 *  Bayes_filter_exception from resampler
 *    unchanged: S, stochastic_samples
 * Return
 *  lcond, Smallest normalised weight, represents conditioning of resampling solution
 *  lcond == 1 if no resampling performed
 *  This should by multiplied by the number of samples to get the Likelihood function conditioning
 */
{
	Float lcond = 1;
	if (wir_update)		// Resampling only required if weights have been updated
	{
		// Resample based on likelihood weights
		std::size_t R_unique;
		lcond = resampler.resample (resamples, R_unique, wir, random);

							// No resampling exceptions: update S
		copy_resamples (S, resamples);
		stochastic_samples = R_unique;

		roughen ();			// Roughen samples

		std::fill (wir.begin(), wir.end(), Float(1));		// Resampling results in uniform weights
		wir_update = false;
	}
	return lcond;
}


void
 SIR_scheme::predict (Sampled_predict_model& f)
/* Predict state posterior with sampled noise model
 *  Pre : S represent the prior distribution
 *  Post: S represent the predicted distribution, stochastic_samples := samples in S
 */
{
						// Predict particles S using supplied predict model
	const std::size_t nSamples = S.size2();
	for (std::size_t i = 0; i != nSamples; ++i) {
		FM::ColMatrix::Column Si(S,i);
		noalias(Si) = f.fw(Si);
	}
	stochastic_samples = S.size2();
}


void
 SIR_scheme::observe (Likelihood_observe_model& h, const Vec& z)
/* Observation fusion using Likelihood at z
 * Pre : wir previous particle likelihood weights
 * Post: wir fused (multiplicative) particle likelihood weights
 */
{
	h.Lz (z);			// Observe likelihood at z

						// Weight Particles. Fused with previous weight
	const std::size_t nSamples = S.size2();
	for (std::size_t i = 0; i != nSamples; ++i) {
		wir[i] *= h.L (FM::column(S,i));
	}
	wir_update = true;
}

void
 SIR_scheme::observe_likelihood (const Vec& lw)
/* Observation fusion directly from likelihood weights
 * lw may be smaller then the number of particles. Weights for additional particles are assumed to be 1
 * Pre : wir previous particle likelihood weights
 * Post: wir fused (multiplicative) particle likelihood weights
 */
{
					// Weight Particles. Fused with previous weight
	Vec::const_iterator lw_end = lw.end();
	for (Vec::const_iterator lw_i = lw.begin(); lw_i != lw_end; ++lw_i) {
		wir[lw_i.index()] *= *lw_i;
	}
	wir_update = true;
}


void SIR_scheme::copy_resamples (ColMatrix& P, const Importance_resampler::Resamples_t& presamples)
/* Update P by selectively copying presamples
 * Uses a in-place copying algorithm
 * Algorithm: In-place copying
 *  First copy the live samples (those resampled) to end of P
 *  Replicate live sample in-place
 */
{
							// reverse_copy_if live
	std::size_t si = P.size2(), livei = si;
	Importance_resampler::Resamples_t::const_reverse_iterator pri, pri_end = presamples.rend();
	for (pri = presamples.rbegin(); pri != pri_end; ++pri) {
		--si;
		if (*pri > 0) {
			--livei;
			noalias(FM::column(P,livei)) = FM::column(P,si);
		}
	}
	assert (si == 0);
							// Replicate live samples
	si = 0;
	Importance_resampler::Resamples_t::const_iterator pi, pi_end = presamples.end();
	for (pi = presamples.begin(); pi != pi_end; ++pi) {
		std::size_t res = *pi;
		if (res > 0) {
			do  {
				noalias(FM::column(P,si)) = FM::column(P,livei);
				++si; --res;
			} while (res > 0);
			++livei;
		}
	}
	assert (si == P.size2()); assert (livei == P.size2());
}


void SIR_scheme::roughen_minmax (ColMatrix& P, Float K) const
/* Roughening
 *  Uses algorithm from Ref[1] using max-min in each state of P
 *  K is scaling factor for roughening noise
 *  unique_samples is unchanged as roughening is used to postprocess observe resamples
 * Numerical collapse of P
 *  P with very small or zero range result in minimal roughening
 * Exceptions:
 *  none
 *		unchanged: P
 */
{
	using namespace std;
						// Scale Sigma by constant and state dimensions
	Float SigmaScale = K * pow (Float(P.size2()), -1/Float(x_size));

						// Find min and max states in all P, precond P not empty
	Vec xmin(x_size); noalias(xmin) = column(P,0);
	Vec xmax(x_size); noalias(xmax) = xmin;
	ColMatrix::iterator2 pi = P.begin2();
	while (pi != P.end2())		// Loop includes 0 to simplify code
	{
		Vec::iterator mini = xmin.begin();
		Vec::iterator maxi = xmax.begin();

#ifdef BOOST_UBLAS_NO_NESTED_CLASS_RELATION
		for (ColMatrix::iterator1 xpi = begin(pi, ublas::iterator2_tag()); xpi != end(pi, ublas::iterator2_tag()); ++xpi)
#else
		for (ColMatrix::iterator1 xpi = pi.begin(); xpi != pi.end(); ++xpi)
#endif
		{
			if (*xpi < *mini) *mini = Float(*xpi);	// ISSUE mixed type proxy assignment
			if (*xpi > *maxi) *maxi = Float(*xpi);
			++mini; ++maxi;
		}
		++pi;
	}
   						// Roughening st.dev max-min
	Vec rootq(x_size);
	rootq = xmax;
	noalias(rootq) -= xmin;
	rootq *= SigmaScale;
   						// Apply roughening predict based on scaled variance
	DenseVec n(x_size);
	for (pi = P.begin2(); pi != P.end2(); ++pi) {
		random.normal (n);			// independent zero mean normal
									// multiply elements by std dev
		for (DenseVec::iterator ni = n.begin(); ni != n.end(); ++ni) {
			*ni *= rootq[ni.index()];
		}
		ColMatrix::Column Pi(P,pi.index2());
		noalias(n) += Pi;			// add to P
		Pi = n;
	}
}


/*
 * SIR implementation of a Kalman filter
 */
SIR_kalman_scheme::SIR_kalman_scheme (std::size_t x_size, std::size_t s_size, SIR_random& random_helper) :
	Sample_state_filter (x_size, s_size), Kalman_state_filter(x_size),
	SIR_scheme (x_size, s_size, random_helper),
	roughen_model (x_size,x_size, random_helper)
{
}

void SIR_kalman_scheme::init ()
/* Initialise sampling from Kalman statistics
 *	Pre: x,X
 *	Post: x,X,S
 */
{
						// Samples at mean
	const std::size_t nSamples = S.size2();
	for (std::size_t i = 0; i != nSamples; ++i) {
		FM::ColMatrix::Column Si(S,i);
		noalias(Si) = x;
	}
						// Decorrelate init state noise
	Matrix UD(x_size,x_size);
	Float rcond = UdUfactor (UD, X);
	rclimit.check_PSD(rcond, "Init X not PSD");

						// Sampled predict model for initial noise variance
	FM::identity (roughen_model.Fx);
	UdUseperate (roughen_model.G, roughen_model.q, UD);
	roughen_model.init_GqG ();
						// Predict using model to apply initial noise
	predict (roughen_model);

	SIR_scheme::init_S ();
}


void SIR_kalman_scheme::mean ()
/* Update state mean
 *  Pre : S
 *  Post: x,S
 */
{
						// Mean of distribution: mean of particles
	x.clear();
	const std::size_t nSamples = S.size2();
	for (std::size_t i = 0; i != nSamples; ++i) {
		FM::ColMatrix::Column Si(S,i);
		x.plus_assign (Si);
	}
	x /= Float(S.size2());
}


Bayes_base::Float
 SIR_kalman_scheme::update_resample (const Importance_resampler& resampler)
/* Modified SIR_scheme update implementation
 *  update mean and covariance of sampled distribution with update_statistics
 * Normal numerical circumstances (such as all identical samples)
 * may lead to illconditioned X which is not PSD when factorised.
 */
{
	Float lcond = SIR_scheme::update_resample(resampler);	// Resample particles

	update_statistics();			// Estimate sample mean and covariance

	return lcond;
}

void SIR_kalman_scheme::update_statistics ()
/* Update kalman statistics without resampling
 * Update mean and covariance estimate of sampled distribution => mean and covariance of particles
 *  Pre : S (s_size >=1 enforced by state_filter base)
 *  Post: x,X,S	(X may be non PSD)
 *
 * Sample Covariance := Sum_i [transpose(S[i]-mean)*(S[i]-mean)] / (s_size)
 *  The definition is the Maximum Likelihood (biased) estimate of covariance given samples with unknown (estimated) mean
 * Numerics
 *  No check is made for the conditioning of samples with regard to mean and covariance
 *  Extreme ranges or very large sample sizes will result in inaccuracy
 *  The covariance should always remain PSD however
 */
{
    mean ();
    X.clear();              // Covariance

	const std::size_t nSamples = S.size2();
	for (std::size_t i = 0; i != nSamples; ++i) {
		FM::ColMatrix::Column Si(S,i);
		X.plus_assign (FM::outer_prod(Si-x, Si-x));
	}
	X /= Float(nSamples);
}


void SIR_kalman_scheme::roughen_correlated (ColMatrix& P, Float K)
/* Roughening
 *  Uses a roughening noise based on covariance of P
 *  This is a more sophisticated algorithm then Ref[1] as it takes
 *  into account the correlation of P
 *  K is scaling factor for roughening noise
 * Numerical colapse of P
 *  Numerically when covariance of P semi definite (or close), X's UdU factorisation
 *  may be negative.
 * Exceptions:
 *   Bayes_filter_exception due collapse of P
 *    unchanged: P
 */
{
	using namespace std;
						// Scale variance by constant and state dimensions
	Float VarScale = sqr(K) * pow (Float(P.size2()), Float(-2.)/Float(x_size));

	update_statistics();	// Estimate sample mean and covariance

						// Decorrelate states
	Matrix UD(x_size,x_size);
	Float rcond = UdUfactor (UD, X);
	rclimit.check_PSD(rcond, "Roughening X not PSD");

						// Sampled predict model for roughening
	FM::identity (roughen_model.Fx);
						// Roughening predict based on scaled variance
	UdUseperate (roughen_model.G, roughen_model.q, UD);
	roughen_model.q *= VarScale;
	roughen_model.init_GqG();
						// Predict using roughening model
	predict (roughen_model);
}


}//namespace
