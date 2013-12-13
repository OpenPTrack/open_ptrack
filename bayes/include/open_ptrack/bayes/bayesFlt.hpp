#ifndef _BAYES_FILTER
#define _BAYES_FILTER

/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id: bayesFlt.hpp 634 2010-08-15 16:39:44Z mistevens $
 */

/*
 * Bayesian filtering represented as Dual hierarchy of:
 *  Prediction and Observation models
 *  Filtering Schemes
 */
 
// Common headers required for declarations
#include "bayesException.hpp"	// exception types
#include "matSupSub.hpp"			// matrix support subsystem

/* Filter namespace */
namespace Bayesian_filter
{
	// Allow use of a short name for matrix namespace
	namespace FM = Bayesian_filter_matrix;


/*
 * Abstraction support classes, at the base of the hierarchy
 */

class Bayes_base {
/*
 * A very abstract Polymorphic base representation!
 * Interface provides: type, internal error handing, and destruction
 */
public:
	typedef Bayesian_filter_matrix::Float Float;
	// Type used throughout as a number representation for state etc

	virtual ~Bayes_base() = 0;
	// Polymorphic

	static void error (const Numeric_exception& a);
	static void error (const Logic_exception& a);
	// Report a filter, throw a Filter_exception
	//  No exception safety rules are specified, assume the object is invalid
	// May have side effects for debugging
};


class Numerical_rcond
/*
 * Numerical comparison of reciprocal condition numbers
 *  Required for all linear algebra in models and filters
 *  Implements minimum allowable reciprocal condition number for PD Matrix factorisations
 */
{
public:
	Numerical_rcond()
	{	limit_PD = limit_PD_init;
	}
	void set_limit_PD(Bayes_base::Float nl)
	{	limit_PD = nl;
	}
	inline void check_PSD (Bayes_base::Float rcond, const char* error_description) const
	/* Checks a the reciprocal condition number
	 * Generates a Bayes_filter_exception if value represents a NON PSD matrix
	 * Inverting condition provides a test for IEC 559 NaN values
	 */
	{	if (!(rcond >= 0))
			Bayes_base::error (Numeric_exception (error_description));
	}

	inline void check_PD (Bayes_base::Float rcond, const char* error_description) const
	/* Checks a reciprocal condition number
	 * Generates a Bayes_filter_exception if value represents a NON PD matrix
	 * I.e. rcond is bellow given conditioning limit
	 * Inverting condition provides a test for IEC 559 NaN values
	 */
	{	if (!(rcond >= limit_PD))
			Bayes_base::error (Numeric_exception (error_description));
	}
private:
	Bayes_base::Float limit_PD;		
	const static Bayes_base::Float limit_PD_init;	// Initial common value for limit_PD
};


/*
 * Abstract Prediction models
 *  Predict models are used to parameterise predict functions of filters
 */
class Predict_model_base : public Bayes_base
{
	// Empty
};


class Sampled_predict_model : virtual public Predict_model_base
/* Sampled stochastic predict model
    x*(k) = fw(x(k-1), w(k))
   fw should generate samples from the stochastic variable w(k)
   This fundamental model is used instead of the predict likelihood function L(x*|x)
   Since drawing samples from an arbitrary L is non-trivial (see MCMC theory)
   the burden is place on the model to generate these samples.
   Defines an Interface without data members
 */
{
public:
	virtual const FM::Vec& fw(const FM::Vec& x) const = 0;
	// Note: Reference return value as a speed optimisation, MUST be copied by caller.
};

class Functional_predict_model :virtual public Predict_model_base
/* Functional (non-stochastic) predict model f
    x*(k) = fx(x(k-1))
   This fundamental model is used instead of the predict likelihood function L(x*|x)
   Since L is a delta function which isn't much use for numerical systems.
   Defines an Interface without data members
 */
{
public:
	virtual const FM::Vec& fx(const FM::Vec& x) const = 0;
	// Functional model
	// Note: Reference return value as a speed optimisation, MUST be copied by caller.
	
	const FM::Vec& operator()(const FM::Vec& x) const
	{	// Operator form of functional model
		return fx(x);
	}
};

class Gaussian_predict_model : virtual public Predict_model_base
/* Gaussian noise predict model
   This fundamental noise model for linear/linearised filtering
    x(k|k-1) = x(k-1|k-1)) + G(k)w(k)
    G(k)w(k)
    q(k) = state noise covariance, q(k) is covariance of w(k)
    G(k) = state noise coupling
*/
{
public:
	Gaussian_predict_model (std::size_t x_size, std::size_t q_size);

	FM::Vec q;		// Noise variance (always dense, use coupling to represent sparseness)
	FM::Matrix G;		// Noise Coupling
	
	Numerical_rcond rclimit;
	// Reciprocal condition number limit of linear components when factorised or inverted
};

class Additive_predict_model : virtual public Predict_model_base
/* Additive Gaussian noise predict model
   This fundamental model for non-linear filtering with additive noise
    x(k|k-1) = f(x(k-1|k-1)) + G(k)w(k)
    q(k) = state noise covariance, q(k) is covariance of w(k)
    G(k) = state noise coupling
   ISSUE Should be privately derived from Gaussian_predict_model but access control in GCC is broken
*/
{
public:
	Additive_predict_model (std::size_t x_size, std::size_t q_size);

	virtual const FM::Vec& f(const FM::Vec& x) const = 0;
	// Functional part of additive model
	// Note: Reference return value as a speed optimisation, MUST be copied by caller.

	FM::Vec q;		// Noise variance (always dense, use coupling to represent sparseness)
	FM::Matrix G;		// Noise Coupling

	Numerical_rcond rclimit;
	// Reciprocal condition number limit of linear components when factorised or inverted
};

class Linrz_predict_model : public Additive_predict_model
/* Linrz predict model
   This fundamental model for linear/linearised filtering
    x(k|k-1) = f(x(k-1|k-1)
    Fx(x(k-1|k-1) = Jacobian of of functional part fx with respect to state x
 */
{
public:
	Linrz_predict_model (std::size_t x_size, std::size_t q_size);
	FM::Matrix Fx;		// Model
};

class Linear_predict_model : public Linrz_predict_model
/* Linear predict model
   Enforces linearity on f
    x(k|k-1) = Fx(k-1|k-1) * x(k-1|k-1)
 */
{
public:
	Linear_predict_model (std::size_t x_size, std::size_t q_size);
	const FM::Vec& f(const FM::Vec& x) const
	{	// Provide the linear implementation of functional f
		xp.assign (FM::prod(Fx,x));
		return xp;
	}
private:
	mutable FM::Vec xp;
};

class Linear_invertable_predict_model : public Linear_predict_model
/* Linear invertable predict model
   Fx has an inverse
    x(k-1|k-1) = inv.Fx(k-1|k-1) * x(k|k-1)
 */
{
public:
	Linear_invertable_predict_model (std::size_t x_size, std::size_t q_size);
	struct inverse_model {
		inverse_model (std::size_t x_size);
		FM::ColMatrix Fx;	// Model inverse (ColMatrix as usually transposed)
	} inv;
};



/*
 * Abstract Observation models
 *  Observe models are used to parameterise the observe functions of filters
 */
class Observe_model_base : public Bayes_base
{
	// Empty
};

class Observe_function : public Bayes_base
// Function object for predict of observations
{
public:
	virtual const FM::Vec& h(const FM::Vec& x) const = 0;
	// Note: Reference return value as a speed optimisation, MUST be copied by caller.
};


class Likelihood_observe_model : virtual public Observe_model_base
/* Likelihood observe model L(z |x)
 *  The most fundamental Bayesian definition of an observation
 * Defines an Interface without data members
 */
{
public:
	Likelihood_observe_model(std::size_t z_size) : z(z_size)
	{}
	virtual Float L(const FM::Vec& x) const = 0;
	// Likelihood L(z | x)

	virtual void Lz (const FM::Vec& zz)
	// Set the observation zz about which to evaluate the Likelihood function
	{
		z = zz;
	}
protected:
	FM::Vec z;			// z set by Lz
};

class Functional_observe_model : virtual public Observe_model_base, public Observe_function
/* Functional (non-stochastic) observe model h
 *  zp(k) = hx(x(k|k-1))
 * This is a separate fundamental model and not derived from likelihood because:
 *  L is a delta function which isn't much use for numerical systems
 * Defines an Interface without data members
 */
{
public:
	Functional_observe_model(std::size_t /*z_size*/)
	{}
	const FM::Vec& operator()(const FM::Vec& x) const
	{	return h(x);
	}

};

class Parametised_observe_model : virtual public Observe_model_base, public Observe_function
/* Observation model parametised with a fixed z size
 *  Includes the functional part of a noise model
 *  Model is assume to have linear and non-linear components
 *  Linear components need to be checked for conditioning
 *  Non-linear components may be discontinuous and need normalisation
 */
{
public:
	Parametised_observe_model(std::size_t /*z_size*/)
	{}
	virtual const FM::Vec& h(const FM::Vec& x) const = 0;
	// Functional part of additive model
	virtual void normalise (FM::Vec& /*z_denorm*/, const FM::Vec& /*z_from*/) const
	// Discontinuous h. Normalise one observation (z_denorm) from another
	{}	//  Default normalisation, z_denorm unchanged
	
	Numerical_rcond rclimit;
	// Reciprocal condition number limit of linear components when factorised or inverted
};

class Uncorrelated_additive_observe_model : public Parametised_observe_model
/* Observation model, uncorrelated additive observation noise
	Z(k) = I * Zv(k) observe noise variance vector Zv
 */
{
public:
	Uncorrelated_additive_observe_model (std::size_t z_size) :
		Parametised_observe_model(z_size), Zv(z_size)
	{}
	FM::Vec Zv;			// Noise Variance
};

class Correlated_additive_observe_model : public Parametised_observe_model
/* Observation model, correlated additive observation noise
    Z(k) = observe noise covariance
 */
{
public:
	Correlated_additive_observe_model (std::size_t z_size) :
		Parametised_observe_model(z_size), Z(z_size,z_size)
	{}
	FM::SymMatrix Z;	// Noise Covariance (not necessarily dense)
};

class Jacobian_observe_model : virtual public Observe_model_base
/* Linrz observation model Hx, h about state x (fixed size)
    Hx(x(k|k-1) = Jacobian of h with respect to state x
	Normalisation consistency Hx: Assume normalise will be from h(x(k|k-1)) so result is consistent with Hx
 */
{
public:
	FM::Matrix Hx;		// Model
protected: // Jacobian model is not sufficient, it is used to build Linrz observe model's
	Jacobian_observe_model (std::size_t x_size, std::size_t z_size) :
		Hx(z_size, x_size)
	{}
};

class Linrz_correlated_observe_model : public Correlated_additive_observe_model, public Jacobian_observe_model
/* Linrz observation model Hx, h with respect to state x (fixed size)
    correlated observation noise
    zp(k) = h(x(k-1|k-1)
    Hx(x(k|k-1) = Jacobian of f with respect to state x
    Z(k) = observe noise covariance
 */
{
public:
	Linrz_correlated_observe_model (std::size_t x_size, std::size_t z_size) :
		Correlated_additive_observe_model(z_size), Jacobian_observe_model(x_size, z_size)
	{}
};

class Linrz_uncorrelated_observe_model : public Uncorrelated_additive_observe_model, public Jacobian_observe_model
/* Linrz observation model Hx, h with respect to state x (fixed size)
    uncorrelated observation noise
    zp(k) = h(x(k-1|k-1)
    Hx(x(k|k-1) = Jacobian of f with respect to state x
    Zv(k) = observe noise covariance
 */
{
public:
	Linrz_uncorrelated_observe_model (std::size_t x_size, std::size_t z_size) :
		Uncorrelated_additive_observe_model(z_size), Jacobian_observe_model(x_size, z_size)
	{}
};

class Linear_correlated_observe_model : public Linrz_correlated_observe_model
/* Linear observation model, correlated observation noise
    zp(k) = Hx(k) * x(k|k-1)
    Enforces linear model invariant. Careful when deriving to to change this invariant!
 */
{
public:
	Linear_correlated_observe_model (std::size_t x_size, std::size_t z_size) :
		Linrz_correlated_observe_model(x_size, z_size), hx(z_size)
	{}
	const FM::Vec& h(const FM::Vec& x) const
	{	// Provide a linear implementation of functional h assumes model is already Linrz for Hx
		hx.assign (FM::prod(Hx,x));
		return hx;
	}
private:
	mutable FM::Vec hx;
};

class Linear_uncorrelated_observe_model : public Linrz_uncorrelated_observe_model
/* Linear observation model, uncorrelated observation noise
    zp(k) = Hx(k) * x(k|k-1)
    Enforces linear model invariant. Careful when deriving to to change this invariant!
 */
{
public:
	Linear_uncorrelated_observe_model (std::size_t x_size, std::size_t z_size) :
		Linrz_uncorrelated_observe_model(x_size, z_size), hx(z_size)
	{}
	const FM::Vec& h(const FM::Vec& x) const
	{	// Provide a linear implementation of functional h assumes model is already Linrz for Hx
		hx.assign (FM::prod(Hx,x));
		return hx;
	}
private:
	mutable FM::Vec hx;
};


/*
 * Bayesian Filter
 *
 * A Bayesian Filter uses Bayes rule to fuse the state probabilities
 * of a prior and a likelihood function
 */
class Bayes_filter_base : public Bayes_base
{
	// Empty
};

/*
 * Likelihood Filter - Abstract filtering property
 * Represents only the Bayesian Likelihood of a state observation
 */
class Likelihood_filter : virtual public Bayes_filter_base
{
public:
	/* Virtual functions for filter algorithm */

	virtual void observe (Likelihood_observe_model& h, const FM::Vec& z) = 0;
	/* Observation state posterior using likelihood model h at z
	*/
};

/*
 * Functional Filter - Abstract filtering property
 * Represents only filter predict by a simple functional
 * (non-stochastic) model
 * 
 * A similar functional observe is not generally useful. The inverse of h is needed for observe!
 */
class Functional_filter : virtual public Bayes_filter_base
{
public:
	/* Virtual functions for filter algorithm */

	virtual void predict (Functional_predict_model& f) = 0;
	/* Predict state with functional no noise model
	    Requires x(k|k), X(k|k) or internal equivalent
	    Predicts x(k+1|k), X(k+1|k), using predict model
	*/
};

/*
 * State Filter - Abstract filtering property
 * Represents only filter state and an update on that state
 */
class State_filter : virtual public Bayes_filter_base
{
public:
	State_filter (std::size_t x_size);
	/* Set constant sizes, state must not be empty (must be >=1)
	    Exceptions:
	     bayes_filter_exception is x_size < 1
	 */

	FM::Vec x;			// expected state

	/* Virtual functions for filter algorithm */

	virtual void update () = 0;
	/* Update filters state
	    Updates x(k|k)
	*/
};


/*
 * Kalman State Filter - Abstract filtering property
 * Linear filter representation for 1st (mean) and 2nd (covariance) moments of a distribution
 *
 * Probability distributions are represented by state vector (x) and a covariance matrix.(X)
 *
 * State (x) sizes is assumed to remain constant.
 * The state and state covariance are public so they can be directly manipulated.
 *  init: Should be called if x or X are altered
 *  update: Guarantees that any internal changes made filter are reflected in x,X.
 *  This allows considerable flexibility so filter implementations can use different numerical representations
 *
 * Derived filters supply definitions for the abstract functions and determine the algorithm used
 * to implement the filter.
 */

class Kalman_state_filter : public State_filter
{
public:
	FM::SymMatrix X;	// state covariance

	Kalman_state_filter (std::size_t x_size);
	/* Initialise filter and set constant sizes
	 */

	/* Virtual functions for filter algorithm */

	virtual void init () = 0;
	/* Initialise from current state and state covariance
	    Requires x(k|k), X(k|k)
	*/
	void init_kalman (const FM::Vec& x, const FM::SymMatrix& X);
	/* Initialise from a state and state covariance
	    Parameters that reference the instance's x and X members are valid
	*/
	virtual void update () = 0;
	/* Update filters state and state covariance 
	    Updates x(k|k), X(k|k)
	*/
						
	// Minimum allowable reciprocal condition number for PD Matrix factorisations
	Numerical_rcond rclimit;
};


/*
 * Information State Filter - Abstract filtering property
 * Linear filter information space representation for 1st (mean) and 2nd (covariance) moments of a distribution
 *   Y = inv(X)   Information
 *   y = Y*x      Information state
 */

class Information_state_filter : virtual public Bayes_filter_base
{
public:
	Information_state_filter (std::size_t x_size);
	FM::Vec y;				// Information state
	FM::SymMatrix Y;		// Information

	virtual void init_yY () = 0;
	/* Initialise from a information state and information
	    Requires y(k|k), Y(k|k)
	    Parameters that reference the instance's y and Y members are valid
	*/
	void init_information (const FM::Vec& y, const FM::SymMatrix& Y);
	/* Initialise from a information state and information
	    Parameters that reference the instance's y and Y members are valid
	*/

	virtual void update_yY () = 0;
	/* Update filters information state and information
	    Updates y(k|k), Y(k|k)
	*/
};


/*
 * Linearizable filter models - Abstract filtering property
 *  Linrz == A linear, or gradient Linearized filter
 *
 * Predict uses a Linrz_predict_model that maintains a Jacobian matrix Fx and additive noise
 * NOTE: Functional (non-stochastic) predict is NOT possible as predict requires Fx.
 *
 * Observe uses a Linrz_observe_model and a variable size observation (z)
 * There are two variants for correlated and uncorrelated observation noise
 * Derived filters supply the init,predict,observe,update functions and determine
 * the algorithm used to implement the filter.
 */

class Linrz_filter : virtual public Bayes_filter_base
{ 
public:
	/* Virtual functions for filter algorithm */

	virtual Float predict (Linrz_predict_model& f) = 0;
	/* Predict state using a Linrz model
	    Requires x(k|k), X(k|k) or internal equivilent
	    Returns: Reciprocal condition number of primary matrix used in predict computation (1. if none)
	*/

	virtual Float observe (Linrz_uncorrelated_observe_model& h, const FM::Vec& z) = 0;
	virtual Float observe (Linrz_correlated_observe_model& h, const FM::Vec& z) = 0;
	/* Observation z(k) and with (Un)correlated observation noise model
	    Requires x(k|k), X(k|k) or internal equivalent
	    Returns: Reciprocal condition number of primary matrix used in observe computation (1. if none)
	*/
};


/*
 * Linearizable Kalman Filter
 *  Kalman state representation and linearizable models
 *
 * Common abstraction for many linear filters
 *  Has a virtual base to represent the common state
 */
class Linrz_kalman_filter : public Linrz_filter, virtual public Kalman_state_filter
{
protected:
	Linrz_kalman_filter() : Kalman_state_filter(0) // define a default constructor
	{}
};


/*
 * Extended Kalman Filter
 *  Kalman state representation and linearizable models
 *
 * Observe is implemented using an innovation computed from the non-linear part of the
 * observe model and linear part of the Linrz_observe_model
 *
 * Common abstraction for many linear filters
 *  Has a virtual base to represent the common state
 */
class Extended_kalman_filter : public Linrz_kalman_filter
{
protected:
	Extended_kalman_filter() : Kalman_state_filter(0) // define a default constructor
	{}
public:
	virtual Float observe (Linrz_uncorrelated_observe_model& h, const FM::Vec& z);
	virtual Float observe (Linrz_correlated_observe_model& h, const FM::Vec& z);
	/* Observation z(k) and with (Un)correlated observation noise model
	    Requires x(k|k), X(k|k) or internal equivalent
	    Returns: Reciprocal condition number of primary matrix used in observe computation (1. if none)
	    Default implementation simple computes innovation for observe_innovation
	*/

	virtual Float observe_innovation (Linrz_uncorrelated_observe_model& h, const FM::Vec& s) = 0;
	virtual Float observe_innovation (Linrz_correlated_observe_model& h, const FM::Vec& s) = 0;
	/* Observation innovation s(k) and with (Un)correlated observation noise model
	    Requires x(k|k), X(k|k) or internal equivalent
	    Returns: Reciprocal condition number of primary matrix used in observe computation (1. if none)
	*/
};


/*
 * Sample State Filter - Abstract filtering property
 *
 * Probability distributions are represented by a finite sampling
 *
 * State (x_size) size and its sampling (s_size) are assumed to remain constant.
 * The state sampling public so they can be directly manipulated.
 *  init: Should be used if they to be altered
 *  update: Guarantees that any internal changes made filter are reflected in sampling S.
 */

class Sample_state_filter : virtual public Bayes_filter_base
{
public:
	FM::ColMatrix S;		// state sampling (x_size,s_size)

	Sample_state_filter (std::size_t x_size, std::size_t s_size);
	/* Initialise filter and set constant sizes for
	    x_size of the state vector
	    s_size sample size
	    Exceptions:
	     bayes_filter_exception is s_size < 1
	*/
	~Sample_state_filter() = 0;

	/* Virtual functions for filter algorithm */

	virtual void init_S () = 0;
	/* Initialise from current sampling
	*/

	void init_sample (const FM::ColMatrix& initS);
	/* Initialise from a sampling
	 */

	virtual Float update_resample () = 0;
	/* Resampling update
	    Returns lcond, Smallest normalised likelihood weight, represents conditioning of resampling solution
	            lcond == 1. if no resampling performed
	            This should by multiplied by the number of samples to get the Likelihood function conditioning
	 */

	std::size_t unique_samples () const;
	/* Count number of unique (unequal value) samples in S
	    Implementation requires std::sort on sample column references
	*/
};


/*
 * Sample Filter: Bayes filter using
 *
 * Probability distributions are represented by a finite sampling
 *
 * The filter is operated by performing a
 * 	predict, observe
 * cycle derived from the bayes_filter. observe Likelihoods are merged into a single combined weight.
 *   update: MUST be used to complete a explicit resampling of the particles using merged weights
 *
 * Derived filters supply definitions for the abstract functions and determine the algorithm used
 * to implement the filter.
 */

class Sample_filter : public Likelihood_filter, public Functional_filter, virtual public Sample_state_filter
{
public:
	Sample_filter (std::size_t x_size, std::size_t s_size);
	/* Initialise filter and set constant sizes for
	    x_size of the state vector
	    s_size sample size
	    Exceptions:
	     bayes_filter_exception is s_size < 1
	*/

	/* Virtual functions for filter algorithm */

	virtual void predict (Functional_predict_model& f);
	/* Predict state posterior with functional no noise model
	*/

	virtual void predict (Sampled_predict_model& f) = 0;
	/* Predict state posterior with sampled noise model
	*/

	virtual void observe (Likelihood_observe_model& h, const FM::Vec& z) = 0;
	/* Observation state posterior using likelihood model h at z
	*/

	virtual void observe_likelihood (const FM::Vec& lw) = 0;
	/* Observation fusion directly from likelihood weights
	    lw may be smaller then the state sampling. Weights for additional particles are assumed to be 1
	*/
};


}//namespace
#endif
