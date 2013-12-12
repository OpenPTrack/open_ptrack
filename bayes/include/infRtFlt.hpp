#ifndef _BAYES_FILTER_INFORMATION_ROOT
#define _BAYES_FILTER_INFORMATION_ROOT

/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id: infRtFlt.hpp 634 2010-08-15 16:39:44Z mistevens $
 */

/*
 * Information Root Filter Scheme.
 *  A extended 'Square-root' Information filter as an Abstract class
 *
 * Algorithm: Square-root information propogation using QR factorisation
 * Ref:	P. Dyer and S. McReynolds, "Extension of Square-Root Filtering to Include Process Noise",
 * [1] Journal of Optimization Theory and Applications, Vol.3 No.6 1969
 * Filter maintains r,R where
 *   inv(R)*inv(R)' = X
 *   r = R*x
 *   R is upper triangular but not strictly a Cholesky factor as diagonal may be negative
 * Observe algorithm has been extended to include linearised models
 * Discontinous observe models require that state is normailised with respect to the observation.
 *
 * The filter is operated by performing a
 *  predict, observe
 * cycle defined by the base class
 */
#include "bayesFlt.hpp"

/* Filter namespace */
namespace Bayesian_filter
{

class Information_root_scheme : public Extended_kalman_filter
{
public:
	FM::Vec r;			// Information Root state
	FM::UTriMatrix R;	// Information Root

	Information_root_scheme (std::size_t x_size, std::size_t z_initialsize = 0);

	void init ();
	void update ();
	// Covariance form state interface

	Float predict (Linrz_predict_model& f, const FM::ColMatrix& invFx, bool linear_r);
	/* Generialised form, using precomputed inverse of f.Fx */
	Float predict (Linrz_predict_model& f);
	/* Use linrz form for r, computes inverse model using inverse_Fx */
	Float predict (Linear_predict_model& f);
	/* Use linear form for r, computes inverse model using inverse_Fx */
	Float predict (Linear_invertable_predict_model& f)
	/* Use linear form for r, and use inv.Fx from invertable model */
	{
		return predict(f, f.inv.Fx, true);
	}

	Float observe_innovation (Linrz_uncorrelated_observe_model& h, const FM::Vec& s);
	Float observe_innovation (Linrz_correlated_observe_model& h, const FM::Vec& s);
	// Extended_kalman_filter observe

	static void inverse_Fx (FM::DenseColMatrix& invFx, const FM::Matrix& Fx);
	/* Numerical Inversion of Fx using LU factorisation */
};


/*
 * Information Root Filter Scheme with exposed information state
 * Augments Information_root_filter with y,Y in the interface
 */

class Information_root_info_scheme : public Information_root_scheme, virtual public Information_state_filter
{
public:
	Information_root_info_scheme (std::size_t x_size, std::size_t z_initialsize = 0);

	void init_yY ();
	void update_yY ();
	// Information form state interface
};


}//namespace
#endif
