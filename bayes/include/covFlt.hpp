#ifndef _BAYES_FILTER_COVARIANCE
#define _BAYES_FILTER_COVARIANCE

/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id: covFlt.hpp 634 2010-08-15 16:39:44Z mistevens $
 */

/*
 * Covariance Filter Scheme.
 *  Implemention of extended Kalman filter
 * 
 * To work with with Linear and Linrz models
 *  a) a state seperate from covariance predict is used.
 *  b) a EKF innovation update algorithm is used.
 * Discontinous observe models require that predict is normailised with
 * respect to the observation.
 *
 * A initial observation size may also be specified for efficiency.
 * 
 * The filter is operated by performing a
 * predict, observe
 * cycle defined by the base class
 */
#include "bayesFlt.hpp"

/* Filter namespace */
namespace Bayesian_filter
{

class Covariance_scheme : public Extended_kalman_filter
{
public:
	Covariance_scheme (std::size_t x_size, std::size_t z_initialsize = 0);
	Covariance_scheme& operator= (const Covariance_scheme&);
	// Optimise copy assignment to only copy filter state

	void init ();
	void update ();

	Float predict (Linrz_predict_model& f);
	// Standard Linrz prediction
	Float predict (Gaussian_predict_model& f);
	// Specialised 'stationary' prediction, only addative noise

	Float observe_innovation (Linrz_uncorrelated_observe_model& h, const FM::Vec& s);
	Float observe_innovation (Linrz_correlated_observe_model& h, const FM::Vec& s);

public:						// Exposed Numerical Results
	FM::SymMatrix S, SI;		// Innovation Covariance and Inverse
	FM::Matrix W;				// Kalman Gain

protected:			   		// Permenantly allocated temps
	FM::RowMatrix tempX;
protected:					// allow fast operation if z_size remains constant
	std::size_t last_z_size;
	void observe_size (std::size_t z_size);
};


}//namespace
#endif
