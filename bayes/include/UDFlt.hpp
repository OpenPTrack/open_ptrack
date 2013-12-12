#ifndef _BAYES_FILTER_UD
#define _BAYES_FILTER_UD
/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id: UDFlt.hpp 634 2010-08-15 16:39:44Z mistevens $
 */

/*
 * UdU' Factorisation of covariance Filter Scheme.
 *  Implementation of a 'Square-root' linearised kalman filter
 * 
 * Bierman's UD factorisatised update algorithm using Agee-Turner UdU' factorisation rank 1 update
 * Thornton's MWG-S factorisation predict algorithm
 * References
 * [1] "Factorisation Methods for Discrete Sequential Estimation" Gerald J. Bierman ISBN 0-12-097350-2
 * [2] "Kalman Filtering, Theory and Practice", Mohinder S. Grewal, Angus P. Andrews ISBN 0-13-211335-X
 *
 * A initial observation size may also be specified for efficiency.
 * 
 * The filter is operated by performing a
 *  predict, observe
 * cycle defined by the base class
 */
#include "bayesFlt.hpp"

/* Filter namespace */
namespace Bayesian_filter
{

class UD_sequential_observe_model : public Linrz_uncorrelated_observe_model
{
public:
	UD_sequential_observe_model (std::size_t x_size, std::size_t z_size) :
		Linrz_uncorrelated_observe_model(x_size, z_size), Hx_o(x_size)
	{}
	virtual const FM::Vec& ho (const FM::Vec& x, const std::size_t o) = 0;
	/* Supplied model (h) for observation using state x, z allows normalisation and model variation
	   Fast model of a single element (o) in observation model
	   Precondition: Hx_o is conformantly dimensioned
	   Postcondition:
		z(k|k-1) = h(x(k|k-1)
		Hx_o(x(k-1|k-1) = Jacobian of h with respect to state x (row o)
	*/
	FM::Vec Hx_o;
};


class UD_scheme : public Linrz_kalman_filter
{
private:
	std::size_t q_max;	// Maxiumum size allocated for noise model, constructed before UD
public:
	FM::Matrix UD;	// UDU factorisation of X with D on diagonal
						// Lower triangle used as workspace
	FM::Vec s;		// Innovation
	FM::Vec Sd;		// Innovation Covariance 

	UD_scheme (std::size_t x_size, std::size_t q_maxsize, std::size_t z_initialsize = 0);
	UD_scheme& operator= (const UD_scheme&);
	// Optimise copy assignment to only copy filter state

	void init ();
	void update ();
	Float predict (Linrz_predict_model& f);

	Float observe (Linrz_correlated_observe_model& h, const FM::Vec& z);
	/* No solution for Correlated noise and Linrz model */
	Float observe (Linrz_uncorrelated_observe_model& h, const FM::Vec& z);
	/* General observe */

	Float observe (Linear_correlated_observe_model& h, const FM::Vec& z);
	/* Special Linear observe for correlated Z, fast Z decorrelation */
	Float observe (UD_sequential_observe_model& h, const FM::Vec& z);
	/* Special Linrz observe using fast sequential model */

protected:
	Float predictGq (const FM::Matrix& Fx, const FM::Matrix& G, const FM::Vec& q);
	FM::Vec d, dv, v;	// predictGQ temporaries
	Float observeUD (FM::Vec& gain, Float& alpha, const FM::Vec& h, const Float r);
	FM::Vec a, b;		// observeUD temporaries
						// Observation temporaies
	void observe_size (std::size_t z_size);
	std::size_t last_z_size;
	FM::Vec h1;				// Single Observation model
	FM::Vec w;				// Single Gain
	FM::Vec znorm;			// Normalised innovation
	FM::Vec zpdecol;		// Decorrelated zp
	FM::Matrix Gz;			// Z coupling
	FM::Matrix GIHx;		// Modified Model for linear decorrelation
};


}//namespace
#endif
