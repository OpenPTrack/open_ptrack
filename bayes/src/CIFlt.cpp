/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id: CIFlt.cpp 634 2010-08-15 16:39:44Z mistevens $
 */

/*
 * Covariance Intersection Filter.
 * TODO: Implement useful Omega based on iterative optimization algorithm from the authors of reference [1]
 */
#include "CIFlt.hpp"
#include "matSup.hpp"
#include "models.hpp"

/* Filter namespace */
namespace Bayesian_filter
{
	using namespace Bayesian_filter_matrix;


CI_scheme::CI_scheme (std::size_t x_size, std::size_t z_initialsize) :
	Kalman_state_filter(x_size),
	S(Empty), SI(Empty)
/* Initialise filter and set the size of things we know about
 */
{
	last_z_size = 0;	// Matrices conform to z_initialsize, they are left Empty if z_initialsize==0
	observe_size (z_initialsize);
}

CI_scheme& CI_scheme::operator= (const CI_scheme& a)
/* Optimise copy assignment to only copy filter state
 * Precond: matrix size conformance
 */
{
	Kalman_state_filter::operator=(a);
	return *this;
}


void CI_scheme::init ()
{
						// Postconditions
	if (!isPSD (X))
		error (Numeric_exception("Initial X not PSD"));
}

void CI_scheme::update ()
{
	// Nothing to do, implicit in observation
}

Bayes_base::Float
 CI_scheme::predict (Linrz_predict_model& f)
{
	x = f.f(x);			// Extended Kalman state predict is f(x) directly
						// Predict state covariance
	RowMatrix tempX(f.Fx.size1(), X.size2());
	noalias(X) = prod_SPD(f.Fx,X, tempX);
	noalias(X) += prod_SPD(f.G, f.q, tempX);

	return 1;
}

void CI_scheme::observe_size (std::size_t z_size)
/* Optimised dynamic observation sizing
 */
{
	if (z_size != last_z_size) {
		last_z_size = z_size;

		S.resize(z_size,z_size, false);
		SI.resize(z_size,z_size, false);
	}
}


Bayes_base::Float
 CI_scheme::observe_innovation (Linrz_uncorrelated_observe_model& h, const FM::Vec& s)
/* Iterated Extended Kalman Filter
 * Bar-Shalom and Fortmann p.119 (full scheme)
 * A hard limit is placed on the iterations whatever the
 * the normal terminal condition is to guarantee termination
 * Uncorrelated noise
 */
{
						// ISSUE: Implement simplified uncorrelated noise equations
	std::size_t z_size = s.size();
	SymMatrix Z(z_size,z_size);

	Adapted_Linrz_correlated_observe_model hh(h);
	return observe_innovation (hh, s);
}


Bayes_base::Float
 CI_scheme::observe_innovation (Linrz_correlated_observe_model& h, const FM::Vec& s)
/* Correlated innovation observe
 */
{
	const Float one = 1;
						// size consistency, z to model
	if (s.size() != h.Z.size1())
		error (Logic_exception("observation and model size inconsistent"));
	observe_size (s.size());	// dynamic sizing

						// Linear conditioning for omega
	SymMatrix invZ(h.Z.size1(),h.Z.size2());
	Float rcond = UdUinversePD (invZ, h.Z);
	rclimit.check_PSD(rcond, "Z not PSD in observe");

	Matrix HTinvZ (prod(trans(h.Hx), invZ));
	SymMatrix HTinvZH (prod(HTinvZ, h.Hx));

	SymMatrix invX(X.size1(),X.size2());
	rcond = UdUinversePD (invX, X);
	rclimit.check_PD(rcond, "X not PD in observe");


						// find omega
	Float omega = Omega(invX, HTinvZH, X);

						// calculate predicted innovation
	Matrix XHT (prod(X, trans(h.Hx)));
	Matrix HXHT (prod(h.Hx, XHT));
	S = HXHT * (one-omega) + h.Z * omega;

						// inverse innovation covariance
	rcond = UdUinversePD (SI, S);
	rclimit.check_PD(rcond, "S not PD in observe");

	Matrix K (prod(XHT*(one-omega), SI));

						// state update
	noalias(x) += prod(K, s);
						// inverse covariance
	invX *= omega;						
	noalias(invX) += HTinvZH*(one-omega);
						// covariance
	rcond = UdUinversePD (X, invX);
	rclimit.check_PD(rcond, "inverse covariance not PD in observe");
	return rcond;
}


}//namespace
