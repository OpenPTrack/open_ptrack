/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $ID$
 */

/*
 * Iterated Covariance Filter.
 */
#include "itrFlt.hpp"
#include "matSup.hpp"
#include "models.hpp"

/* Filter namespace */
namespace Bayesian_filter
{
	using namespace Bayesian_filter_matrix;


bool Counted_iterated_terminator::term_or_relinearize (const Iterated_covariance_scheme& f)
{
	--i;
	if (i != 0)
		m.relinearise (f.x);
	return (--i) == 0;
}


Iterated_covariance_scheme::Iterated_covariance_scheme(std::size_t x_size, std::size_t z_initialsize) :
		Kalman_state_filter(x_size),
		S(Empty), SI(Empty),
		tempX(x_size,x_size),
		s(Empty), HxT(Empty)
/* Initialise filter and set the size of things we know about
 */
{
	last_z_size = 0;	// Matrices conform to z_initialsize, they are left Empty if z_initialsize==0
	observe_size (z_initialsize);
}

Iterated_covariance_scheme&
 Iterated_covariance_scheme::operator= (const Iterated_covariance_scheme& a)
/* Optimise copy assignment to only copy filter state
 * Precond: matrix size conformance
 */
{
	Kalman_state_filter::operator=(a);
	return *this;
}

void Iterated_covariance_scheme::init ()
{
						// Postconditions
	if (!isPSD (X))
		error (Numeric_exception("Initial X not PSD"));
}

void Iterated_covariance_scheme::update ()
{
	// Nothing to do, implicit in observation
}

Bayes_base::Float
 Iterated_covariance_scheme::predict (Linrz_predict_model& f)
{
	x = f.f(x);			// Extended Kalman state predict is f(x) directly
						// Predict state covariance
	noalias(X) = prod_SPD(f.Fx,X, tempX);
	noalias(X) += prod_SPD(f.G, f.q, tempX);

	return 1;
}


void Iterated_covariance_scheme::observe_size (std::size_t z_size)
/* Optimised dynamic observation sizing
 */
{
	if (z_size != last_z_size) {
		last_z_size = z_size;

		s.resize(z_size, false);
		S.resize(z_size,z_size, false);
		SI.resize(z_size,z_size, false);
		HxT.resize(x.size(),z_size, false);
	}
}

Bayes_base::Float
 Iterated_covariance_scheme::observe (Linrz_uncorrelated_observe_model& h, Iterated_terminator& term, const FM::Vec& z)
/* Iterated Extended Kalman Filter
 * Bar-Shalom and Fortmann p.119 (full scheme)
 * A hard limit is placed on the iterations whatever the
 * the normal terminal condition is to guarantee termination
 * Uncorrelated noise
 */
{
						// ISSUE: Implement simplified uncorrelated noise equations
	Adapted_Linrz_correlated_observe_model hh(h);
	return observe (hh, term, z);
}

Bayes_base::Float
 Iterated_covariance_scheme::observe (Linrz_correlated_observe_model& h, Iterated_terminator& term, const FM::Vec& z)
/* Iterated Extended Kalman Filter
 * Bar-Shalom and Fortmann p.119 (full scheme)
 * A hard limit is placed on the iterations whatever the
 * the normal terminal condition is to guarantee termination
 * returned rcond is of S (or 1 if no iterations are performed)
 */
{
	std::size_t x_size = x.size();
	std::size_t z_size = z.size();
	SymMatrix ZI(z_size,z_size);
	Matrix HxT(x_size,z_size);

	Vec xpred = x;			// Initialise iteration
	SymMatrix Xpred = X;
							// Inverse predicted covariance
	SymMatrix XpredI(x_size,x_size);
	Float rcond = UdUinversePD (XpredI, Xpred);
	rclimit.check_PD(rcond, "Xpred not PD in observe");

							// Inverse observation covariance
	rcond = UdUinversePD (ZI, h.Z);
	rclimit.check_PD(rcond, "Z not PD in observe");
				
	RowMatrix HxXtemp(h.Hx.size1(),X.size2());
	RowMatrix temp1(x_size,x_size), temp2(x_size,z_size);
	SymMatrix temp3(x_size,x_size);
	Vec tempz(z_size);

	do {
							// Observation model, linearize about new x
		const Vec& zp = h.h(x);
		
		noalias(HxT) = trans(h.Hx);
							// Innovation
		h.normalise(s = z, zp);
		noalias(s) -= zp;
							// Innovation covariance
		noalias(S) = prod_SPD(h.Hx, Xpred, HxXtemp) + h.Z;
							// Inverse innovation covariance
		rcond = UdUinversePD (SI, S);
		rclimit.check_PD(rcond, "S not PD in observe");

							// Iterative observe
		noalias(temp3) = prod_SPD(HxT,SI, temp2);
		noalias(temp1) = prod(Xpred,temp3);
		noalias(X) = Xpred - prod(temp1,Xpred);

							// New state iteration
		noalias(temp2) = prod(X,HxT);
		noalias(temp1) = prod(X,XpredI);
		noalias(tempz) = prod(ZI,s);
		x += prod(temp2, tempz) - prod(temp1, (x - xpred));
	} while (!term.term_or_relinearize(*this));
	return rcond;
}

}//namespace
