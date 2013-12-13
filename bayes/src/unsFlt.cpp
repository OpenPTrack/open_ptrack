/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id$
 */

/*
 * Unscented Filter.
 */
#include "unsFlt.hpp"
#include "matSup.hpp"
#include "models.hpp"
#include <cmath>


/* Filter namespace */
namespace Bayesian_filter
{
	using namespace Bayesian_filter_matrix;


Unscented_scheme::Unscented_scheme (std::size_t x_size, std::size_t z_initialsize) :
		Kalman_state_filter(x_size), Functional_filter(),
		XX(x_size, 2*x_size+1),
		s(Empty), S(Empty), SI(Empty),
		fXX(x_size, 2*x_size+1)
/* Initialise filter and set the size of things we know about
 */
{
	Unscented_scheme::x_size = x_size;
	Unscented_scheme::XX_size = 2*x_size+1;
	last_z_size = 0;	// Matrices conform to z_initialsize, they are left Empty if z_initialsize==0
	observe_size (z_initialsize);
}

Unscented_scheme& Unscented_scheme::operator= (const Unscented_scheme& a)
/* Optimise copy assignment to only copy filter state
 * Precond: matrix size conformance
 */
{
	Kalman_state_filter::operator=(a);
	XX = a.XX;
	return *this;
}

void Unscented_scheme::unscented (FM::ColMatrix& XX, const FM::Vec& x, const FM::SymMatrix& X, Float scale)
/*
 * Generate the Unscented point representing a distribution
 * Fails if scale is negative
 */
{
	UTriMatrix Sigma(x_size,x_size);

						// Get a upper Cholesky factorisation
	Float rcond = UCfactor(Sigma, X);
	rclimit.check_PSD(rcond, "X not PSD");
	Sigma *= std::sqrt(scale);

						// Generate XX with the same sample Mean and Covariance as before
	column(XX,0) = x;

	for (std::size_t c = 0; c < x_size; ++c) {
		UTriMatrix::Column SigmaCol = column(Sigma,c);
		column(XX,c+1) .assign (x  + SigmaCol);
		column(XX,x_size+c+1) .assign (x - SigmaCol);
	}
}

Unscented_scheme::Float Unscented_scheme::predict_Kappa (std::size_t size) const
// Default Kappa for predict: state augmented with predict noise
{
	// Use the rule to minimise mean squared error of 4 order term
	return Float(3-signed(size));
}

Unscented_scheme::Float Unscented_scheme::observe_Kappa (std::size_t size) const
// Default Kappa for observation: state on its own
{
	// Use the rule to minimise mean squared error of 4 order term
	return Float(3-signed(size));
}

void Unscented_scheme::init ()
/* Initialise state
 *  Pre : x,X
 *  Post: x,X is PSD
 */
{
						// Postconditions
	if (!isPSD (X))
		error (Numeric_exception("Initial X not PSD"));
}

void Unscented_scheme::init_XX ()
/* Initialise from Unscented state
 *  Pre : XX, kappa
 *  Post: x,X is PSD
 */
{
	Float x_kappa = Float(x_size) + kappa;
						// Mean of predicted distribution: x
	noalias(x) = column(fXX,0) * kappa;
	for (std::size_t i = 1; i < XX_size; ++i) {
		noalias(x) += column(fXX,i) / Float(2); // ISSUE uBlas may not be able to promote integer 2
	}
	x /= x_kappa;
						// Covariance of distribution: X
							// Subtract mean from each point in fXX
	for (std::size_t i = 0; i < XX_size; ++i) {
		column(fXX,i).minus_assign (x);
	}
							// Center point, premult here by 2 for efficiency
    {
		ColMatrix::Column fXX0 = column(fXX,0);
		noalias(X) = FM::outer_prod(fXX0, fXX0);
		X *= 2*kappa;
	}
							// Remaining Unscented points
	for (std::size_t i = 1; i < XX_size; ++i) {
		ColMatrix::Column fXXi = column(fXX,i);
		noalias(X) += FM::outer_prod(fXXi, fXXi);
	}
	X /= 2*x_kappa;
}

void Unscented_scheme::update ()
/* Update state
 *  Pre : x,X
 *  Post: x,X
 */
{
}

void Unscented_scheme::update_XX (Float kappa)
/* Update Unscented state
 *  Pre : x,X
 *  Post: x,X, XX, kappa
 */
{
	Unscented_scheme::kappa = kappa;
	Float x_kappa = Float(x_size) + kappa;
	unscented (XX, x, X, x_kappa);
}


// ISSUE GCC2.95 cannot link if these are locally defined in member function
// Move them back into member functions for standard compilers
namespace {
	class Adapted_zero_model : public Unscented_predict_model
	{
	public:
		Adapted_zero_model(Functional_predict_model& fm) :
			Unscented_predict_model(0),
			fmodel(fm), zeroQ(0,0)
		{}
		const Vec& f(const Vec& x) const
		{
			return fmodel.fx(x);
		}
		const SymMatrix& Q(const FM::Vec& /*x*/) const
		{
			return zeroQ;
		}
	private:
		Functional_predict_model& fmodel;
		SymMatrix zeroQ;
	};

	class Adapted_model : public Unscented_predict_model
	{
	public:
		Adapted_model(Additive_predict_model& am) :
			Unscented_predict_model(am.G.size1()),
			amodel(am), QGqG(am.G.size1(),am.G.size1())		// Q gets size from GqG'
		{
			RowMatrix temp (am.G.size1(), am.G.size1());
			noalias(QGqG) = prod_SPD(am.G, am.q, temp);
		}
		const Vec& f(const Vec& x) const
		{
			return amodel.f(x);
		}
		const SymMatrix& Q(const FM::Vec& /*x*/) const
		{
			return QGqG;
		}
	private:
		Additive_predict_model& amodel;
		mutable SymMatrix QGqG;
	};
}//namespace


void Unscented_scheme::predict (Functional_predict_model& f)
/* Adapt model by creating an Unscented predict with zero noise
 * ISSUE: A simple specialisation is possible, rather then this adapted implementation
 */
{
	Adapted_zero_model adaptedmodel(f);
	predict (adaptedmodel);
}


void Unscented_scheme::predict (Additive_predict_model& f)
/* Adapt model by creating an Unscented predict with additive noise
 *  Computes noise covariance Q = GqG'
 */
{
	Adapted_model adaptedmodel(f);
	predict (adaptedmodel);
}


void Unscented_scheme::predict (Unscented_predict_model& f)
/* Predict forward
 *  Pre : x,X
 *  Post: x,X is PSD
 * Implementation uses specific model for fast Unscented computation
 */
{
	const std::size_t XX_size = XX.size2();

						// Create Unscented distribution
	kappa = predict_Kappa(x_size);
	Float x_kappa = Float(x_size) + kappa;
	unscented (XX, x, X, x_kappa);

						// Predict points of XX using supplied predict model
							// State covariance
	for (std::size_t i = 0; i < XX_size; ++i) {
		column(fXX,i).assign (f.f( column(XX,i) ));
	}

	init_XX ();
						// Additive Noise Prediction, computed about center point
	noalias(X) += f.Q( column(fXX,0) );
}


void Unscented_scheme::observe_size (std::size_t z_size)
/* Optimised dynamic observation sizing
 */
{
	if (z_size != last_z_size) {
		last_z_size = z_size;

		s.resize(z_size, false);
		S.resize(z_size,z_size, false);
		SI.resize(z_size,z_size, false);
	}
}


Bayes_base::Float Unscented_scheme::observe (Uncorrelated_additive_observe_model& h, const FM::Vec& z)
/* Observation fusion
 *  Pre : x,X
 *  Post: x,X is PSD
 *
 * Uncorrelated noise
 * ISSUE: Simplified implementation using uncorrelated noise equations
 */
{
	Adapted_Correlated_additive_observe_model hh(h);
	return observe (hh, z);
}


Bayes_base::Float Unscented_scheme::observe (Correlated_additive_observe_model& h, const FM::Vec& z)
/* Observation fusion
 *  Pre : x,X
 *  Post: x,X is PSD
 */
{
	std::size_t z_size = z.size();
	ColMatrix zXX (z_size, 2*x_size+1);
	Vec zp(z_size);
	SymMatrix Xzz(z_size,z_size);
	Matrix Xxz(x_size,z_size);
	Matrix W(x_size,z_size);

	observe_size (z.size());	// Dynamic sizing

						// Create Unscented distribution
	kappa = observe_Kappa(x_size);
	Float x_kappa = Float(x_size) + kappa;
	unscented (XX, x, X, x_kappa);

						// Predict points of XX using supplied observation model
	{
		Vec zXXi(z_size), zXX0(z_size);
		zXX0 = h.h( column(XX,0) );
		column(zXX,0) = zXX0;
		for (std::size_t i = 1; i < XX.size2(); ++i) {
			zXXi = h.h( column(XX,i) );
						// Normalise relative to zXX0
			h.normalise (zXXi, zXX0);
			column(zXX,i) = zXXi;
		}
	}

						// Mean of predicted distribution: zp
	noalias(zp) = column(zXX,0) * kappa;
	for (std::size_t i = 1; i < zXX.size2(); ++i) {
		noalias(zp) += column(zXX,i) / Float(2); // ISSUE uBlas may not be able to promote integer 2
	}
	zp /= x_kappa;

						// Covariance of observation predict: Xzz
							// Subtract mean from each point in zXX
	for (std::size_t i = 0; i < XX_size; ++i) {
		column(zXX,i).minus_assign (zp);
	}
							// Center point, premult here by 2 for efficiency
	{
		ColMatrix::Column zXX0 = column(zXX,0);
		noalias(Xzz) = FM::outer_prod(zXX0, zXX0);
		Xzz *= 2*kappa;
	}
							// Remaining Unscented points
	for (std::size_t i = 1; i < zXX.size2(); ++i) {
		ColMatrix::Column zXXi = column(zXX,i);
		noalias(Xzz) += FM::outer_prod(zXXi, zXXi);
	}
	Xzz /= 2*x_kappa;

						// Correlation of state with observation: Xxz
							// Center point, premult here by 2 for efficiency
	{
		noalias(Xxz) = FM::outer_prod(column(XX,0) - x, column(zXX,0));
		Xxz *= 2*kappa;
	}
							// Remaining Unscented points
	for (std::size_t i = 1; i < zXX.size2(); ++i) {
		noalias(Xxz) += FM::outer_prod(column(XX,i) - x, column(zXX,i));
	}
	Xxz /= 2* (Float(x_size) + kappa);

						// Innovation covariance
	S = Xzz;
	noalias(S) += h.Z;
						// Inverse innovation covariance
	Float rcond = UdUinversePD (SI, S);
	rclimit.check_PD(rcond, "S not PD in observe");
						// Kalman gain
	noalias(W) = prod(Xxz,SI);

						// Normalised innovation
	h.normalise(s = z, zp);
	noalias(s) -= zp;

						// Filter update
	noalias(x) += prod(W,s);
	RowMatrix WStemp(W.size1(), S.size2());
	noalias(X) -= prod_SPD(W,S, WStemp);

	return rcond;
}


}//namespace
