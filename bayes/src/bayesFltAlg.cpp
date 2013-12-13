/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id$
 */
 
/*
 * Implement models.hpp :
 */
#include "bayesFlt.hpp"
#include "matSup.hpp"
#include "models.hpp"


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

Simple_additive_predict_model::Simple_additive_predict_model (State_function f_init, const FM::Matrix& G_init, const FM::Vec& q_init) :
// Precondition: G, q are conformantly dimensioned (not checked)
	Additive_predict_model (G_init.size1(), q_init.size()),
	ff(f_init)
{
	G = G_init;
	q = q_init;
}

Simple_linrz_predict_model::Simple_linrz_predict_model (State_function f_init, const FM::Matrix& Fx_init, const FM::Matrix& G_init, const FM::Vec& q_init) :
// Precondition: Fx, G, q are conformantly dimensioned (not checked)
	Linrz_predict_model (Fx_init.size1(), q_init.size()),
	ff(f_init)
{
	Fx = Fx_init;
	G = G_init;
	q = q_init;
}

Simple_linear_predict_model::Simple_linear_predict_model (const FM::Matrix& Fx_init, const FM::Matrix& G_init, const FM::Vec& q_init) :
// Precondition: Fx, q and G are conformantly dimensioned (not checked)
	Linear_predict_model (Fx_init.size1(), q_init.size())
{
	Fx = Fx_init;
	G = G_init;
	q = q_init;
}

Simple_linrz_correlated_observe_model::Simple_linrz_correlated_observe_model (State_function f_init, const FM::Matrix& Hx_init, const FM::SymMatrix& Z_init) :
	Linrz_correlated_observe_model (Hx_init.size2(), Hx_init.size1()),
	ff(f_init)
/* Linrz observe model initialised from function and model matrices
   Precondition:
   Hx, Z are conformantly dimensioned (not checked)
 */
{
	Hx = Hx_init;
	Z = Z_init;
};

Simple_linrz_uncorrelated_observe_model::Simple_linrz_uncorrelated_observe_model (State_function f_init, const FM::Matrix& Hx_init, const FM::Vec& Zv_init) :
	Linrz_uncorrelated_observe_model (Hx_init.size2(), Hx_init.size1()),
	ff(f_init)
/* Linrz observe model initialised from function and model matrices
   Precondition:
   Hx, Z are conformantly dimensioned (not checked)
 */
{
	Hx = Hx_init;
	Zv = Zv_init;
};

Simple_linear_correlated_observe_model::Simple_linear_correlated_observe_model (const FM::Matrix& Hx_init, const FM::SymMatrix& Z_init) :
	Linear_correlated_observe_model (Hx_init.size2(), Hx_init.size1())
/* Linear observe model initialised from model matrices
   Precondition:
   Hx, Z are conformantly dimensioned (not checked)
 */
{
	Hx = Hx_init;
	Z = Z_init;
};

Simple_linear_uncorrelated_observe_model::Simple_linear_uncorrelated_observe_model (const FM::Matrix& Hx_init, const FM::Vec& Zv_init) :
	Linear_uncorrelated_observe_model (Hx_init.size2(), Hx_init.size1())
/* Linear observe model initialised from model matrices
   Precondition:
   Hx, Z are conformantly dimensioned (not checked)
 */
{
	Hx = Hx_init;
	Zv = Zv_init;
};



Bayes_base::Float
 General_LzUnAd_observe_model::Likelihood_uncorrelated::L(const Uncorrelated_additive_observe_model& model, const FM::Vec& z, const FM::Vec& zp) const
/* Definition of likelihood given an additive Gaussian observation model:
 *  p(z|x) = exp(-0.5*(z-h(x))'*inv(Z)*(z-h(x))) / sqrt(2pi^nz*det(Z));
 *  L(x) the the Likelihood L(x) doesn't depend on / sqrt(2pi^nz) for constant z size
 * Precond: Observation Information: z,Zv_inv,detZterm
 */
{
	if (!zset)
		Bayes_base::error (Logic_exception("General_observe_model used without Lz set"));
					// Normalised innovation
	zInnov = z;
	model.normalise (zInnov, zp);
	FM::noalias(zInnov) -= zp;

	// Likelihood w of observation z given particular state xi is true state
	// The state, xi, defines a predicted observation with a Gaussian
	// distribution with variance Zd. Thus, the likelihood can be determined directly from the Gaussian

	FM::Vec::iterator zi = zInnov.begin(), zi_end = zInnov.end();
	for (; zi != zi_end; ++zi) {
		*zi *= *zi;
	}
	Float logL = FM::inner_prod(zInnov, Zv_inv);

	using namespace std;
	return exp(Float(-0.5)*(logL + logdetZ));
}

void General_LzUnAd_observe_model::Likelihood_uncorrelated::Lz (const Uncorrelated_additive_observe_model& model)
/* Set the observation zz and Zv about which to evaluate the Likelihood function
 * Postcond: Observation Information: z,Zv_inv,detZterm
 */
{
	zset = true;
					// Compute inverse of Zv and its reciprocal condition number
	Float rcond = FM::UdUrcond(model.Zv);
	model.rclimit.check_PD(rcond, "Z not PD in observe");

	Bayes_base::Float detZ = 1;
	
	for (FM::Vec::const_iterator zi = model.Zv.begin(), zi_end = model.Zv.end(); zi != zi_end; ++zi) {
		detZ *= *zi;
		Zv_inv[zi.index()] = 1 / (*zi);		// Protected from /0 by rcond check
	}
	using namespace std;
	logdetZ = log(detZ);		// Protected from ln(0) by rcond check
}

Bayes_base::Float
 General_LzCoAd_observe_model::Likelihood_correlated::L(const Correlated_additive_observe_model& model, const FM::Vec& z, const FM::Vec& zp) const
/* Definition of likelihood given an additive Gaussian observation model:
 *  p(z|x) = exp(-0.5*(z-h(x))'*inv(Z)*(z-h(x))) / sqrt(2pi^nz*det(Z));
 *  L(x) the the Likelihood L(x) doesn't depend on / sqrt(2pi^nz) for constant z size
 * Precond: Observation Information: z,Z_inv,detZterm
 */
{
	if (!zset)
		Bayes_base::error (Logic_exception ("General_observe_model used without Lz set"));
					// Normalised innovation
	zInnov = z;
	model.normalise (zInnov, zp);
	FM::noalias(zInnov) -= zp;

	Float logL = scaled_vector_square(zInnov, Z_inv);
	using namespace std;
	return exp(Float(-0.5)*(logL + logdetZ));
}

void General_LzCoAd_observe_model::Likelihood_correlated::Lz (const Correlated_additive_observe_model& model)
/* Set the observation zz and Z about which to evaluate the Likelihood function
 * Postcond: Observation Information: z,Z_inv,detZterm
 */
{
	zset = true;
						// Compute inverse of Z and its reciprocal condition number
	Float detZ;
	Float rcond = FM::UdUinversePD (Z_inv, detZ, model.Z);
	model.rclimit.check_PD(rcond, "Z not PD in observe");
						// detZ > 0 as Z PD
	using namespace std;
	logdetZ = log(detZ);
}


Bayes_base::Float
 General_LzCoAd_observe_model::Likelihood_correlated::scaled_vector_square(const FM::Vec& v, const FM::SymMatrix& V)
/* Compute covariance scaled square inner product of a Vector: v'*V*v
 */
{
	return FM::inner_prod(v, FM::prod(V,v));
}


Adapted_Correlated_additive_observe_model::Adapted_Correlated_additive_observe_model (Uncorrelated_additive_observe_model& adapt) :
		Correlated_additive_observe_model(adapt.Zv.size()),
		unc(adapt)
{
	Z.clear();
	for (std::size_t i = 0; i < unc.Zv.size(); ++i)
		Z(i,i) = Float(unc.Zv[i]);	// ISSUE mixed type proxy assignment
}

Adapted_Linrz_correlated_observe_model::Adapted_Linrz_correlated_observe_model (Linrz_uncorrelated_observe_model& adapt) :
		Linrz_correlated_observe_model(adapt.Hx.size2(), adapt.Hx.size1()),
		unc(adapt)
{
	Hx = unc.Hx;
	Z.clear();
	for (std::size_t i = 0; i < unc.Zv.size(); ++i)
		Z(i,i) = Float(unc.Zv[i]);	// ISSUE mixed type proxy assignment
}


}//namespace
