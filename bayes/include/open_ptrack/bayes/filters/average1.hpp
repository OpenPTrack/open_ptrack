#ifndef _BAYES_FILTER_AVERAGE1
#define _BAYES_FILTER_AVERAGE1

/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id: average1.hpp 634 2010-08-15 16:39:44Z mistevens $
 */
 
/*
 * Predefined filter: Average1_filter
 *  A single state averager
 */

/* Filter namespace */
namespace Bayesian_filter
{

template <class Filter_base>
class Average1_filter
{
	Filter_base ksf;		// Kalman State Filter
	
	typedef typename Filter_base::Float Float;
	class Cpredict : public Linear_predict_model
	// Constant predict model
	{
	public:
		Cpredict(Float qq) : Linear_predict_model(1, 1)
		{
			Fx(0,0) = 1.;
			q[0] = qq;
			G(0,0) = 1.;
		}
	};

	class Cobserve : public Linear_correlated_observe_model
	// Constant observe model
	{
	public:
		Cobserve(Float ZZ) : Linear_correlated_observe_model(1,1)
		{
			Hx(0,0) = 1.;
			Z(0,0) = ZZ;
		}
	};

public:
	Average1_filter (Float iQ, Float iZ, Float z);
	Average1_filter (Float iQ, Float iZ);
	Float observe (Float zz);
	operator Float () const
	/* Returns filtered estimate
	 */
	{	if (!bInit)
			ksf.error (Logic_exception("Average1 not init"));
		return ksf.x[0];
	}

private:
	Cpredict f;
	Cobserve h;
	
	bool bInit;
	FM::Vec z;
};



template <typename Filter_base>
Average1_filter<Filter_base>::Average1_filter (Float iQ, Float iZ, Float zz) :
	ksf(1), f(iQ), h(iZ), z(1)
/* Initialise noises and set sizes
 * include first observation zz */
{
	bInit = false;
	
	observe (zz);
}

template <typename Filter_base>
Average1_filter<Filter_base>::Average1_filter (Float iQ, Float iZ) :
	ksf(1), f(iQ), h(iZ), z(1)
// Initialise noises and set sizes
{
	bInit = false;
}

template <typename Filter_base>
typename Average1_filter<Filter_base>::Float Average1_filter<Filter_base>::observe(Float zz)
/* Observe z, first call set initial state to z
 * Returns filtered estimate
 */
{
	z[0] = zz;

	if (!bInit) {
		ksf.init_kalman (z, h.Z);
		bInit = true;
	}

	ksf.predict(f);
	ksf.observe(h, z);
	ksf.update ();

	return ksf.x[0];
}

}//namespace
#endif
