#ifndef _BAYES_FILTER_INDIRECT
#define _BAYES_FILTER_INDIRECT

/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id: indirect.hpp 634 2010-08-15 16:39:44Z mistevens $
 */
 
/*
 * Indirect Filter Adaptor
 *  Hides the details of the indirect operation of the filter
 *  The error filter uses the same linear models as the direct filter,
 *  observation error computation (subtraction) is linear!
 */

/* Filter namespace */
namespace Bayesian_filter
{


template <typename Error_base>
class Indirect_state_filter : public State_filter {
/*
 * Indirect state filter
 *  Estimates state using an associated observation error filter
 */
public:
	Indirect_state_filter (Error_base& error_filter)
		: State_filter(error_filter.x.size()), direct(error_filter)
	{	// Construct and zero initial error filter
		direct.x.clear();
	}

	template <typename P_model>
	void predict (P_model& f)
	{
		x = f.f(x);
		direct.predict(f);				// May be optimised for linear f as x = 0
	};

	template <typename O_model>
	void observe (O_model& h, const FM::Vec& z)
	{
				// Observe error (explict temporary)
		FM::Vec z_error(z.size());
		z_error = h.h(x);
		z_error -= z;
		direct.observe (h, z_error);
				// Update State estimate with error
		x -= direct.x;
				// Reset the error
		direct.x.clear();
	}

	template <typename O_model>
	void observe_error (O_model& h, const FM::Vec& z_error)
	{
		direct.observe (h, z_error);
				// Update State estimate with error
		x -= direct.x;
				// Reset the error
		direct.x.clear();
	}

	void update ()
	/* Update filters state
	     Updates x(k|k)
	*/
	{}

private:
	Error_base& direct;
};


template <typename Error_base>
class Indirect_kalman_filter : public Kalman_state_filter {
/*
 * Indirect kalman filter
 *  Estimates state using an associated observation error filter
 */
public:
	Indirect_kalman_filter (Error_base& error_filter)
		: Kalman_state_filter(error_filter.x.size()), direct(error_filter)
	{	
	}

	void init ()
	/* Initialise from state and state covariance
	*/
	{
		direct.x.clear();				// Zero initial error
		direct.X = X;
		direct.init();
	}

	template <typename P_model>
	void predict (P_model& f)
	{
		x = f.f(x);
		direct.predict(f);				// May be optimised for linear f as x = 0
	};

	template <typename O_model>
	void observe (O_model& h, const FM::Vec& z)
	{
				// Observe error (explict temporary)
		FM::Vec z_error(z.size());
		z_error = h.h(x);
		z_error -= z;
		direct.observe (h, z_error);
		direct.update();
				// Update State estimate with error
		x -= direct.x;
				// Reset the error
		direct.x.clear();
		direct.init ();
	}

	template <typename O_model>
	void observe_error (O_model& h, const FM::Vec& z_error)
	{
		direct.observe (h, z_error);
				// Update State estimate with error
		x -= direct.x;
				// Reset the error
		direct.clear();
	}

	void update ()
	/* Update filters state
	     Updates x(k|k)
	*/
	{
		direct.update();
		X = direct.X;
	}

private:
	Error_base& direct;
};


}//namespace
#endif
