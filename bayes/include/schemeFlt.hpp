#ifndef _BAYES_FILTER_SCHEME
#define _BAYES_FILTER_SCHEME

/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id: schemeFlt.hpp 634 2010-08-15 16:39:44Z mistevens $
 */

/*
 * Generic Filter
 *  Filter schemes vary in their constructor parameterisation
 *  Filter_scheme derives a generic filter with consistent constructor interface
 *
 *  Provides specialisations for all Bayesian_filter schemes
 */


/* Filter namespace */
namespace Bayesian_filter
{

template <class Scheme>
struct Filter_scheme : public Scheme
/*
 * A Generic Filter Scheme
 *  Class template to provide a consistent constructor interface
 *  Default for Kalman_state_filter
 */
{
	Filter_scheme(std::size_t x_size, std::size_t q_maxsize, std::size_t z_initialsize) :
		Kalman_state_filter (x_size), Scheme (x_size, z_initialsize)
	{}
};


// UD_filter specialisation
template <>
struct Filter_scheme<UD_scheme> : public UD_scheme
{
	Filter_scheme(std::size_t x_size, std::size_t q_maxsize, std::size_t z_initialsize) :
		Kalman_state_filter (x_size), UD_scheme (x_size, q_maxsize, z_initialsize)
	{}
};

// Information_scheme specialisation
template <>
struct Filter_scheme<Information_scheme> : public Information_scheme
{
	Filter_scheme(std::size_t x_size, std::size_t q_maxsize, std::size_t z_initialsize) :
		Kalman_state_filter (x_size), Information_state_filter (x_size),
		Information_scheme (x_size, z_initialsize)
	{}
};

// Information_root_info_scheme specialisation
template <>
struct Filter_scheme<Information_root_info_scheme> : public Information_root_info_scheme
{
	Filter_scheme(std::size_t x_size, std::size_t q_maxsize, std::size_t z_initialsize) :
		Kalman_state_filter (x_size), Information_state_filter (x_size),
		Information_root_info_scheme (x_size, z_initialsize)
	{}
};

// SIR_scheme specialisation, inconsistent constructor
template <>
struct Filter_scheme<SIR_scheme> : public SIR_scheme
{
	Filter_scheme(std::size_t x_size, std::size_t s_size, SIR_random& random_helper) :
		Sample_state_filter (x_size, s_size),
		SIR_scheme (x_size, s_size, random_helper)
	{}
};

// SIR_kalman_scheme specialisation, inconsistent constructor
template <>
struct Filter_scheme<SIR_kalman_scheme> : public SIR_kalman_scheme
{
	Filter_scheme(std::size_t x_size, std::size_t s_size, SIR_random& random_helper) :
		Sample_state_filter (x_size, s_size),
		Kalman_state_filter (x_size),
		SIR_kalman_scheme (x_size, s_size, random_helper)
	{}
};


}//namespace
#endif
