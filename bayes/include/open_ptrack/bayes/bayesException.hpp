#ifndef _BAYES_FILTER_EXCEPTION
#define _BAYES_FILTER_EXCEPTION

/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id: bayesException.hpp 634 2010-08-15 16:39:44Z mistevens $
 */

/*
 * Exception types: Exception heirarchy for Bayesian filtering 
 */
 
// Common headers required for declerations
#include <exception>

/* Filter namespace */
namespace Bayesian_filter
{


class Filter_exception : virtual public std::exception
/*
 *	Base class for all exception produced by filter heirachy
 */
{
public:
	const char *what() const throw()
	{	return error_description;
	}
protected:
	Filter_exception (const char* description)
	{	error_description = description;
	}
private:
	const char* error_description;
};

class Logic_exception : virtual public Filter_exception
/*
 * Logic Exception
 */
{
public:
	Logic_exception (const char* description) :
		Filter_exception (description)
	{}
};

class Numeric_exception : virtual public Filter_exception
/*
 * Numeric Exception
 */
{
public:
	Numeric_exception (const char* description) :
		Filter_exception (description)
	{}
};


}//namespace
#endif
