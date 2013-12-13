/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id$
 */

/*
 * Matrix support functions for filter classes
 * Relies on Bayesian_filter::Bayes_filter for exception
 * thrown by internal matrix checks
 */
#include "bayesFlt.hpp"		// Exceptions required
#include "matSup.hpp"
#include <cassert>
#ifndef NDEBUG
#include "boost/numeric/ublas/io.hpp"
#endif

namespace {

template <class scalar>
inline scalar sqr(scalar x)
// Square 
{
	return x*x;
}
};//namespace


/* Filter Matrix Namespace */
namespace Bayesian_filter_matrix
{


#ifndef NDEBUG
void assert_isPSD (const SymMatrix &M)
/* Assert a Matrix is Positive Semi Definite via the algorithm in isPSD
 *  Requires 'cerr' and 'assert' to abort execution
 */
{
	bool bPSD = isPSD(M);
	if (!bPSD) {
		// Display Non PSD
		std::cerr.flags(std::ios::scientific); std::cerr.precision(17);
		std::cerr << M;
	}
	assert(bPSD);
}
#endif


bool isPSD (const SymMatrix &M)
/* Check a Matrix is both Symetric and Positive Semi Definite
 *  Creates a temporary copy
 *  
 * Numerics of Algorithm:
 *  Use UdUfactor and checks reciprocal condition number >=0
 *  Algorithms using UdUfactor will will therefore succeed if isPSD(M) is true
 *  Do NOT assume because isPSD is true that M will appear to be PSD to other numerical algorithms
 * Return:
 *  true iff M isSymetric and is PSD by the above algorithm
 */
{
	RowMatrix UD(M.size1(),M.size1());

	RowMatrix::value_type rcond = UdUfactor(UD, M);
	return rcond >= 0.;
}


bool isSymmetric (const Matrix &M)
/* Check a Symmetric Matrix really is Square and Symmetric
 * The later may be implied by the SymMatrix type
 * Implicitly also checks matrix is without IEC 559 NaN values as they are always !=
 * Return:
 *  true iff M is Square and Symmetric and without NaN 
 */
{
	// Check Square
	if (M.size1() != M.size2() ) {
		return false;
	}

	// Check equality of upper and lower
	bool bSym = true;
	std::size_t size = M.size1();
	for (std::size_t r = 0; r < size; ++r) {
		for (std::size_t c = 0; c <= r; ++c) {
			if( M(r,c) != M(c,r) ) {
				bSym = false;
			}
		}
	}
	return bSym;
}


void forceSymmetric (Matrix &M, bool bUpperToLower)
/* Force Matrix Symmetry
 *	Normally Copies lower triangle to upper or
 *   upper to lower triangle if specified
 */
{
	// Check Square
	if (M.size1() != M.size2() ) {
		using namespace Bayesian_filter;
		Bayes_base::error (Logic_exception ("Matrix is not square"));
	}

	std::size_t size = M.size1();

	if (bUpperToLower)
	{
		// Copy Lower to Upper
		for (std::size_t r = 1; r < size; ++r) {
			for (std::size_t c = 0; c < r; ++c) {
				M(c,r) = M(r,c);
			}
		}
	}
	else
	{
		// Copy Upper to Lower
		for (std::size_t r = 1; r < size; ++r) {
			for (std::size_t c = 0; c < r; ++c) {
				M(r,c) = M(c,r);
			}
		}
	}
}



}//namespace
