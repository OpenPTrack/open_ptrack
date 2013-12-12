#ifndef _BAYES_FILTER_MATRIX_SUPPORT
#define _BAYES_FILTER_MATRIX_SUPPORT

/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id: matSup.hpp 634 2010-08-15 16:39:44Z mistevens $
 */

/*
 * Matrix support functions for filter classes
 *  Members of the Bayesian_filter_matrix namespace are used in the
 *  interface of Bayesian_filter and for internal operations.
 *  Be aware! These functions and their implemenation are more likely to change
 *  then those in Bayesian_filter.
 */

/* Filter Matrix Namespace */
namespace Bayesian_filter_matrix
{


/*
 * Assertion support
 */
#ifndef NDEBUG
void assert_isPSD (const SymMatrix &M);
#else
inline void assert_isPSD (const SymMatrix &M) {}
#endif

/*
 * Local support functions
 */
bool isPSD (const SymMatrix &M);
bool isSymmetric (const Matrix &M);
void forceSymmetric (Matrix &M, bool bUpperToLower = false);

/*
 * UdU' and LdL' and UU' Cholesky Factorisation and function
 * Very important to manipulate PD and PSD matrices
 *
 * Return values:
 *  Many algorithms return a value_type which is a reciprocal condition number
 *  These values are documented for each algorithm and are important way to
 *  determine the validity of the results
 */
Vec::value_type UdUrcond (const Vec& d);
RowMatrix::value_type UdUrcond (const RowMatrix& UD);
RowMatrix::value_type UdUrcond (const RowMatrix& UD, std::size_t n);
UTriMatrix::value_type UCrcond (const UTriMatrix& UC);
SymMatrix::value_type UdUdet (const SymMatrix& UD);

// In-place factorisations
RowMatrix::value_type UdUfactor_variant1 (RowMatrix& M, std::size_t n);
RowMatrix::value_type UdUfactor_variant2 (RowMatrix& M, std::size_t n);
inline RowMatrix::value_type UdUfactor (RowMatrix& M, std::size_t n)
{	return UdUfactor_variant2(M,n);
}
LTriMatrix::value_type LdLfactor (LTriMatrix& M, std::size_t n);
UTriMatrix::value_type UCfactor (UTriMatrix& M, std::size_t n);

// Copy factorisations
RowMatrix::value_type UdUfactor (RowMatrix& UD, const SymMatrix& M);
LTriMatrix::value_type LdLfactor (LTriMatrix& LD, const SymMatrix& M);
UTriMatrix::value_type UCfactor (UTriMatrix& UC, const SymMatrix& M);

// Factor manipulations
bool UdUinverse (RowMatrix& UD);
bool UTinverse (UTriMatrix& U);
void UdUrecompose_transpose (RowMatrix& M);
void UdUrecompose (RowMatrix& M);
void UdUrecompose (SymMatrix& X, const RowMatrix& M);
void UdUfromUCholesky (RowMatrix& U);
void UdUseperate (RowMatrix& U, Vec& d, const RowMatrix& UD);
void Lzero (RowMatrix& M);
void Uzero (RowMatrix& M);

/*
 * Functions using UdU factorisation:
 *  inverse of Positive Definate matrix returning rcond
 */
SymMatrix::value_type UdUinversePDignoreInfinity (SymMatrix& M);
SymMatrix::value_type UdUinversePD (SymMatrix& M);
SymMatrix::value_type UdUinversePD (SymMatrix& M, SymMatrix::value_type& detM);
SymMatrix::value_type UdUinversePD (SymMatrix& MI, const SymMatrix& M);
SymMatrix::value_type UdUinversePD (SymMatrix& MI, SymMatrix::value_type& detM, const SymMatrix& M);


}//namespace

#endif
