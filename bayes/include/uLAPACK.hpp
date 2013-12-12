/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id: uLAPACK.hpp 634 2010-08-15 16:39:44Z mistevens $
 */

/*
 * uBLAS to LAPACK Interface
 *  Very basic we only functions for information_root_filter supported
 */

/* Filter Matrix Namespace */
namespace Bayesian_filter_matrix
{
namespace LAPACK {

namespace rawLAPACK {
	// LAPACK routines as C callable functions
	extern "C" {

	void dgeqrf_(
			const int & m,
			const int & n,
			double da[],
			const int & lda,
			double dtau[],
			double dwork[],
			const int& ldwork,
			int& info);
	void sgeqrf_(
			const int & m,
			const int & n,
			float da[],
			const int & lda,
			float dtau[],
			float dwork[],
			const int& ldwork,
			int& info);

	void dgetrs_(
			const char& transa,
			const int& n,
			const int& nrhs,
			const double da[],
			const int& lda,
			int ipivot[],
			double db[],
			const int& ldb,
			int& info);
	void sgetrs_(
			const char& transa,
			const int& n,
			const int& nrhs,
			const float da[],
			const int& lda,
			int ipivot[],
			float db[],
			const int& ldb,
			int& info);

	void dgetrf_(
			const int& m,
			const int& n,
			double da[],
			const int& lda,
			int ipivot[],
			int& info);
	void sgetrf_(
			const int& m,
			const int& n,
			float da[],
			const int& lda,
			int ipivot[],
			int& info);

	}// extern "C"

	// Type overloads for C++
	inline void geqrf( const int & m, const int & n, double da[], const int & lda, double dtau[], double dwork[], const int& ldwork, int& info)
	{
		dgeqrf_(m,n,da,lda,dtau,dwork,ldwork,info);
	}
	inline void geqrf( const int & m, const int & n, float da[], const int & lda, float dtau[], float dwork[], const int& ldwork, int& info)
	{
		sgeqrf_(m,n,da,lda,dtau,dwork,ldwork,info);
	}
	inline void getrs( const char& transa, const int& n, const int& nrhs, const double da[], const int& lda, int ipivot[], double db[], const int& ldb, int& info)
	{
		dgetrs_(transa,n,nrhs,da,lda,ipivot,db,ldb,info);
	}
	inline void getrs( const char& transa, const int& n, const int& nrhs, const float da[], const int& lda, int ipivot[], float db[], const int& ldb, int& info)
	{
		sgetrs_(transa,n,nrhs,da,lda,ipivot,db,ldb,info);
	}
	inline void getrf( const int& m, const int& n, double da[], const int& lda, int ipivot[], int& info)
	{
		dgetrf_(m,n,da,lda,ipivot,info);
	}
	inline void getrf( const int& m, const int& n, float da[], const int& lda, int ipivot[], int& info)
	{
		sgetrf_(m,n,da,lda,ipivot,info);
	}
}// namespace rawLAPACK


/* Support types */
typedef boost::numeric::ublas::vector<int> pivot_t;
typedef Bayesian_filter_matrix::DenseVec vector_t;
typedef Bayesian_filter_matrix::DenseColMatrix matrix_t;

/* LAPACK Interface*/



// QR Factorization of a MxN General Matrix A.
//    a       (IN/OUT - matrix(M,N)) On entry, the coefficient matrix A. On exit , the upper triangle and diagonal is the min(M,N) by N upper triangular matrix R.  The lower triangle, together with the tau vector, is the orthogonal matrix Q as a product of min(M,N) elementary reflectors.
//    tau     (OUT - vector (min(M,N))) Vector of the same numerical type as A. The scalar factors of the elementary reflectors.
//    info    (OUT - int)
//   0   : function completed normally
//   < 0 : The ith argument, where i = abs(return value) had an illegal value.
int geqrf (matrix_t& a, vector_t& tau)
{
	int              _m = int(a.size1());
	int              _n = int(a.size2());
	int              _lda = int(a.size1());
	int              _info;

	// make_sure tau's size is greater than or equal to min(m,n)
	if (int(tau.size()) < (_n<_m ? _n : _m) )
		return -104;

	int ldwork = _n*_n;
	vector_t dwork(ldwork);
	rawLAPACK::geqrf (_m, _n, a.data().begin(), _lda, tau.data().begin(), dwork.data().begin(), ldwork, _info);

	return _info;
}

// LU factorization of a general matrix A.  
//    Computes an LU factorization of a general M-by-N matrix A using
//    partial pivoting with row interchanges. Factorization has the form
//    A = P*L*U.
//    a       (IN/OUT - matrix(M,N)) On entry, the coefficient matrix A to be factored. On exit, the factors L and U from the factorization A = P*L*U.
//    ipivot  (OUT - vector(min(M,N))) Integer vector. The row i of A was interchanged with row IPIV(i).
//    info    (OUT - int)
//   0   :  successful exit
//   < 0 :  If INFO = -i, then the i-th argument had an illegal value.
//   > 0 :  If INFO = i, then U(i,i) is exactly zero. The  factorization has been completed, but the factor U is exactly singular, and division by zero will occur if it is used to solve a system of equations.
int getrf (matrix_t& a, pivot_t& ipivot)
{
	matrix_t::value_type* _a = a.data().begin();
	int _m = int(a.size1());
	int _n = int(a.size2());
	int _lda = _m;	// minor size
	int _info;

	rawLAPACK::getrf (_m, _n,	_a, _lda, ipivot.data().begin(), _info);

	return _info;
}

// Solution to a system using LU factorization 
//   Solves a system of linear equations A*X = B with a general NxN
//   matrix A using the LU factorization computed by GETRF.
//   transa  (IN - char)  'T' for the transpose of A, 'N' otherwise.
//   a       (IN - matrix(M,N)) The factors L and U from the factorization A = P*L*U as computed by GETRF.
//   ipivot  (IN - vector(min(M,N))) Integer vector. The pivot indices from GETRF; row i of A was interchanged with row IPIV(i).
//   b       (IN/OUT - matrix(ldb,NRHS)) Matrix of same numerical type as A. On entry, the right hand side matrix B. On exit, the solution matrix X.
//
//   info    (OUT - int)
//   0   : function completed normally
//   < 0 : The ith argument, where i = abs(return value) had an illegal value.
//   > 0 : if INFO =  i,  U(i,i)  is  exactly  zero;  the  matrix is singular and its inverse could not be computed.
int getrs (char transa, matrix_t& a,
	    pivot_t& ipivot, matrix_t& b)
{
	matrix_t::value_type* _a = a.data().begin();
	int a_n = int(a.size1());
	int _lda = a_n;
	int p_n = int(ipivot.size());

	matrix_t::value_type* _b = b.data().begin();
	int b_n = int(b.size1());
	int _ldb = b_n;
	int _nrhs = int(b.size2()); /* B's size2 is the # of vectors on rhs */

	if (a_n != b_n) /*Test to see if AX=B has correct dimensions */
		return -101;
	if (p_n < a_n)     /*Check to see if ipivot is big enough */
		return -102;

	int _info;
	rawLAPACK::getrs (transa, a_n, _nrhs, _a,	_lda, ipivot.data().begin(), 
				_b, _ldb, _info);

	return _info;
} 

}//namespace LAPACK
}//namespace Bayesian_filter_matrix
