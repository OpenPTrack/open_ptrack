/*
 * Bayes++ the Bayesian Filtering Library
 * Copyright (c) 2002 Michael Stevens
 * See accompanying Bayes++.htm for terms and conditions of use.
 *
 * $Id$
 */

/*
 * Information Root Filter.
 */
#include "infRtFlt.hpp"
#include "matSup.hpp"
#include "uLAPACK.hpp"	// Common LAPACK interface
#include <algorithm>

/* Filter namespace */
namespace Bayesian_filter
{
	using namespace Bayesian_filter_matrix;


Information_root_scheme::Information_root_scheme (std::size_t x_size, std::size_t /*z_initialsize*/) :
		Kalman_state_filter(x_size),
		r(x_size), R(x_size,x_size)
/* Set the size of things we know about
 */
{}

Information_root_info_scheme::Information_root_info_scheme (std::size_t x_size, std::size_t z_initialsize) :
		Kalman_state_filter(x_size), Information_state_filter (x_size), 
		Information_root_scheme (x_size, z_initialsize)
{}


void Information_root_scheme::init ()
/* Initialise the filter from x,X
 * Precondition:
 *		x,X
 * Postcondition:
 *		inv(R)*inv(R)' = X is PSD
 *		r = R*x
 */
{
						// Information Root
	Float rcond = UCfactor (R, X);
	rclimit.check_PD(rcond, "Initial X not PD");
	bool singular = UTinverse (R);
	assert (!singular); (void)singular;
						// Information Root state r=R*x
	noalias(r) = prod(R,x);
}

void Information_root_info_scheme::init_yY ()
/* Special Initialisation directly from Information form
 * Precondition:
 *		y,Y
 * Postcondition:
 *		R'*R = Y is PSD
 *		r = inv(R)'*y
 */
{
					// Temporary R Matrix for factorisation
	const std::size_t n = x.size();
	LTriMatrix LC(n,n);
					// Information Root
	Float rcond = LdLfactor (LC, Y);
	rclimit.check_PD(rcond, "Initial Y not PD");

	{				// Lower triangular Cholesky factor of LdL'
		std::size_t i,j;
		for (i = 0; i < n; ++i)
		{
			using namespace std;		// for sqrt
			LTriMatrix::value_type sd = LC(i,i);
			sd = sqrt(sd);
			LC(i,i) = sd;
						// Multiply columns by square of non zero diagonal. TODO use column operation
			for (j = i+1; j < n; ++j)
			{
				LC(j,i) *= sd;
			}
		}
	}
	R = FM::trans(LC);			// R = (L*sqrt(d))'

	UTriMatrix RI(n,n);
	RI = R;
	bool singular = UTinverse(RI);
	assert (!singular); (void)singular;
	noalias(r) = prod(FM::trans(RI),y);
}

void Information_root_scheme::update ()
/* Recompute x,X from r,R
 * Precondition:
 *		r(k|k),R(k|k)
 * Postcondition:
 *		r(k|k),R(k|k)
 *		x = inv(R)*r
 *		X = inv(R)*inv(R)'
 */
{
	UTriMatrix RI (R);	// Invert Cholesky factor
	bool singular = UTinverse (RI);
	if (singular)
		error (Numeric_exception("R not PD"));

	noalias(X) = prod_SPD(RI);		// X = RI*RI'
	noalias(x) = prod(RI,r);
}

void Information_root_info_scheme::update_yY ()
/* Recompute y,Y from r,R
 * Precondition:
 *		r(k|k),R(k|k)
 * Postcondition:
 *		r(k|k),R(k|k)
 *		Y = R'*R
 *		y = Y*x
 */
{
	Information_root_scheme::update();
	noalias(Y) = prod(trans(R),R);		// Y = R'*R
	noalias(y) = prod(Y,x);
}


void Information_root_scheme::inverse_Fx (FM::DenseColMatrix& invFx, const FM::Matrix& Fx)
/* Numerical Inversion of Fx using LU factorisation
 * Required LAPACK getrf (with PIVOTING) and getrs
 */
{
								// LU factorise with pivots
	DenseColMatrix FxLU (Fx);
	LAPACK::pivot_t ipivot(FxLU.size1());		// Pivots initialised to zero
	ipivot.clear();

	int info = LAPACK::getrf(FxLU, ipivot);
	if (info < 0)
		error (Numeric_exception("Fx not LU factorisable"));

	FM::identity(invFx);				// Invert
	info = LAPACK::getrs('N', FxLU, ipivot, invFx);
	if (info != 0)
		error (Numeric_exception("Predict Fx not LU invertable"));
}



Bayes_base::Float
 Information_root_scheme::predict (Linrz_predict_model& f, const FM::ColMatrix& invFx, bool linear_r)
/* Linrz Prediction: using precomputed inverse of f.Fx
 * Precondition:
 *   r(k|k),R(k|k)
 * Postcondition:
 *   r(k+1|k) computed using QR decomposition see Ref[1]
 *   R(k+1|k)
 *
 * r can be computed in two was:
 * Either directly in the linear form or  using extended form via R*f.f(x)
 *
 * Requires LAPACK geqrf for QR decomposition (without PIVOTING)
 */
{
	if (!linear_r)
		update ();		// x is required for f(x);

						// Require Root of correlated predict noise (may be semidefinite)
	Matrix Gqr (f.G);
	for (Vec::const_iterator qi = f.q.begin(); qi != f.q.end(); ++qi)
	{
		if (*qi < 0)
			error (Numeric_exception("Predict q Not PSD"));
		column(Gqr, qi.index()) *= std::sqrt(*qi);
	}
						// Form Augmented matrix for factorisation
	const std::size_t x_size = x.size();
	const std::size_t q_size = f.q.size();
						// Column major required for LAPACK, also this property is using in indexing
	DenseColMatrix A(q_size+x_size, q_size+x_size+unsigned(linear_r));
	FM::identity (A);	// Prefill with identity for top left and zero's in off diagonals

	Matrix RFxI (prod(R, invFx));
	A.sub_matrix(q_size,q_size+x_size, 0,q_size) .assign (prod(RFxI, Gqr));
	A.sub_matrix(q_size,q_size+x_size, q_size,q_size+x_size) .assign (RFxI);
	if (linear_r)
		A.sub_column(q_size,q_size+x_size, q_size+x_size) .assign (r);

						// Calculate factorisation so we have and upper triangular R
	DenseVec tau(q_size+x_size);
	int info = LAPACK::geqrf (A, tau);
	if (info != 0)
			error (Numeric_exception("Predict no QR factor"));
						// Extract the roots, junk in strict lower triangle
	R = UpperTri( A.sub_matrix(q_size,q_size+x_size, q_size,q_size+x_size) );
    if (linear_r)
		noalias(r) = A.sub_column(q_size,q_size+x_size, q_size+x_size);
	else
		noalias(r) = prod(R,f.f(x));	// compute r using f(x)

	return UCrcond(R);	// compute rcond of result
}


Bayes_base::Float
 Information_root_scheme::predict (Linrz_predict_model& f)
/* Linrz Prediction: computes inverse model using inverse_Fx
 */
{
						// Require inverse(Fx)
	DenseColMatrix FxI(f.Fx.size1(), f.Fx.size2());
	inverse_Fx (FxI, f.Fx);

	return predict (f, ColMatrix(FxI), false);
}


Bayes_base::Float
 Information_root_scheme::predict (Linear_predict_model& f)
/* Linear Prediction: computes inverse model using inverse_Fx
 */
{
						// Require inverse(Fx)
	DenseColMatrix FxI(f.Fx.size1(), f.Fx.size2());
	inverse_Fx (FxI, f.Fx);

	return predict (f, ColMatrix(FxI), true);
}


Bayes_base::Float Information_root_scheme::observe_innovation (Linrz_correlated_observe_model& h, const FM::Vec& s)
/* Extended linrz correlated observe
 * Precondition:
 *		r(k+1|k),R(k+1|k)
 * Postcondition:
 *		r(k+1|k+1),R(k+1|k+1)
 *
 * Uses LAPACK geqrf for QR decomposition (without PIVOTING)
 * ISSUE correctness of linrz form needs validation
 */
{
	const std::size_t x_size = x.size();
	const std::size_t z_size = s.size();
						// Size consistency, z to model
	if (z_size != h.Z.size1())
		error (Logic_exception("observation and model size inconsistent"));

						// Require Inverse of Root of uncorrelated observe noise
	UTriMatrix Zir(z_size,z_size);
	Float rcond = UCfactor (Zir, h.Z);
	rclimit.check_PD(rcond, "Z not PD");
	bool singular = UTinverse (Zir);
	assert (!singular); (void)singular;
						// Form Augmented matrix for factorisation
	DenseColMatrix A(x_size+z_size, x_size+1);	// Column major required for LAPACK, also this property is using in indexing
	A.sub_matrix(0,x_size, 0,x_size) .assign (R);
	A.sub_matrix(x_size,x_size+z_size, 0,x_size) .assign (prod(Zir, h.Hx));
	A.sub_column(0,x_size, x_size) .assign (r);
	A.sub_column(x_size,x_size+z_size, x_size) .assign (prod(Zir, s+prod(h.Hx,x)));

						// Calculate factorisation so we have and upper triangular R
	DenseVec tau(x_size+1);
	int info = LAPACK::geqrf (A, tau);
	if (info != 0)
			error (Numeric_exception("Observe no QR factor"));
						// Extract the roots, junk in strict lower triangle
	noalias(R) = UpperTri( A.sub_matrix(0,x_size, 0,x_size) );
	noalias(r) = A.sub_column(0,x_size, x_size);

	return UCrcond(R);	// compute rcond of result
}


Bayes_base::Float Information_root_scheme::observe_innovation (Linrz_uncorrelated_observe_model& h, const FM::Vec& s)
/* Extended linrz uncorrelated observe
 * Precondition:
 *		r(k+1|k),R(k+1|k)
 * Postcondition:
 *		r(k+1|k+1),R(k+1|k+1)
 *
 * Uses LAPACK geqrf for QR decomposition (without PIVOTING)
 * ISSUE correctness of linrz form needs validation
 * ISSUE Efficiency. Product of Zir can be simplified
 */
{
	const std::size_t x_size = x.size();
	const std::size_t z_size = s.size();
						// Size consistency, z to model
	if (z_size != h.Zv.size())
		error (Logic_exception("observation and model size inconsistent"));

						// Require Inverse of Root of uncorrelated observe noise
	DiagMatrix Zir(z_size,z_size);
	Zir.clear();
	for (std::size_t i = 0; i < z_size; ++i)
	{
		Zir(i,i) = 1 / std::sqrt(h.Zv[i]);
	}
						// Form Augmented matrix for factorisation
	DenseColMatrix A(x_size+z_size, x_size+1);	// Column major required for LAPACK, also this property is using in indexing
	A.sub_matrix(0,x_size, 0,x_size) .assign(R);
	A.sub_matrix(x_size,x_size+z_size, 0,x_size) .assign (prod(Zir, h.Hx));
	A.sub_column(0,x_size, x_size) .assign (r);
	A.sub_column(x_size,x_size+z_size, x_size) .assign (prod(Zir, s+prod(h.Hx,x)));

						// Calculate factorisation so we have and upper triangular R
	DenseVec tau(x_size+1);
	int info = LAPACK::geqrf (A, tau);
	if (info != 0)
			error (Numeric_exception("Observe no QR factor"));
						// Extract the roots, junk in strict lower triangle
	noalias(R) = UpperTri( A.sub_matrix(0,x_size, 0,x_size) );
	noalias(r) = A.sub_column(0,x_size, x_size);

	return UCrcond(R);	// compute rcond of result
}

}//namespace
