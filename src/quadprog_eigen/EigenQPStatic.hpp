/* 

 Modified by: Eric Cousineau
	 wingsit

 Author: Luca Di Gaspero
 DIEGM - University of Udine, Italy
 l.digaspero@uniud.it
 http://www.diegm.uniud.it/digaspero/

 LICENSE 

 This file is part of QuadProg++: a C++ library implementing
 the algorithm of Goldfarb and Idnani for the solution of a (convex) 
 Quadratic Programming problem by means of an active-set dual method.
 Copyright (C) 2007-2009 Luca Di Gaspero.
 Copyright (C) 2009 Eric Moyer.  

 QuadProg++ is free software: you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 QuadProg++ is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with QuadProg++. If not, see <http://www.gnu.org/licenses/>.

 */

#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>
//#define TRACE_SOLVER

using namespace Eigen;
using std::vector;
//namespace ublas = boost::numeric::ublas;


#define EMATd(n, m) Matrix<double, n, m>
#define EVECd(n) Matrix<double, n, 1>
#define EMATi(n, m) Matrix<int, n, m>
#define EVECi(n) Matrix<int, n, 1>
//#define EROWS(X) X::RowsAtCompileTime
//#define ECOLS(X) X::ColsAtCompileTime

// TODO: Make some sort of workspace just like CVXGEN... Later though?

namespace QP
{

// The Solving function, implementing the Goldfarb-Idnani method

template<typename Scalar, int n, int m>
inline void print_stuff(const char *name, const Matrix<Scalar, n, m> &X)
{
	std::cout << name << " = \n" << X << "\n";
}



inline double distance(double a, double b)
{
	register double a1, b1, t;
	a1 = fabs(a);
	b1 = fabs(b);
	if (a1 > b1) 
	{
		t = (b1 / a1);
		return a1 * ::std::sqrt(1.0 + t * t);
	}
	else
		if (b1 > a1)
		{
			t = (a1 / b1);
			return b1 * ::std::sqrt(1.0 + t * t);
		}
	return a1 * ::std::sqrt(2.0);
}

template<int n>
inline void compute_d(EVECd(n)& d, const EMATd(n, n)& J, const EVECd(n)& np)
{
	register int i, j;
	register double sum;

	/* compute d = H^T * np */
	for (i = 0; i < n; i++)
	{
		sum = 0.0;
		for (j = 0; j < n; j++)
			sum += J(j, i) * np(j);
		d(i) = sum;
	}
}

template<int n>
inline void update_z(EVECd(n)& z, const EMATd(n, n)& J, const EVECd(n)& d, int iq)
{
	register int i, j;

	/* setting of z = H * d */
	for (i = 0; i < n; i++)
	{
		z(i) = 0.0;
		for (j = iq; j < n; j++)
			z(i) += J(i, j) * d(j);
	}
}

template<int n, int p, int m>
inline void update_r(const EMATd(n, n)& R, EVECd(m + p)& r, const EVECd(n)& d, int iq)
{
	register int i, j;/*, n = EROWS(d);*/
	register double sum;

	/* setting of r = R^-1 d */
	for (i = iq - 1; i >= 0; i--)
	{
		sum = 0.0;
		for (j = i + 1; j < iq; j++)
			sum += R(i, j) * r(j);
		r(i) = (d(i) - sum) / R(i, i);
	}
}

template<int n>
bool add_constraint(EMATd(n, n)& R, EMATd(n, n)& J, EVECd(n)& d, int& iq, double& R_norm)
{
#ifdef TRACE_SOLVER
	std::cout << "Add constraint " << iq << '/';
#endif
	register int i, j, k;
	double cc, ss, h, t1, t2, xny;

	/* we have to find the Givens rotation which will reduce the element
    d(j) to zero.
    if it is already zero we don't have to do anything, except of
    decreasing j */  
	for (j = n - 1; j >= iq + 1; j--)
	{
		/* The Givens rotation is done with the ublas::matrix (cc cs, cs -cc).
    If cc is one, then element (j) of d is zero compared with element
    (j - 1). Hence we don't have to do anything. 
    If cc is zero, then we just have to switch column (j) and column (j - 1) 
    of J. Since we only switch columns in J, we have to be careful how we
    update d depending on the sign of gs.
    Otherwise we have to apply the Givens rotation to these columns.
    The i - 1 element of d has to be updated to h. */
		cc = d(j - 1);
		ss = d(j);
		h = distance(cc, ss);
		if (fabs(h) < std::numeric_limits<double>::epsilon()) // h == 0
			continue;
		d(j) = 0.0;
		ss = ss / h;
		cc = cc / h;
		if (cc < 0.0)
		{
			cc = -cc;
			ss = -ss;
			d(j - 1) = -h;
		}
		else
			d(j - 1) = h;
		xny = ss / (1.0 + cc);
		for (k = 0; k < n; k++)
		{
			t1 = J(k, j - 1);
			t2 = J(k, j);
			J(k, j - 1) = t1 * cc + t2 * ss;
			J(k, j) = xny * (t1 + J(k, j - 1)) - t2;
		}
	}
	/* update the number of constraints added*/
	iq++;
	/* To update R we have to put the iq components of the d ublas::vector
    into column iq - 1 of R
	 */
	for (i = 0; i < iq; i++)
		R(i, iq - 1) = d(i);
#ifdef TRACE_SOLVER
	std::cout << iq << std::endl;
	print_stuff("R", R, iq, iq);
	print_stuff("J", J);
	print_stuff("d", d, iq);
#endif

	if (fabs(d(iq - 1)) <= std::numeric_limits<double>::epsilon() * R_norm) 
	{
		// problem degenerate
		return false;
	}
	R_norm = std::max<double>(R_norm, fabs(d(iq - 1)));
	return true;
}

template<int n, int p, int m>
void delete_constraint(EMATd(n, n)& R, EMATd(n, n)& J, EVECi(m + p)& A, EVECd(m + p)& u, int _unsed_n, int _unsed_p, int& iq, int l)
{
#ifdef TRACE_SOLVER
	std::cout << "Delete constraint " << l << ' ' << iq;
#endif
	register int i, j, k, qq = -1; // just to prevent warnings from smart compilers
	double cc, ss, h, xny, t1, t2;

	/* Find the index qq for active constraint l to be removed */
	for (i = p; i < iq; i++)
		if (A(i) == l)
		{
			qq = i;
			break;
		}

	/* remove the constraint from the active set and the duals */
	for (i = qq; i < iq - 1; i++)
	{
		A(i) = A(i + 1);
		u(i) = u(i + 1);
		for (j = 0; j < n; j++)
			R(j, i) = R(j, i + 1);
	}

	A(iq - 1) = A(iq);
	u(iq - 1) = u(iq);
	A(iq) = 0; 
	u(iq) = 0.0;
	for (j = 0; j < iq; j++)
		R(j, iq - 1) = 0.0;
	/* constraint has been fully removed */
	iq--;
#ifdef TRACE_SOLVER
	std::cout << '/' << iq << std::endl;
#endif 

	if (iq == 0)
		return;

	for (j = qq; j < iq; j++)
	{
		cc = R(j, j);
		ss = R(j + 1, j);
		h = distance(cc, ss);
		if (fabs(h) < std::numeric_limits<double>::epsilon()) // h == 0
			continue;
		cc = cc / h;
		ss = ss / h;
		R(j + 1, j) = 0.0;
		if (cc < 0.0)
		{
			R(j, j) = -h;
			cc = -cc;
			ss = -ss;
		}
		else
			R(j, j) = h;

		xny = ss / (1.0 + cc);
		for (k = j + 1; k < iq; k++)
		{
			t1 = R(j, k);
			t2 = R(j + 1, k);
			R(j, k) = t1 * cc + t2 * ss;
			R(j + 1, k) = xny * (t1 + R(j, k)) - t2;
		}
		for (k = 0; k < n; k++)
		{
			t1 = J(k, j);
			t2 = J(k, j + 1);
			J(k, j) = t1 * cc + t2 * ss;
			J(k, j + 1) = xny * (J(k, j) + t1) - t2;
		}
	}
}

template<int n>
inline double scalar_product(const EVECd(n)& x, const EVECd(n)& y)
{
	register int i;
	register double sum;

	sum = 0.0;
	for (i = 0; i < n; i++)
		sum += x(i) * y(i);
	return sum;			
}

template<int n>
void cholesky_decomposition(EMATd(n, n)& A) 
{
	register int i, j, k;
	register double sum;

	for (i = 0; i < n; i++)
	{
		for (j = i; j < n; j++)
		{
			sum = A(i, j);
			for (k = i - 1; k >= 0; k--)
				sum -= A(i, k)*A(j, k);
			if (i == j) 
			{
				if (sum <= 0.0)
				{
					std::ostringstream os;
					// raise error
					print_stuff("A", A);
					os << "Error in cholesky decomposition, sum: " << sum;
					throw std::logic_error(os.str());
					exit(-1);
				}
				A(i, i) = ::std::sqrt(sum);
			}
			else
				A(j, i) = sum / A(i, i);
		}
		for (k = i + 1; k < n; k++)
			A(i, k) = A(k, i);
	} 
}

// TODO: Replace this with Eigen implementation!

template<int n>
void cholesky_solve(const EMATd(n, n)& L, EVECd(n)& x, const EVECd(n)& b)
{
	static EVECd(n) y;
	y.setZero(n);

	/* Solve L * y = b */
	forward_elimination(L, y, b);
	/* Solve L^T * x = y */
	backward_elimination(L, x, y);
}

template<int n>
inline void forward_elimination(const EMATd(n, n)& L, EVECd(n)& y, const EVECd(n)& b)
{
	register int i, j;

	y(0) = b(0) / L(0, 0);
	for (i = 1; i < n; i++)
	{
		y(i) = b(i);
		for (j = 0; j < i; j++)
			y(i) -= L(i, j) * y(j);
		y(i) = y(i) / L(i, i);
	}
}

template<int n>
inline void backward_elimination(const EMATd(n, n)& U, EVECd(n)& x, const EVECd(n)& y)
{
	register int i, j;

	x(n - 1) = y(n - 1) / U(n - 1, n - 1);
	for (i = n - 2; i >= 0; i--)
	{
		x(i) = y(i);
		for (j = i + 1; j < n; j++)
			x(i) -= U(i, j) * x(j);
		x(i) = x(i) / U(i, i);
	}
}





template<int n, int p, int m>
double solve_quadprog(EMATd(n, n)& G, EVECd(n)& g0, 
		const EMATd(n, p)& CE, const EVECd(p)& ce0,  
		const EMATd(n, m)& CI, const EVECd(m)& ci0, 
		EVECd(n)& x)
{
	std::ostringstream msg;
	{
		// Static typing handles sizes
		register int i, j, k, l; /* indices */
		int ip; // this is the index of the constraint to be added to the active set
		static EMATd(n,n) R, J;
		static EVECd(m + p) s, r, u, u_old;
		static EVECd(n) z, d, np, x_old;
		double f_value, psi, c1, c2, sum, ss, R_norm;
		double inf;
		if (std::numeric_limits<double>::has_infinity)
			inf = std::numeric_limits<double>::infinity();
		else
			inf = 1.0E300;
		double t, t1, t2; /* t is the step lenght, which is the minimum of the partial step length t1 
		 * and the full step length t2 */
		static EVECi(m + p) A, A_old, iai;
		int q, iq, iter = 0;
		// Meh...
		//vector<bool> iaexcl(m + p);
		bool iaexcl[m + p];

		/* p is the number of equality constraints */
		/* m is the number of inequality constraints */
		q = 0;  /* size of the active set A (containing the indices of the active constraints) */
#ifdef TRACE_SOLVER
		std::cout << std::endl << "Starting solve_quadprog" << std::endl;
		print_stuff("G", G);
		print_stuff("g0", g0);
		print_stuff("CE", CE);
		print_stuff("ce0", ce0);
		print_stuff("CI", CI);
		print_stuff("ci0", ci0);
#endif  

		/*
		 * Preprocessing phase
		 */

		/* compute the trace of the original ublas::matrix G */
		c1 = 0.0;
		for (i = 0; i < n; i++)
		{
			c1 += G(i, i);
		}
		/* decompose the ublas::matrix G in the form L^T L */
		cholesky_decomposition(G);
#ifdef TRACE_SOLVER
		print_stuff("G", G);
#endif
		/* initialize the ublas::matrix R */
		for (i = 0; i < n; i++)
		{
			d(i) = 0.0;
			for (j = 0; j < n; j++)
				R(i, j) = 0.0;
		}
		R_norm = 1.0; /* this variable will hold the norm of the ublas::matrix R */

		/* compute the inverse of the factorized ublas::matrix G^-1, this is the initial value for H */
		c2 = 0.0;
		for (i = 0; i < n; i++) 
		{
			d(i) = 1.0;
			forward_elimination(G, z, d);
			for (j = 0; j < n; j++)
				J(i, j) = z(j);
			c2 += z(i);
			d(i) = 0.0;
		}
#ifdef TRACE_SOLVER
		print_stuff("J", J);
#endif

		/* c1 * c2 is an estimate for cond(G) */

		/* 
		 * Find the unconstrained minimizer of the quadratic form 0.5 * x G x + g0 x 
		 * this is a feasible point in the dual space
		 * x = G^-1 * g0
		 */
		cholesky_solve(G, x, g0);
		for (i = 0; i < n; i++)
			x(i) = -x(i);
		/* and compute the current solution value */ 
		f_value = 0.5 * scalar_product(g0, x);
#ifdef TRACE_SOLVER
		std::cout << "Unconstrained solution: " << f_value << std::endl;
		print_stuff("x", x);
#endif

		/* Add equality constraints to the working set A */
		iq = 0;
		for (i = 0; i < p; i++)
		{
			for (j = 0; j < n; j++)
				np(j) = CE(j, i);
			compute_d(d, J, np);
			update_z(z, J, d, iq);
			update_r<n, p, m>(R, r, d, iq);
#ifdef TRACE_SOLVER
			print_stuff("R", R, n, iq);
			print_stuff("z", z);
			print_stuff("r", r, iq);
			print_stuff("d", d);
#endif

			/* compute full step length t2: i.e., the minimum step in primal space s.t. the contraint 
      becomes feasible */
			t2 = 0.0;
			if (fabs(scalar_product(z, z)) > std::numeric_limits<double>::epsilon()) // i.e. z != 0
				t2 = (-scalar_product(np, x) - ce0(i)) / scalar_product(z, np);

			/* set x = x + t2 * z */
			for (k = 0; k < n; k++)
				x(k) += t2 * z(k);

			/* set u = u+ */
			u(iq) = t2;
			for (k = 0; k < iq; k++)
				u(k) -= t2 * r(k);

			/* compute the new solution value */
			f_value += 0.5 * (t2 * t2) * scalar_product(z, np);
			A(i) = -i - 1;

			if (!add_constraint(R, J, d, iq, R_norm))
			{	  
				// Equality constraints are linearly dependent
				throw std::runtime_error("Constraints are linearly dependent");
				return f_value;
			}
		}

		/* set iai = K \ A */
		for (i = 0; i < m; i++)
			iai(i) = i;

		l1:	iter++;
#ifdef TRACE_SOLVER
		print_stuff("x", x);
#endif
		/* step 1: choose a violated constraint */
		for (i = p; i < iq; i++)
		{
			ip = A(i);
			iai(ip) = -1;
		}

		/* compute s(x) = ci^T * x + ci0 for all elements of K \ A */
		ss = 0.0;
		psi = 0.0; /* this value will contain the sum of all infeasibilities */
		ip = 0; /* ip will be the index of the chosen violated constraint */
		for (i = 0; i < m; i++)
		{
			iaexcl[i] = true;
			sum = 0.0;
			for (j = 0; j < n; j++)
				sum += CI(j, i) * x(j);
			sum += ci0(i);
			s(i) = sum;
			psi += std::min(0.0, sum);
		}
#ifdef TRACE_SOLVER
		print_stuff("s", s, m);
#endif


		if (fabs(psi) <= m * std::numeric_limits<double>::epsilon() * c1 * c2* 100.0)
		{
			/* numerically there are not infeasibilities anymore */
			q = iq;

			return f_value;
		}

		/* save old values for u and A */
		for (i = 0; i < iq; i++)
		{
			u_old(i) = u(i);
			A_old(i) = A(i);
		}
		/* and for x */
		for (i = 0; i < n; i++)
			x_old(i) = x(i);

		l2: /* Step 2: check for feasibility and determine a new S-pair */
		for (i = 0; i < m; i++)
		{
			if (s(i) < ss && iai(i) != -1 && iaexcl[i])
			{
				ss = s(i);
				ip = i;
			}
		}
		if (ss >= 0.0)
		{
			q = iq;

			return f_value;
		}

		/* set np = n(ip) */
		for (i = 0; i < n; i++)
			np(i) = CI(i, ip);
		/* set u = (u 0)^T */
		u(iq) = 0.0;
		/* add ip to the active set A */
		A(iq) = ip;

#ifdef TRACE_SOLVER
		std::cout << "Trying with constraint " << ip << std::endl;
		print_stuff("np", np);
#endif

		l2a:/* Step 2a: determine step direction */
		/* compute z = H np: the step direction in the primal space (through J, see the paper) */
		compute_d(d, J, np);
		update_z(z, J, d, iq);
		/* compute N* np (if q > 0): the negative of the step direction in the dual space */
		update_r<n, p, m>(R, r, d, iq);
#ifdef TRACE_SOLVER
		std::cout << "Step direction z" << std::endl;
		print_stuff("z", z);
		print_stuff("r", r, iq + 1);
		print_stuff("u", u, iq + 1);
		print_stuff("d", d);
		print_stuff("A", A, iq + 1);
#endif

		/* Step 2b: compute step length */
		l = 0;
		/* Compute t1: partial step length (maximum step in dual space without violating dual feasibility */
		t1 = inf; /* +inf */
		/* find the index l s.t. it reaches the minimum of u+(x) / r */
		for (k = p; k < iq; k++)
		{
			if (r(k) > 0.0)
			{
				if (u(k) / r(k) < t1)
				{
					t1 = u(k) / r(k);
					l = A(k);
				}
			}
		}
		/* Compute t2: full step length (minimum step in primal space such that the constraint ip becomes feasible */
		if (fabs(scalar_product(z, z))  > std::numeric_limits<double>::epsilon()) // i.e. z != 0
			t2 = -s(ip) / scalar_product(z, np);
		else
			t2 = inf; /* +inf */

		/* the step is chosen as the minimum of t1 and t2 */
		t = std::min(t1, t2);
#ifdef TRACE_SOLVER
		std::cout << "Step sizes: " << t << " (t1 = " << t1 << ", t2 = " << t2 << ") ";
#endif

		/* Step 2c: determine new S-pair and take step: */

		/* case (i): no step in primal or dual space */
		if (t >= inf)
		{
			/* QPP is infeasible */
			// FIXME: unbounded to raise
			q = iq;
			return inf;
		}
		/* case (ii): step in dual space */
		if (t2 >= inf)
		{
			/* set u = u +  t * (-r 1) and drop constraint l from the active set A */
			for (k = 0; k < iq; k++)
				u(k) -= t * r(k);
			u(iq) += t;
			iai(l) = l;
			delete_constraint<n, p, m>(R, J, A, u, n, p, iq, l);
#ifdef TRACE_SOLVER
			std::cout << " in dual space: " 
					<< f_value << std::endl;
			print_stuff("x", x);
			print_stuff("z", z);
			print_stuff("A", A, iq + 1);
#endif
			goto l2a;
		}

		/* case (iii): step in primal and dual space */

		/* set x = x + t * z */
		for (k = 0; k < n; k++)
			x(k) += t * z(k);
		/* update the solution value */
		f_value += t * scalar_product(z, np) * (0.5 * t + u(iq));
		/* u = u + t * (-r 1) */
		for (k = 0; k < iq; k++)
			u(k) -= t * r(k);
		u(iq) += t;
#ifdef TRACE_SOLVER
		std::cout << " in both spaces: " 
				<< f_value << std::endl;
		print_stuff("x", x);
		print_stuff("u", u, iq + 1);
		print_stuff("r", r, iq + 1);
		print_stuff("A", A, iq + 1);
#endif

		if (fabs(t - t2) < std::numeric_limits<double>::epsilon())
		{
#ifdef TRACE_SOLVER
			std::cout << "Full step has taken " << t << std::endl;
			print_stuff("x", x);
#endif
			/* full step has taken */
			/* add constraint ip to the active set*/
			if (!add_constraint(R, J, d, iq, R_norm))
			{
				iaexcl[ip] = false;
				delete_constraint<n, p, m>(R, J, A, u, n, p, iq, ip);
#ifdef TRACE_SOLVER
				print_stuff("R", R);
				print_stuff("A", A, iq);
				print_stuff("iai", iai);
#endif
				for (i = 0; i < m; i++)
					iai(i) = i;
				for (i = p; i < iq; i++)
				{
					A(i) = A_old(i);
					u(i) = u_old(i);
					iai(A(i)) = -1;
				}
				for (i = 0; i < n; i++)
					x(i) = x_old(i);
				goto l2; /* go to step 2 */
			}    
			else
				iai(ip) = -1;
#ifdef TRACE_SOLVER
			print_stuff("R", R);
			print_stuff("A", A, iq);
			print_stuff("iai", iai);
#endif
			goto l1;
		}

		/* a patial step has taken */
#ifdef TRACE_SOLVER
		std::cout << "Partial step has taken " << t << std::endl;
		print_stuff("x", x);
#endif
		/* drop constraint l */
		iai(l) = l;
		delete_constraint<n, p, m>(R, J, A, u, n, p, iq, l);
#ifdef TRACE_SOLVER
		print_stuff("R", R);
		print_stuff("A", A, iq);
#endif

		/* update s(ip) = CI * x + ci0 */
		sum = 0.0;
		for (k = 0; k < n; k++)
			sum += CI(k, ip) * x(k);
		s(ip) = sum + ci0(ip);

#ifdef TRACE_SOLVER
		print_stuff("s", s, m);
#endif
		goto l2a;
	}

}



}
