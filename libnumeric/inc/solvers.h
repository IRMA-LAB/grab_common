/**
 * @file solvers.h
 * @author Edoardo Idà, Simone Comari
 * @date 15 Mar 2019
 * @brief File containing numeric solver to be included in the GRAB numeric library.
 */

#ifndef GRABCOMMON_LIBNUMERIC_SOLVERS_H
#define GRABCOMMON_LIBNUMERIC_SOLVERS_H

#include "matrix_utilities.h"
#include <functional>
/**
 * @brief Namespace for GRAB numeric library.
 */
namespace grabnum
{

/**
 * @brief Namespace for GRAB numeric solvers.
 */
namespace solvers
{
/**
 * Solve a @f$m \times n@f$ linear system in matrix form.
 *
 * A generic linear system of _m_ equations with _n_unknowns can be written in matrix form
 *as
 * @f[
 * \mathbf{A}\mathbf{x} = \mathbf{b}
 * @f]
 * being @f$\mathbf{A} \in \mathbb{R}^{m \times n}@f$ the matrix of coefficients of the
 * system, @f$\mathbf{x} \in \mathbb{R}^n@f$ the column vector of unknowns and
 * @f$\mathbf{b} \in \mathbb{R}^m@f$ the column vector of constant terms.
 * This functions finds the least squares solutions @f$\mathbf{x}@f$.
 * @param[in] _mat A @f$m \times n@f$ matrix of coefficients @f$\mathbf{A}@f$.
 * @param[in] _vect b @f$m@f$-dimensional vector of constant terms @f$\mathbf{b}@f$.
 * @return A @f$m@f$-dimensional vector with the solution @f$\mathbf{x}@f$.
 */

template <typename T, uint  unk_dim, uint  eq_dim>
VectorX<T, unk_dim> Linsolve(const Matrix<T, eq_dim, unk_dim>& _mat, const VectorX<T, eq_dim>& _vect);

/**
 * Solve a @f$m \times m@f$ linear system in matrix form.
 *
 * A generic linear system of _m_ equations with _n_unknowns can be written in matrix form
 *as
 * @f[
 * \mathbf{A}\mathbf{x} = \mathbf{b}
 * @f]
 * being @f$\mathbf{A} \in \mathbb{R}^{m \times n}@f$ the matrix of coefficients of the
 * system, @f$\mathbf{x} \in \mathbb{R}^n@f$ the column vector of unknowns and
 * @f$\mathbf{b} \in \mathbb{R}^m@f$ the column vector of constant terms.
 * This functions finds solutions @f$\mathbf{x}@f$ for the case @f$m=n@f$.
 * @param[in] _mat A @f$m \times m@f$ square matrix of coefficients @f$\mathbf{A}@f$.
 * @param[in] _vect A @f$m@f$-dimensional vector of constant terms @f$\mathbf{b}@f$.
 * @return A @f$m@f$-dimensional vector with the solution @f$\mathbf{x}@f$.
 */

template <typename T, uint  dim>
VectorX<T, dim> Linsolve(const Matrix<T, dim, dim>& _mat, const VectorX<T, dim>& _vect);

/**
 * Solve a @f$m \times m@f$ linear system in matrix form.
 *
 * @param[in] _mat A @f$m \times m@f$ square matrix of coefficients @f$\mathbf{A}@f$.
 * @param[in] _vect A @f$m@f$-dimensional vector of constant terms @f$\mathbf{b}@f$.
 * @param[out] result A @f$m@f$-dimensional vector with the solution @f$\mathbf{x}@f$.
 * @see Linsolve()
 * TODO: modify it for handling non-square matrices
 */
template <typename T, uint dim>
void Linsolve(const Matrix<T, dim, dim>& _mat, const VectorX<T, dim>& _vect,
              VectorX<T, dim>& result);

/**
 * Solve a @f$m \times m@f$ linear system in matrix form where the coefficients matrix is
 * upper-triangular.
 *
 * @param[in] mat A @f$m \times m@f$ square upper-triangular matrix.
 * @param[in] vect A @f$m@f$-dimensional vector of constant terms.
 * @return A @f$m@f$-dimensional vector with the solution.
 * @see Linsolve()
 * TODO: modify it for handling non-square matrices
 */
template <typename T, uint dim>
VectorX<T, dim> LinsolveUp(const Matrix<T, dim, dim>& mat, const VectorX<T, dim>& vect);

/**
 * Solve a @f$m \times m@f$ linear system in matrix form where the coefficient matrix is
 * upper-triangular.
 *
 * @param[in] mat A @f$m \times m@f$ square upper-triangular matrix.
 * @param[in] vect A @f$m@f$-dimensional vector of constant terms.
 * @param[out] result A @f$m@f$-dimensional vector with the solution.
 * @see LinsolveUp()
 * TODO: modify it for handling non-square matrices
 */
template <typename T, uint dim>
void LinsolveUp(const Matrix<T, dim, dim>& mat, const VectorX<T, dim>& vect,
                VectorX<T, dim>& result);

/**
 * Solve a non-linear system
 *
 * @param[in] fun_ptr Pointer to function.
 * @param[in/out] solution vector; the input value is the guess, the
 *                output value is the solution
 * @param[in] nmax (Optional) Maximum number of iterations. Default is 100.
 * @todo set a correct type for the parameters.
 * @return A scalar with the number of iterations.
 */
template <typename T, uint unk_dim, uint res_dim>
int NonLinsolveJacobian(
  std::function<void(const grabnum::VectorX<T, unk_dim>&,
                     grabnum::VectorX<T, res_dim>&,
                     grabnum::Matrix<T, res_dim, unk_dim>&)> fun_ptr,
  grabnum::VectorX<T, unk_dim>& solution,
  uint nmax = 100);

/**
 * Minimize a residual vector using the LM method
 *
 * @param[in] fun_ptr Pointer to function.
 * @param[in/out] solution vector; the input value is the guess, the
 *                output value is the solution
 * @todo set a correct type for the parameters.
 * @return the exitflag falue (see next function for reference)
 */
template <typename T, uint unk_dim, uint res_dim>
int LevembergMarquardt(
  std::function<void(const grabnum::VectorX<T, unk_dim>&,
                     grabnum::VectorX<T, res_dim>&,
                     grabnum::Matrix<T, res_dim, unk_dim>&)> fun_ptr,
  grabnum::VectorX<T, unk_dim>& solution);

/**
 * Evaluate the termination conditions of the LM solver
 *
 * @param[in] grad_cost_fun gradient of the scalar cost function.
 * @param[in] opt_tol optimality tolerance.
 * @param[in] fun_tol function tolerance.
 * @param[in] iter current iteration.
 * @param[in] x current solution vector.
 * @param[in] new_cost_fun new cost function value
 * @param[in] step_tol step tolerance.
 * @param[in] step current step.
 * @param[in] old_cost_fun previous cost function value
 * @param[in] max_iter maximum number of admissible iterations
 * @param[out] done flag for convergence
 * @param[out] exitflag ID of the termination condition: 0 = max iterations
 *                                                       1 = optimality tolerance reached
 *                                                       3 = function tolerance reached
 *                                                       4 = step tolerance reached
 *
 */
template <typename T, uint unk_dim>
void TestConvergenceLM(grabnum::VectorX<T, unk_dim> grad_cost_fun, const T opt_tol,
                       const T fun_tol, uint iter, grabnum::VectorX<T, unk_dim> x,
                       T new_cost_fun, T step_tol, grabnum::VectorX<T, unk_dim> step,
                       T old_cost_fun, const uint max_iter, bool& done, int& exitflag);

/**
 * Solve a non-linear system??
 *
 * @param[in] fun_ptr Pointer to function.
 * @param[out] solution Solution vector.
 * @param[in] nmax (Optional) Maximum number of iterations. Default is 100.
 * @todo set a correct type for the parameters.
 * @return A scalar with the number of iterations.
 */
template <typename Params, typename T, uint unk_dim, uint res_dim>
int fsolveB(void (*fun_ptr)(const Params&, const VectorX<T, unk_dim>&,
                            VectorX<T, res_dim>&),
            VectorX<T, unk_dim>& solution, const uint nmax = 100);

/**
 * @brief _Runge–Kutta–Fehlberg method_ for the numerical solution of ODEs.
 *
 * The _Runge–Kutta–Fehlberg method_ (or _Fehlberg method_) is an algorithm in numerical
 * analysis for the numerical solution of ordinary differential equations. It is a method
 *of order
 * @f$O(h^4)@f$ with an error estimator of order @f$O(h^5)@f$.
 * @param[in] fun_ptr Pointer to differential equation of type
 * @f$\dot{\mathbf{y}} = f(t, \mathbf{y}), \mathbf{y} \in \mathbb{R}^m@f$. The arguments
 * of such function @f$f@f$ are (_time instant_ @f$t@f$ [s], _input vector_ @f$\mathbf{y}@f$,
 * _output vector_ @f$\dot{\mathbf{y}}@f$).
 * @param[in] time _n_-dimensional time vector with time step @f$h = t_k - t_{k-1}@f$ [s].
 * @param[in] y0 Values of @f$\mathbf{y}@f$ at initial time @f$t_0@f$, i.e.
 * @f$\mathbf{y}_0@f$.
 * @param[out] sol @f$m \times n@f$ solution matrix, where _i-th_ column represents the
 * solution of the problem at instant @f$t_i@f$, i.e. @f$\mathbf{y}_i@f$.
 */
template <typename T, uint dim, size_t t_steps>
void RKSolver(void (*fun_ptr)(const T, const VectorX<T, dim>, VectorX<T, dim>),
              const VectorX<T, t_steps>& time, const VectorX<T, dim>& y0,
              Matrix<T, dim, t_steps>& sol);

       // here start the definitions of the template functions
template <typename T, uint unk_dim, uint eq_dim>
VectorX<T, unk_dim> Linsolve(const Matrix<T, eq_dim, unk_dim>& _mat, const VectorX<T, eq_dim>& _vect)
{
  Matrix<T, unk_dim, unk_dim> mat(_mat.Transpose() * _mat);
  VectorX<T, unk_dim> vect(_mat.Transpose()*_vect);

  return Linsolve(mat, vect);
}

template <typename T, uint dim>
VectorX<T, dim> Linsolve(const Matrix<T, dim, dim>& _mat, const VectorX<T, dim>& _vect)
{
  static_assert(std::is_floating_point<T>::value, "ERROR: invalid type in LinSolve()!");

  Matrix<T, dim, dim> mat(_mat);
  VectorX<T, dim> vect(_vect);
  for (uint i = 1; i < dim; ++i)
  {
    uint max_idx = i;
    T max_val = mat(i, i);
    for (uint j = i + 1; j <= dim; ++j)
    {
      if (fabs(mat(j, i)) > fabs(max_val))
      {
        max_val = mat(j, i);
        max_idx = j;
      }
    }
    mat.SwapRow(i, max_idx);
    vect.SwapRow(i, max_idx);
    for (uint j = i + 1; j <= dim; ++j)
    {
      T m = mat(j, i) / mat(i, i);
      vect(j) -= vect(i) * m;
      for (uint k = i; k <= dim; ++k)
      {
        mat(j, k) -= mat(i, k) * m;
      }
    }
  }
  return LinsolveUp(mat, vect);
}

template <typename T, uint dim>
void Linsolve(const Matrix<T, dim, dim>& _mat, const VectorX<T, dim>& _vect,
              VectorX<T, dim>& result)
{
  static_assert(std::is_floating_point<T>::value, "ERROR: invalid type in LinSolve()!");

  Matrix<T, dim, dim> mat(_mat);
  VectorX<T, dim> vect(_vect);
  for (uint i = 1; i < dim; ++i)
  {
    uint max_idx = i;
    T max_val = mat(i, i);
    for (uint j = i + 1; j <= dim; ++j)
    {
      if (fabs(mat(j, i)) > fabs(max_val))
      {
        max_val = mat(j, i);
        max_idx = j;
      }
    }
    mat.SwapRow(i, max_idx);
    vect.SwapRow(i, max_idx);
    for (uint j = i + 1; j <= dim; ++j)
    {
      T m = mat(j, i) / mat(i, i);
      vect(j) -= vect(i) * m;
      for (uint k = i; k <= dim; ++k)
      {
        mat(j, k) -= mat(i, k) * m;
      }
    }
  }
  LinsolveUp(mat, vect, result);
}

template <typename T, uint dim>
VectorX<T, dim> LinsolveUp(const Matrix<T, dim, dim>& mat, const VectorX<T, dim>& vect)
{
  static_assert(std::is_floating_point<T>::value, "ERROR: invalid type in LinSolveUp()!");

  VectorX<T, dim> result;
  result = vect;
  result(dim) /= mat(dim, dim);
  for (uint j = dim - 1; j > 0; j--)
  {
    for (uint k = j + 1; k <= dim; ++k)
    {
      result(j) -= mat(j, k) * result(k);
    }
    result(j) /= mat(j, j);
  }

  return result;
}

template <typename T, uint dim>//correzioneuint con unisgned int
void LinsolveUp(const Matrix<T, dim, dim>& mat, const VectorX<T, dim>& vect,
                VectorX<T, dim>& result)
{
  static_assert(std::is_floating_point<T>::value, "ERROR: invalid type in LinSolveUp()!");

  result = vect;
  result(dim) /= mat(dim, dim);
  for (uint j = dim - 1; j > 0; j--)
  {
    for (uint k = j + 1; k <= dim; ++k)
    {
      result(j) -= mat(j, k) * result(k);
    }
    result(j) /= mat(j, j);
  }
}

template <typename T, uint unk_dim, uint res_dim>
int NonLinsolveJacobian(
  std::function<void(const grabnum::VectorX<T, unk_dim>&,
                     grabnum::VectorX<T, res_dim>&,
                     grabnum::Matrix<T, res_dim, unk_dim>&)> fun_ptr,
  grabnum::VectorX<T, unk_dim>& solution,
  uint nmax)
{
  static const T ftol = 1e-9;
  static const T xtol = 1e-7;
  uint iter = 0;
  T err = 1.0;
  T cond = 0.0;
  VectorX<T, unk_dim> s;
  VectorX<T, res_dim> F;
  Matrix<T, res_dim, unk_dim> J;

  fun_ptr(solution, F, J);

  while (iter < nmax && Norm(F) > ftol && err > cond)
  {
    iter++;
    s = Linsolve(J, F);
    solution -= s;
    fun_ptr(solution, F, J);
    err = Norm(s);
    cond = xtol * (1 + Norm(solution));
  }

  return iter;
}

template <typename T, uint unk_dim, uint res_dim>
int LevembergMarquardt(
  std::function<void(const grabnum::VectorX<T, unk_dim>&,
                     grabnum::VectorX<T, res_dim>&,
                     grabnum::Matrix<T, res_dim, unk_dim>&)> fun_ptr,
  grabnum::VectorX<T, unk_dim>& solution)
{
  // solver params
  static const uint nmax = 100;
  static const T ftol = 1e-9;
  static const T xtol = 1e-8;
  static const T optol = 1e-4*ftol;

         // solver handlers
  bool done;
  uint iter = 0;
  bool success_step = true;
  VectorX<T, unk_dim> damping(1e-3); // adaptive damping
  int exitflag;

         // cost function variables
  VectorX<T, unk_dim> s(0);
  VectorX<T, res_dim> F;
  Matrix<T, res_dim, unk_dim> J;
  T fval;

         // cost function updates
  VectorX<T, unk_dim> sol_new;
  VectorX<T, res_dim> F_new;
  Matrix<T, res_dim, unk_dim> J_new;
  T fval_new = 0.0;

  fun_ptr(solution, F, J);
  fval = pow(Norm(F),2)/2;

         // test convergence with tentative solution
  TestConvergenceLM(J.Transpose()*F,optol,ftol,iter,solution,
                    fval_new,xtol,s,fval,nmax,done,exitflag);

  while(!done) {

      // classic LM step with damping
    s =-Linsolve((J.Transpose()*J+Diag(damping)),J.Transpose()*F);
    sol_new = solution + s;

    fun_ptr(sol_new, F_new, J_new);
    fval_new = pow(Norm(F_new),2)/2;

           // damping scaling
    std::cout<<"Difference: "<<fval_new-fval<<std::endl;
    if (fval_new<=fval+EPSILON)
    {
      solution = sol_new;
      F = F_new;
      J = J_new;
      if (success_step)
        damping /= 10;
      success_step = true;
      iter++;
      TestConvergenceLM(J.Transpose()*F,optol,ftol,iter,solution,
                        fval_new,xtol,s,fval,nmax,done,exitflag);
      fval = fval_new;
    }
    else {
      damping *= 10;
      success_step = false;
    }
  }

  return exitflag;
}

template <typename T, uint unk_dim>
void TestConvergenceLM(grabnum::VectorX<T, unk_dim> grad_cost_fun, const T opt_tol,
                       const T fun_tol, uint iter, grabnum::VectorX<T, unk_dim> x,
                       T new_cost_fun, T step_tol, grabnum::VectorX<T, unk_dim> step,
                       T old_cost_fun, const uint max_iter, bool& done, int& exitflag)
{
  done = false;

  if (iter>=max_iter)
  {
    exitflag = 0;
    done = true;
  }
  else if (grabnum::NormInf(grad_cost_fun)<=opt_tol)
  {
    exitflag = 1;
    done = true;
  }
  else if (iter > 0)
  {
    if (std::abs(new_cost_fun-old_cost_fun)<=fun_tol*old_cost_fun)
    {
      exitflag = 3;
      done = true;
    }
    else if (grabnum::Norm(step)<=step_tol*(1.5e-8+grabnum::Norm(x)))
    {
      exitflag = 4;
      done = true;
    }
  }
}

template <typename Params, typename T, uint unk_dim, uint res_dim>
int fsolveB(void (*fun_ptr)(const Params&, const VectorX<T, unk_dim>&, VectorX<T, res_dim>&),
            VectorX<T, unk_dim>& solution, const uint nmax /*= 100*/)
{
  static_assert(std::is_floating_point<T>::value, "ERROR: invalid type in fsolveB()!");
  static_assert(std::is_default_constructible<Params>::value,
                "Params must be default-constructible.");

  static const T ftol = 1e-9;
  static const T xtol = 1e-9;
  uint iter = 0;
  T err = 1.0;
  T cond = 0.0;
  Params params;
  VectorX<T, unk_dim> s;
  VectorX<T, res_dim> F;
  VectorX<T, res_dim> F_prev;
  Matrix<T, res_dim, unk_dim> J;

  fun_ptr(params, solution, F);
  J.Identity();

  while (iter < nmax && Norm(F) > ftol && err > cond)
  {
    iter++;
    Linsolve(J, F, s);
    solution -= s;
    fun_ptr(params, solution, F);
    J += (((F - F_prev) - J * s) * s.Transpose()) / (Dot(s, s));
    F_prev = F;
    err = Norm(s);
    cond = xtol * (1 + Norm(solution));
  }

  return iter;
}

template <typename T, uint dim, size_t t_steps>
void RKSolver(void (*fun_ptr)(const T, const VectorX<T, dim>, VectorX<T, dim>),
              const VectorX<T, t_steps>& time, const VectorX<T, dim>& y0,
              Matrix<T, dim, t_steps>& sol)
{
  static constexpr uint rk_dim = 6;
  static const VectorX<T, rk_dim> c(
    std::vector<T>{0., 0.25, 0.375, 12. / 13., 1., 0.5});
  static const VectorX<T, rk_dim> b(
    std::vector<T>{16. / 135., 0., 6656. / 12825., 28561. / 56430., -0.18, 2. / 5.});
  static const Matrix<T, rk_dim, rk_dim> rk_mat(std::vector<T>{
                                                                0., 0., 0., 0., 0., 0.,
                                                                0.25, 0., 0., 0., 0., 0,
                                                                0.09375, 0, 28125, 0., 0., 0., 0.,
                                                                1932. / 2197., -7200. / 2197., 7296. / 2197., 0., 0., 0.,
                                                                439. / 216., -8., 3680. / 513., -845. / 4104., 0, 0,
                                                                -8. / 27., 2., -3544. / 2565., 1859. / 4104., -0.275, 0.});

         // Initialize
  Matrix<T, dim, rk_dim> K;
  VectorX<T, dim> f, s, col;
  T h = time(2) - time(1);
  sol.SetZero();
  sol.SetCol(1, y0);

         // Solve
  for (size_t i = 2; i <= t_steps; ++i)
  {
    f.SetZero();
    for (uint j = 1; j <= rk_dim; ++j)
    {
      s.SetZero();
      for (uint k = 1; k <= j - 1; ++k)
      {
        s += rk_mat(j, k) * K.getCol(k);
      }
      fun_ptr(time(i - 1) + h * c(j), sol.GetCol(i - 1) + h * s, col);
      K.SetCol(j, col);
      f += b(j) * col;
    }
    sol.SetCol(i, sol.GetCol(i - 1) + h * f);
  }
}
} // end namespace solvers

} // end namespace grabnum

#endif // GRABCOMMON_LIBNUMERIC_SOLVERS_H
