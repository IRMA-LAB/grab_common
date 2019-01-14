/**
 * @file common.h
 * @author Simone Comari
 * @date 06 Dec 2018
 * @brief File containing common basic utilities to be included in the GRAB numeric
 * library.
 */
#ifndef GRABCOMMON_LIBNUMERIC_COMMON_H
#define GRABCOMMON_LIBNUMERIC_COMMON_H

#include <vector>
#include <stdlib.h>
#include <cmath>

#ifndef SQUARE
#define SQUARE(x) ((x) * (x)) /**< returns the square of an element. */
#endif

namespace grabnum
{

static constexpr double EPSILON = 1e-7; /**< tolerance for floating point comparison */

/**
 * @brief Check whether two floating numbers are close within a certain tolerance.
 *
 * This function is safer when comparing two floating numbers than using the standard `==`
 * operator, which could deliver misleading result due to numerical errors.
 * @param[in] a First floating point value to be compared.
 * @param[in] b Second floating point value to be compared.
 * @param[in] tol (Optional) Maximum tolerance on the absolute difference in order to
 * consider the two values close (aka "equal").
 * return _True_ if their absolute difference is within the threshold.
 * @see EPSILON
 */
template <typename T>
inline bool IsClose(const T a, const T b, const double tol = EPSILON)
{
  return fabs(a - b) <= tol;
}

/**
 * Mean value of a standard vector.
 *
 * @param[in] vvect A m-dimensional standard vector.
 * @return A scalar value.
 */
template <typename T> double Mean(const std::vector<T>& vect)
{
  T sum = 0;
  for (size_t i = 0; i < vect.size(); ++i)
    sum += vect[i];
  return static_cast<double>(sum) / static_cast<double>(vect.size());
}

/**
 * Standard deviation of a standard vector.
 *
 * @param[in] vvect A m-dimensional standard vector.
 * @return A scalar value.
 */
template <typename T> double Std(const std::vector<T>& vect)
{
  double mean = Mean(vect);
  double sum = 0;
  for (size_t i = 0; i < vect.size(); ++i)
    sum += SQUARE(vect[i] - mean);
  return sqrt(static_cast<double>(sum) / static_cast<double>(vect.size()));
}

} //  end namespace grabnum

#endif // GRABCOMMON_LIBNUMERIC_COMMON_H
