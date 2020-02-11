/**
 * @file common.h
 * @author Simone Comari
 * @date 07 Feb 2020
 * @brief File containing common basic utilities to be included in the GRAB numeric
 * library.
 */

#ifndef GRABCOMMON_LIBNUMERIC_COMMON_H
#define GRABCOMMON_LIBNUMERIC_COMMON_H

#include <cmath>
#include <stdlib.h>
#include <string>
#include <vector>

#ifndef SQUARE
#define SQUARE(x) ((x) * (x)) /**< returns the square of an element. */
#endif

/**
 * @brief Namespace for GRAB numeric library.
 */
namespace grabnum {

/**
 * @brief Custom lib numeric runtime error exception structure.
 */
struct Exception: public std::exception
{
 public:
  /**
   * @brief Constructor.
   * @param[in] message The optional detailed description of the error occured.
   */
  explicit Exception(const std::string& message = "")
  {
    if (!message.empty())
      message_ = "GRAB numeric runtime error: " + message;
    else
      message_ = "GRAB numeric runtime error";
  }

  // @cond DO_NOT_DOCUMENT
  virtual const char* what() const noexcept { return message_.c_str(); }
  // @endcond

 private:
  std::string message_;
};

static constexpr double EPSILON = 1e-7; /**< tolerance for floating point comparison */

template <typename T>
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
inline bool IsClose(const T a, const T b, const double tol = EPSILON)
{
  return fabs(a - b) <= tol;
}

template <typename T>
/**
 * @brief Mean value of a standard vector.
 * @param[in] vect A m-dimensional standard vector.
 * @return A scalar value.
 */
double Mean(const std::vector<T>& vect)
{
  T sum = 0;
  for (size_t i = 0; i < vect.size(); ++i)
    sum += vect[i];
  return static_cast<double>(sum) / static_cast<double>(vect.size());
}

template <typename T>
/**
 * @brief Standard deviation of a standard vector.
 * @param[in] vect A m-dimensional standard vector.
 * @return A scalar value.
 */
double Std(const std::vector<T>& vect)
{
  double mean = Mean(vect);
  double sum  = 0;
  for (size_t i = 0; i < vect.size(); ++i)
    sum += SQUARE(vect[i] - mean);
  return sqrt(static_cast<double>(sum) / static_cast<double>(vect.size()));
}

} //  end namespace grabnum

#endif // GRABCOMMON_LIBNUMERIC_COMMON_H
