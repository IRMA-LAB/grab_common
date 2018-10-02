/**
 * @file clocks.h
 * @author Simone Comari
 * @date 17 Sep 2018
 * @brief This file includes time related utilities such as time conversion and clock class.
 */

#ifndef GRABCOMMON_LIBGRABRT_CLOCKS_H
#define GRABCOMMON_LIBGRABRT_CLOCKS_H

#include <iostream>
#include "grabcommon.h"

namespace grabrt
{

  /**
 * @brief Converts seconds in nanoseconds.
 * @param seconds Time in seconds.
 * @return Time in nanoseconds.
 */
uint64_t Sec2NanoSec(const double seconds);
/**
 * @brief Converts nanoseconds in seconds.
 * @param nanoseconds Time in nanoseconds.
 * @return Time in seconds.
 */
double NanoSec2Sec(const long nanoseconds);

/**
 * @brief This simple class can be used to keep track of time to perform cyclic tasks such as
 * in real-time thread loops.
 * @todo usage example.
 */
class ThreadClock
{
public:
  /**
   * @brief Constructor.
   * @param[in] cycle_time_nsec (Optional) Cycle period in nanoseconds.
   * @param[in] clk_name (Optional) Instance name.
   */
  ThreadClock(const uint64_t cycle_time_nsec = 1000LL,
              const std::string& clk_name = "ThreadClock")
    : name_(clk_name), period_nsec_(cycle_time_nsec)
  {
  }

  /**
   * @brief Set cycle time.
   * @param[in] cycle_time_nsec (Optional) Cycle period in nanoseconds.
   */
  void SetCycleTime(const uint64_t cycle_time_nsec) { period_nsec_ = cycle_time_nsec; }
  /**
   * @brief Sets internal time to current clock value (it uses @c CLOCK_MONOTONIC ).
   * @note For further details, click
   * <a href="https://linux.die.net/man/3/clock_gettime">here</a>.
   */
  void Reset();
  /**
   * @brief Return elapsed time since latest Reset().
   * @return Elapsed time since latest Reset() in seconds.
   */
  double Elapsed() const;
  /**
   * @brief Sets internal time to current value + _cycle period_.
   * @see WaitUntilNext() GetNextTime()
   */
  void Next();
  /**
   * @brief Sets internal time to current value + _cycle period_ and sleep until that time.
   * @note If request is less than or equal to the current value of the clock, then returns
   * immediately without suspending the calling thread.
   * @note For further details, click
   * <a href="http://man7.org/linux/man-pages/man2/clock_nanosleep.2.html">here</a>.
   */
  void WaitUntilNext();

  /**
   * @brief Gets instance name.
   * @return A string reporting the instance name.
   */
  std::string GetName() const { return name_; }
  /**
   * @brief Gets the cycle period.
   * @return The cycle period in nanoseconds.
   */
  uint64_t GetCycleTime() const { return period_nsec_; }
  /**
   * @brief Gets current internal time.
   * @return Current internal time.
   * @note For further details about output time structure, click
   * <a href="https://en.cppreference.com/w/c/chrono/timespec">here</a>.
   * @see GetNextTime()
   */
  struct timespec GetCurrentTime() const { return time_; }
  /**
   * @brief Sets internal time to current value + _cycle period_ and returns it.
   * @return Next internal time.
   * @note For further details about output time structure, click
   * <a href="https://en.cppreference.com/w/c/chrono/timespec">here</a>.
   * @see GetCurrentTime()
   */
  struct timespec GetNextTime();

  /**
   * @brief Displays current internal time in a pretty format.
   */
  void DispCurrentTime() const;
  /**
   * @brief Sets internal time to current value + _cycle period_ and displays current internal
   * time in a pretty format.
   */
  void DispNextTime();

private:
  static constexpr uint64_t kNanoSec2Sec = 1000000000L;

  std::string name_;
  struct timespec time_;
  uint64_t period_nsec_;

  /**
   * @brief This wrapper adds the instance name before the error message displayed by
   * HandleErrorEn().
   * @see HandleErrorEn()
   */
  [[noreturn]] void HandleErrorEnWrapper(const int en, const char* msg) const;
};

} // end namespace grabrt

#endif // GRABCOMMON_LIBGRABRT_CLOCKS_H
