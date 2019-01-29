/**
 * @file clocks.h
 * @author Simone Comari
 * @date 25 Gen 2019
 * @brief This file includes time related utilities such as time conversion and clock
 * classes.
 */

#ifndef GRABCOMMON_LIBGRABRT_CLOCKS_H
#define GRABCOMMON_LIBGRABRT_CLOCKS_H

#include <iostream>
#include "grabcommon.h"

/**
 * @brief operator +
 * @param time1
 * @param time2
 * @return
 */
struct timespec operator+(const struct timespec& time1, const struct timespec& time2);
/**
 * @brief operator +
 * @param time1
 * @param time_sec
 * @return
 */
struct timespec operator+(const struct timespec& time1, const double& time_sec);
/**
 * @brief operator +
 * @param time1
 * @param time_nsec
 * @return
 */
struct timespec operator+(const struct timespec& time1, const int64_t& time_nsec);
/**
 * @brief operator +
 * @param time1
 * @param time_nsec
 * @return
 */
struct timespec operator+(const struct timespec& time1, const uint64_t& time_nsec);

namespace grabrt
{

/**
*@brief Converts seconds in nanoseconds.
*@param seconds Time in seconds.
*@return Time in nanoseconds.
*/
uint64_t Sec2NanoSec(const double seconds);
/**
 * @brief Converts nanoseconds in seconds.
 * @param nanoseconds Time in nanoseconds.
 * @return Time in seconds.
 */
double NanoSec2Sec(const long nanoseconds);

/**
 * @brief The Clock class
 */
class Clock
{
public:
  /**
   * @brief Clock
   * @param clk_name
   */
  Clock(const std::string& clk_name = "ThreadClock");
  virtual ~Clock() {}

  static constexpr uint64_t kNanoSec2Sec = 1000000000UL; /**< .. */

  /**
   * @brief Sets internal time to current clock value (it uses @c CLOCK_MONOTONIC ).
   * @note For further details, click
   * <a href="https://linux.die.net/man/3/clock_gettime">here</a>.
   */
  void Reset();
  /**
   * @brief Return elapsed time since latest Reset() or object instantiation.
   * @return Elapsed time since latest Reset() in seconds.
   */
  double Elapsed() const { return Elapsed(time_); }
  /**
   * @brief Return elapsed time since given time.
   * @param start_time Reference for elapsed time.
   * @return Elapsed time in seconds since given time.
   */
  double Elapsed(const struct timespec& start_time) const;
  /**
   * @brief Return elapsed time since instantiation time.
   * @return Elapsed time since instantiation time.
   */
  double ElapsedFromStart() const { return Elapsed(start_time_); }

  /**
   * @brief Gets instance name.
   * @return A string reporting the instance name.
   */
  std::string GetName() const { return name_; }
  /**
   * @brief Gets current internal time.
   * @return Current internal time.
   * @note For further details about output time structure, click
   * <a href="https://en.cppreference.com/w/c/chrono/timespec">here</a>.
   * @see GetNextTime()
   */
  struct timespec GetCurrentTime() const { return time_; }

  /**
   * @brief Displays current internal time in a pretty format.
   */
  virtual void DispCurrentTime() const;

protected:
  std::string name_;     /**< .. */
  struct timespec time_; /**< .. */

  /**
    */
  [[noreturn]] void HandleErrorEnWrapper(const int en, const char* msg) const;

private:
  struct timespec start_time_;
};

/**
 * @brief This simple class can be used to keep track of time to perform cyclic tasks such
 * as
 * in real-time thread loops.
 * @todo usage example.
 */
class ThreadClock : public Clock
{
public:
  /**
   * @brief Constructor.
   * @param[in] cycle_time_nsec (Optional) Cycle period in nanoseconds.
   * @param[in] clk_name (Optional) Instance name.
   */
  ThreadClock(const uint64_t cycle_time_nsec = 1000000LL,
              const std::string& clk_name = "ThreadClock")
    : Clock(clk_name), period_nsec_(cycle_time_nsec)
  {
  }

  /**
   * @brief Set cycle time.
   * @param[in] cycle_time_nsec (Optional) Cycle period in nanoseconds.
   */
  void SetCycleTime(const uint64_t cycle_time_nsec) { period_nsec_ = cycle_time_nsec; }
  /**
   * @brief Sets internal time to current value + _cycle period_.
   * @see WaitUntilNext() GetNextTime()
   */
  void Next();
  /**
   * @brief Sets internal time to current value + _cycle period_ and sleep until that
   * time.
   * @note If request is less than or equal to the current value of the clock, then
   * returns
   * immediately without suspending the calling thread.
   * @note For further details, click
   * <a href="http://man7.org/linux/man-pages/man2/clock_nanosleep.2.html">here</a>.
   */
  bool WaitUntilNext();

  /**
   * @brief Gets the cycle period.
   * @return The cycle period in nanoseconds.
   */
  uint64_t GetCycleTime() const { return period_nsec_; }
  /**
   * @brief Sets internal time to current value + _cycle period_ and returns it.
   * @return Next internal time.
   * @note For further details about output time structure, click
   * <a href="https://en.cppreference.com/w/c/chrono/timespec">here</a>.
   * @see GetCurrentTime() GetNextTime()
   */
  struct timespec SetAndGetNextTime();
  /**
   * @brief Returns current value + _cycle period_.
   * @return Next internal time.
   * @note For further details about output time structure, click
   * <a href="https://en.cppreference.com/w/c/chrono/timespec">here</a>.
   * @see GetCurrentTime() SetAndGetNextTime()
   */
  struct timespec GetNextTime() const { return time_ + period_nsec_; }

  /**
   * @brief Displays current internal time in a pretty format.
   */
  void DispCurrentTime() const override;
  /**
   * @brief Sets internal time to current value + _cycle period_ and displays current
   * internal
   * time in a pretty format.
   */
  void DispNextTime();

private:
  uint64_t period_nsec_;
};

} // end namespace grabrt

#endif // GRABCOMMON_LIBGRABRT_CLOCKS_H
