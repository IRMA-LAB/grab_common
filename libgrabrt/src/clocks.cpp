/**
 * @file clocks.cpp
 * @author Simone Comari
 * @date 25 Gen 2019
 * @brief File containing definitions of functions and class declared in clocks.h.
 */

#include "clocks.h"

struct timespec operator+(const struct timespec& time1, const struct timespec& time2)
{
  struct timespec total_time;
  total_time.tv_sec =
    time1.tv_sec + time2.tv_sec + (time1.tv_nsec + time2.tv_nsec) / 1000000000L;
  total_time.tv_nsec = static_cast<int64_t>(time1.tv_nsec + time2.tv_nsec) % 1000000000L;
  return total_time;
}

struct timespec operator+(const struct timespec& time1, const double& time_sec)
{
  struct timespec total_time;
  int64_t whole = static_cast<int64_t>(time_sec);
  int64_t decimal = static_cast<int64_t>((time_sec - whole) * 1000000000L);
  total_time.tv_sec = time1.tv_sec + whole + (time1.tv_nsec + decimal) / 1000000000L;
  total_time.tv_nsec = (time1.tv_nsec + decimal) % 1000000000L;
  return total_time;
}

struct timespec operator+(const struct timespec& time1, const int64_t& time_nsec)
{
  struct timespec total_time;
  total_time.tv_sec = time1.tv_sec + (time1.tv_nsec + time_nsec) / 1000000000L;
  total_time.tv_nsec = (time1.tv_nsec + time_nsec) % 1000000000L;
  return total_time;
}

struct timespec operator+(const struct timespec& time1, const uint64_t& time_nsec)
{
  return time1 + static_cast<int64_t>(time_nsec);
}

namespace grabrt
{

uint64_t Sec2NanoSec(const double seconds)
{
  return static_cast<uint64_t>(seconds * 1000000000);
}

double NanoSec2Sec(const long nanoseconds)
{
  return static_cast<double>(nanoseconds) * 0.000000001;
}

/////////////////////////////////////////////////
/// Clock Class Methods
/////////////////////////////////////////////////

void Clock::Reset()
{
  //  printf("[%s] RESET\n", name_.c_str());
  clock_gettime(CLOCK_MONOTONIC, &time_);
}

double Clock::Elapsed(const struct timespec& start_time) const
{
  static struct timespec end;
  clock_gettime(CLOCK_MONOTONIC, &end);
  return end.tv_sec - start_time.tv_sec +
         (end.tv_nsec - start_time.tv_nsec) / static_cast<double>(kNanoSec2Sec);
}

void Clock::DispCurrentTime() const
{
  printf("%s status:\n\ttime =\t%lu.%ld sec\n", name_.c_str(), time_.tv_sec,
         time_.tv_nsec);
}

[[noreturn]] void Clock::HandleErrorEnWrapper(const int en, const char* msg) const
{
  std::string full_msg = "[";
  full_msg.append(name_);
  full_msg.append("] ");
  full_msg.append(msg);
  HandleErrorEn(en, full_msg.c_str());
}

/////////////////////////////////////////////////
/// ThreadClock Class Methods
/////////////////////////////////////////////////

void ThreadClock::Next()
{
  time_.tv_sec += (static_cast<uint64_t>(time_.tv_nsec) + period_nsec_) / kNanoSec2Sec;
  time_.tv_nsec = (static_cast<uint64_t>(time_.tv_nsec) + period_nsec_) % kNanoSec2Sec;
}

bool ThreadClock::WaitUntilNext()
{
  Next();
  if (Elapsed() > 0)
    return false;
  int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &time_, NULL);
  if (ret != 0)
    HandleErrorEnWrapper(ret, "clock_nanosleep ");
  return true;
}

struct timespec ThreadClock::SetAndGetNextTime()
{
  Next();
  return GetCurrentTime();
}

void ThreadClock::DispCurrentTime() const
{
  printf("%s status:\n\ttime =\t%lu.%ld sec\n\tperiod =\t%lu\n", name_.c_str(),
         time_.tv_sec, time_.tv_nsec, period_nsec_);
}

void ThreadClock::DispNextTime()
{
  Next();
  DispCurrentTime();
}

} // end namespace grabrt
