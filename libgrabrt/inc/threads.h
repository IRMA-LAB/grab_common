#ifndef GRABCOMMON_LIBGRABRT_THREADS_H
#define GRABCOMMON_LIBGRABRT_THREADS_H

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/syscall.h>
#include <iostream>
#include <vector>
#include <limits.h>

namespace grabrt
{

[[noreturn]] void HandleErrorEn(const int en, const char* msg);

cpu_set_t BuildCPUSet(const int cpu_core = 0);

cpu_set_t BuildCPUSet(const std::vector<size_t>& cpu_cores);

void SetThreadCPUs(const cpu_set_t& cpu_set, const pthread_t thread_id = pthread_self());

void SetThreadPolicy(const int policy, const int priority = -1,
                     const pthread_t thread_id = pthread_self());

class ThreadClock
{
public:
  ThreadClock(const uint64_t cycle_time_nsec = 1000LL,
              const std::string& clk_name = "ThreadClock")
    : name_(clk_name), period_nsec_(cycle_time_nsec)
  {
  }

  void SetCycleTime(const uint64_t cycle_time_nsec) { period_nsec_ = cycle_time_nsec; }
  void Reset() { clock_gettime(CLOCK_MONOTONIC, &time_); }
  void Next();
  void WaitUntilNext();

private:
  static constexpr uint64_t kNanoSec2Sec_ = 1000000000L;

  std::string name_;
  struct timespec time_;
  uint64_t period_nsec_;

  [[noreturn]] void HandleErrorEnWrapper(const int en, const char* msg) const;
};

class Thread
{
public:
  Thread(const std::string& thread_name = "Thread") : name_(thread_name) { InitDefault(); }
  Thread(pthread_attr_t& attr, const std::string& thread_name = "Thread")
    : name_(thread_name) { SetAttr(attr); }
  Thread(const cpu_set_t& cpu_set, const std::string& thread_name = "Thread");
  Thread(const int policy, const int priority = -1,
         const std::string& thread_name = "Thread");
  Thread(const cpu_set_t& cpu_set, const int policy, const int priority = -1,
         const std::string& thread_name = "Thread");
  ~Thread();

  void SetAttr(const pthread_attr_t& attr);
  void SetCPUs(const cpu_set_t& cpu_set);
  void SetCPUs(const int cpu_core = 0);
  void SetCPUs(const std::vector<size_t>& cpu_cores);
  void SetPolicy(const int policy, int priority = -1);

  void SetInitFunc(void (*fun_ptr)(void*), void* args);
  void SetLoopFunc(void (*fun_ptr)(void*), void* args);
  void SetEndFunc(void (*fun_ptr)(void*), void* args);

  long GetTID() const;
  pthread_t GetPID() const;
  cpu_set_t GetCPUs() const { return cpu_set_; }
  int GetPolicy() const;
  int GetPriority() const { return sched_param_.sched_priority; }
  pthread_attr_t GetAttr() const { return attr_; }

  void Start(const uint64_t cycle_time_nsec = 1000LL);
  void Stop() { run_ = false; }
  void Close();

  bool IsRunning() const { return run_; }
  bool IsActive() const { return active_; }

  void DispAttr() const;

private:
  static constexpr unsigned long kStackSize_ = 10 * 1024 * 1024; // 10 Mb

  std::string name_;
  long tid_;
  pthread_t thread_id_;
  pthread_attr_t attr_;
  struct sched_param sched_param_;
  cpu_set_t cpu_set_;
  uint64_t cycle_time_nsec_;

  void (*init_fun_ptr_)(void*) = NULL;
  void (*loop_fun_ptr_)(void*) = NULL;
  void (*end_fun_ptr_)(void*) = NULL;
  void* init_fun_args_ptr_ = NULL;
  void* loop_fun_args_ptr_ = NULL;
  void* end_fun_args_ptr_ = NULL;

  bool run_ = false;
  bool active_ = false;

  pthread_mutex_t mutex_ = PTHREAD_MUTEX_INITIALIZER;

  void InitDefault();
  void TargetFun();

  static void* StaticTargetFun(void* args)
  {
    static_cast<Thread*>(args)->TargetFun();
    return NULL;
  }

  [[noreturn]] void HandleErrorEnWrapper(const int en, const char* msg) const;
};

} // end namespace grabrt

#endif // GRABCOMMON_LIBGRABRT_THREADS_H
