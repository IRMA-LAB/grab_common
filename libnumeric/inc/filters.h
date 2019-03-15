/**
 * @file filters.h
 * @author Simone Comari
 * @date 15 Mar 2019
 * @brief This file include filters implementations.
 */

#ifndef GRABCOMMON_LIBNUMERIC_FILTERS_H
#define GRABCOMMON_LIBNUMERIC_FILTERS_H

/**
 * @brief Namespace for GRAB numeric library.
 */
namespace grabnum {

/**
 * @brief A low-pass filter implementation.
 */
class LowPassFilter
{
 public:
  /**
   * @brief LowPassFilter constructor.
   * @param cutoff_freq [Hz] Cutoff frequency of low-pass filter.
   * @param sampling_time_sec Sampling time in seconds.
   */
  LowPassFilter(const double& cutoff_freq, const double& sampling_time_sec);
  /**
   * @brief LowPassFilter full constructor.
   * @param cutoff_freq [Hz] Cutoff frequency of low-pass filter.
   * @param sampling_time_sec Sampling time in seconds.
   * @param init_value Initial value of the signal to be filtered.
   */
  LowPassFilter(const double& cutoff_freq, const double& sampling_time_sec,
                const double& init_value);

  /**
   * @brief Set cutoff frequency.
   * @param cutoff_freq [Hz] Cutoff frequency of low-pass filter.
   */
  void SetCutoffFreq(const double& cutoff_freq) { cutoff_freq_ = cutoff_freq; }
  /**
   * @brief Set sampling time.
   * @param sampling_time_sec Sampling time in seconds.
   */
  void SetDeltaTime(const double& sampling_time_sec) { Ts_ = sampling_time_sec; }

  /**
   * @brief Get cutoff frequency.
   * @return [Hz] Cutoff frequency of low-pass filter.
   */
  double GetCutoffFreq() const { return cutoff_freq_; }
  /**
   * @brief Get sampling time.
   * @return Sampling time in seconds.
   */
  double GetDeltaTime() const { return Ts_; }
  /**
   * @brief Get latest filtered value.
   * @return Latest filtered value.
   */
  double GetLatestFilteredValue() const { return filtered_value_; }

  /**
   * @brief Filter a new value of a signal.
   * @param raw_value New signal value.
   * @return Filtered value.
   */
  double Filter(const double& raw_value);
  /**
   * @brief Reset filter, clearing its history.
   */
  void Reset() { initialized_ = false; }

 private:
  double cutoff_freq_; // [Hz]
  double Ts_;          // [sec]
  double alpha_;

  bool initialized_;
  double filtered_value_;
};

} // end namespace grabnum

#endif // GRABCOMMON_LIBNUMERIC_FILTERS_H
