/**
 * @file filters.h
 * @author Simone Comari
 * @date 10 Apr 2019
 * @brief This file include filters implementations.
 */

#ifndef GRABCOMMON_LIBNUMERIC_FILTERS_H
#define GRABCOMMON_LIBNUMERIC_FILTERS_H

#include <math.h>
#include <vector>

/**
 * @brief Namespace for GRAB numeric library.
 */
namespace grabnum {

/**
 * @brief A first-order low-pass filter implementation.
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
  virtual ~LowPassFilter() {}

  /**
   * @brief Set LP filter cutoff frequency.
   * @param cutoff_freq [Hz] Cutoff frequency of low-pass filter.
   */
  void SetCutoffFreq(const double& cutoff_freq, bool reset = false);
  /**
   * @brief Set LP filter sampling time.
   * @param sampling_time_sec Sampling time in seconds.
   */
  void SetSamplingTime(const double& sampling_time_sec, bool reset = false);
  /**
   * @brief Set LP filter time constant.
   * @param time_const_sec Time constant in seconds.
   */
  void SetTimeConstant(const double& time_const_sec, bool reset = false);

  /**
   * @brief Get LP filter cutoff frequency.
   * @return [Hz] Cutoff frequency of low-pass filter.
   */
  double GetCutoffFreq() const { return cutoff_freq_; }
  /**
   * @brief Get LP filter sampling time.
   * @return Sampling time in seconds.
   */
  double GetSamplingTime() const { return Ts_; }
  /**
   * @brief Get LP filter time constant.
   * @return LP filter time constant in seconds.
   */
  double GetTimeConstant() const { return Tf_; }
  /**
   * @brief Get latest filtered value.
   * @return Latest filtered value.h
   */
  double GetLatestFilteredValue() const { return filtered_value_; }

  /**
   * @brief Filter a new value of a signal.
   * @param raw_value New signal value.
   * @return Filtered value.
   */
  virtual double Filter(const double& raw_value);
  /**
   * @brief Reset filter, clearing its history.
   */
  void Reset() { initialized_ = false; }

 protected:
  double cutoff_freq_; // [Hz]
  double Ts_;          // [sec]
  double Tf_;          // [sec]
  double alpha_ = 0.0;

  bool initialized_;
  double filtered_value_;

  virtual void UpdateConstant(bool reset = false);
};

class SecondOrderLPFilter: LowPassFilter
{
 public:
  /**
   * @brief LowPassFilter constructor.
   * @param cutoff_freq [Hz] Cutoff frequency of low-pass filter.
   * @param sampling_time_sec Sampling time in seconds.
   */
  SecondOrderLPFilter(const double& cutoff_freq1, const double& cutoff_freq2,
                      const double& sampling_time_sec);
  /**
   * @brief LowPassFilter full constructor.
   * @param cutoff_freq [Hz] Cutoff frequency of low-pass filter.
   * @param sampling_time_sec Sampling time in seconds.
   * @param init_value Initial value of the signal to be filtered.
   */
  SecondOrderLPFilter(const double& cutoff_freq, const double& cutoff_freq2,
                      const double& sampling_time_sec, const double& init_value);

  /**
   * @brief Set LP filter cutoff frequencies.
   * @param cutoff_freq [Hz] Cutoff frequencies of low-pass filter.
   */
  void SetCutoffFreq(const std::vector<double>& cutoff_freq, bool reset = false);
  /**
   * @brief Set LP filter time constants.
   * @param time_const_sec Time constants in seconds.
   */
  void SetTimeConstant(const std::vector<double>& time_const_sec, bool reset = false);

  /**
   * @brief Get LP filter cutoff frequencies.
   * @return [Hz] Cutoff frequencies of low-pass filter.
   */
  std::vector<double> GetCutoffFreq() const { return {cutoff_freq_, cutoff_freq2_}; }
  /**
   * @brief Get LP filter time constants.
   * @return LP filter time constants in seconds.
   */
  std::vector<double> GetTimeConstant() const { return {Tf_, Tf2_}; }

  /**
   * @brief Filter a new value of a signal.
   * @param raw_value New signal value.
   * @return Filtered value.
   */
  double Filter(const double& raw_value) override;

 private:
  double cutoff_freq2_; // [Hz]
  double Tf2_;          // [sec]
  double beta_ = 0.0;

  double prev_filt_value_ = 0.0;

  void UpdateConstant(bool reset = false) override;

  using LowPassFilter::GetCutoffFreq;
  using LowPassFilter::GetTimeConstant;
  using LowPassFilter::SetCutoffFreq;
  using LowPassFilter::SetTimeConstant;
};

} // end namespace grabnum

#endif // GRABCOMMON_LIBNUMERIC_FILTERS_H
