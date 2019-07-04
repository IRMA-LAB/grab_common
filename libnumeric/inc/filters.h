/**
 * @file filters.h
 * @author Simone Comari
 * @date 04 Jul 2019
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
   * @param[in] cutoff_freq [Hz] Cutoff frequency of low-pass filter.
   * @param[in] sampling_time_sec Sampling time in seconds.
   */
  LowPassFilter(const double& cutoff_freq, const double& sampling_time_sec);
  /**
   * @brief LowPassFilter full constructor.
   * @param[in] cutoff_freq [Hz] Cutoff frequency of low-pass filter.
   * @param[in] sampling_time_sec Sampling time in seconds.
   * @param[in] init_value Initial value of the signal to be filtered.
   */
  LowPassFilter(const double& cutoff_freq, const double& sampling_time_sec,
                const double& init_value);
  virtual ~LowPassFilter() {}

  /**
   * @brief Set LP filter cutoff frequency.
   * @param[in] cutoff_freq [Hz] Cutoff frequency of low-pass filter.
   * @param[in] reset If _True_ reinitialize the filter, i.e. clear its history.
   */
  void SetCutoffFreq(const double& cutoff_freq, bool reset = false);
  /**
   * @brief Set LP filter sampling time.
   * @param[in] sampling_time_sec Sampling time in seconds.
   * @param[in] reset If _True_ reinitialize the filter, i.e. clear its history.
   */
  void SetSamplingTime(const double& sampling_time_sec, bool reset = false);
  /**
   * @brief Set LP filter time constant.
   * @param[in] time_const_sec Time constant in seconds.
   * @param[in] reset If _True_ reinitialize the filter, i.e. clear its history.
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
   * @param[in] raw_value New signal value.
   * @return Filtered value.
   */
  virtual double Filter(const double& raw_value);
  /**
   * @brief Reset filter, clearing its history.
   */
  void Reset() { initialized_ = false; }

 protected:
  double cutoff_freq_; /**< Cut-off frequency in [Hz]. */
  double Ts_;          /**< Sampling time in [sec]. */
  double
    Tf_; /**< Filter time constant in [sec]. This is equal to @f$1/(2/pi f_{cutoff})@f$.*/
  double alpha_ = 0.0; /**< Time constants dependend filter factor. */

  bool initialized_;      /**< Boolean flag to check if filter is already initialized. */
  double filtered_value_; /**< Latest filtered value. */

  /**
   * @brief Update filter constants
   * @param[in] reset If _True_ reinitialize the filter, i.e. clear its history.
   */
  virtual void UpdateConstant(bool reset = false);
};


/**
 * @brief A second-order low-pass filter implementation.
 */
class SecondOrderLPFilter: LowPassFilter
{
 public:
  /**
   * @brief LowPassFilter constructor.
   * @param[in] cutoff_freq1 [Hz] First order cutoff frequency of low-pass filter.
   * @param[in] cutoff_freq2 [Hz] Second order cutoff frequency of low-pass filter.
   * @param[in] sampling_time_sec Sampling time in seconds.
   */
  SecondOrderLPFilter(const double& cutoff_freq1, const double& cutoff_freq2,
                      const double& sampling_time_sec);
  /**
   * @brief LowPassFilter full constructor.
   * @param[in] cutoff_freq1 [Hz] First order cutoff frequency of low-pass filter.
   * @param[in] cutoff_freq2 [Hz] Second order cutoff frequency of low-pass filter.
   * @param[in] sampling_time_sec Sampling time in seconds.
   * @param[in] init_value Initial value of the signal to be filtered.
   */
  SecondOrderLPFilter(const double& cutoff_freq1, const double& cutoff_freq2,
                      const double& sampling_time_sec, const double& init_value);

  /**
   * @brief Set LP filter cutoff frequencies.
   * @param[in] cutoff_freq [Hz] Cutoff frequencies of low-pass filter.
   * @param[in] reset If _True_ reinitialize the filter, i.e. clear its history.
   */
  void SetCutoffFreq(const std::vector<double>& cutoff_freq, bool reset = false);
  /**
   * @brief Set LP filter time constants.
   * @param[in] time_const_sec Time constants in seconds.
   * @param[in] reset If _True_ reinitialize the filter, i.e. clear its history.
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
   * @param[in] raw_value New signal value.
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
