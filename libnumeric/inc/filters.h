#ifndef GRABCOMMON_LIBNUMERIC_FILTERS_H
#define GRABCOMMON_LIBNUMERIC_FILTERS_H

namespace grabnum
{

class LowPassFilter
{
public:
  LowPassFilter(const double& cutoff_freq, const double& delta_time_sec);
  LowPassFilter(const double& cutoff_freq, const double& delta_time_sec,
                const double& init_value);

  void SetCutoffFreq(const double& cutoff_freq) { cutoff_freq_ = cutoff_freq; }
  void SetDeltaTime(const double& delta_time_sec) { delta_time_ = delta_time_sec; }

  double GetCutoffFreq() const { return cutoff_freq_; }
  double GetDeltaTime() const { return delta_time_; }
  double GetLatestFilteredValue() const { return filtered_value_; }

  double Filter(const double& raw_value);
  void Reset() { initialized_ = false; }

private:
  double cutoff_freq_; // [Hz]
  double delta_time_;  // [sec]
  double alpha_;

  bool initialized_;
  double filtered_value_;
};

} // end namespace grabnum

#endif // GRABCOMMON_LIBNUMERIC_FILTERS_H
