/**
 * @file filters.cpp
 * @author Simone Comari
 * @date 10 Apr 2019
 * @brief This file includes definitions of class declared in filters.h.
 */

#include "filters.h"

namespace grabnum {

//------------------------------------------------------------------------------------//
//--------- LowPassFilter class ------------------------------------------------------//
//------------------------------------------------------------------------------------//

LowPassFilter::LowPassFilter(const double& cutoff_freq, const double& sampling_time_sec)
  : cutoff_freq_(cutoff_freq), Ts_(sampling_time_sec), initialized_(false)
{
  Tf_ = 1. / (2. * M_PI * cutoff_freq_); // time constant
  UpdateConstant(true);
}

LowPassFilter::LowPassFilter(const double& cutoff_freq, const double& sampling_time_sec,
                             const double& init_value)
  : cutoff_freq_(cutoff_freq), Ts_(sampling_time_sec), initialized_(true),
    filtered_value_(init_value)
{
  Tf_ = 1. / (2. * M_PI * cutoff_freq_); // time constant
  UpdateConstant(false);
}

//--------- Public functions ---------------------------------------------------------//

void LowPassFilter::SetCutoffFreq(const double& cutoff_freq, bool reset /*=false*/)
{
  cutoff_freq_ = cutoff_freq;
  Tf_          = 1. / (2. * M_PI * cutoff_freq_); // time constant
  UpdateConstant(reset);
}

void LowPassFilter::SetSamplingTime(const double& sampling_time_sec,
                                    bool reset /*=false*/)
{
  Ts_ = sampling_time_sec;
  UpdateConstant(reset);
}

void LowPassFilter::SetTimeConstant(const double& time_const_sec, bool reset /*=false*/)
{
  Tf_          = time_const_sec;
  cutoff_freq_ = 1. / (2. * M_PI * Tf_);
  UpdateConstant(reset);
}

double LowPassFilter::Filter(const double& raw_value)
{
  if (initialized_)
    // y_k = (1 - a) * y_k-1 + a * x_k
    filtered_value_ += alpha_ * (raw_value - filtered_value_);
  else
  {
    filtered_value_ = raw_value;
    initialized_    = true;
  }
  return filtered_value_;
}

//--------- Private functions --------------------------------------------------------//

void LowPassFilter::UpdateConstant(bool reset /*= false*/)
{
  alpha_ = Ts_ / (Tf_ + Ts_);
  if (reset)
    Reset();
}

//------------------------------------------------------------------------------------//
//--------- SecondOrderLPFilter class ------------------------------------------------//
//------------------------------------------------------------------------------------//

SecondOrderLPFilter::SecondOrderLPFilter(const double& cutoff_freq1,
                                         const double& cutoff_freq2,
                                         const double& sampling_time_sec)
  : LowPassFilter(cutoff_freq1, sampling_time_sec), cutoff_freq2_(cutoff_freq2)
{
  Tf2_ = 1. / (2. * M_PI * cutoff_freq2_); // time constant
  UpdateConstant(true);
}

SecondOrderLPFilter::SecondOrderLPFilter(const double& cutoff_freq1,
                                         const double& cutoff_freq2,
                                         const double& sampling_time_sec,
                                         const double& init_value)
  : LowPassFilter(cutoff_freq1, sampling_time_sec, init_value),
    cutoff_freq2_(cutoff_freq2)
{
  Tf2_             = 1. / (2. * M_PI * cutoff_freq2_); // time constant
  prev_filt_value_ = init_value;
  UpdateConstant(false);
}

//--------- Public functions ---------------------------------------------------------//

void SecondOrderLPFilter::SetCutoffFreq(const std::vector<double>& cutoff_freq,
                                        bool reset /*= false*/)
{
  cutoff_freq2_ = cutoff_freq[1];
  Tf2_          = 1. / (2. * M_PI * cutoff_freq2_); // time constant
  LowPassFilter::SetCutoffFreq(cutoff_freq[0], reset);
}

void SecondOrderLPFilter::SetTimeConstant(const std::vector<double>& time_const_sec,
                                          bool reset /*= false*/)
{
  Tf2_          = time_const_sec[1];
  cutoff_freq2_ = 1. / (2. * M_PI * Tf2_);
  LowPassFilter::SetTimeConstant(time_const_sec[0], reset);
}

double SecondOrderLPFilter::Filter(const double& raw_value)
{
  if (initialized_)
  {
    // Equivalent to: z_n = (α+β)*z_n−1 + α*β*z_n−2 + (1−α)(1−β)*x_n
    double filtered_value = (2 - alpha_ - beta_) * filtered_value_ +
                            (1 - alpha_) * (1 - beta_) * prev_filt_value_ +
                            alpha_ * beta_ * raw_value;
    prev_filt_value_ = filtered_value_;
    filtered_value_  = filtered_value;
  }
  else
  {
    prev_filt_value_ = raw_value;
    filtered_value_  = raw_value;
    initialized_     = true;
  }
  return filtered_value_;
}

//--------- Private functions --------------------------------------------------------//

void SecondOrderLPFilter::UpdateConstant(bool reset /*= false*/)
{
  beta_ = Ts_ / (Tf2_ + Ts_);
  LowPassFilter::UpdateConstant(reset);
}

} // end namespace grabnum
