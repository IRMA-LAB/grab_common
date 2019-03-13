/**
 * @file filters.cpp
 * @author Simone Comari
 * @date 13 Mar 2019
 * @brief This file includes definitions of class declared in filters.h.
 */

#include "filters.h"

namespace grabnum {

LowPassFilter::LowPassFilter(const double& cutoff_freq, const double& sampling_time_sec)
  : cutoff_freq_(cutoff_freq), Ts_(sampling_time_sec),
    alpha_(cutoff_freq * sampling_time_sec), initialized_(false)
{}

LowPassFilter::LowPassFilter(const double& cutoff_freq, const double& sampling_time_sec,
                             const double& init_value)
  : cutoff_freq_(cutoff_freq), Ts_(sampling_time_sec),
    alpha_(cutoff_freq * sampling_time_sec), initialized_(true),
    filtered_value_(init_value)
{}

//--------- Public functions ---------------------------------------------------------//

double LowPassFilter::Filter(const double& raw_value)
{
  if (initialized_)
    filtered_value_ += alpha_ * (raw_value - filtered_value_);
  else
  {
    filtered_value_ = raw_value;
    initialized_    = true;
  }
  return filtered_value_;
}

} // end namespace grabnum
