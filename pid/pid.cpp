#include "pid/pid.h"


PID::PID(const double& Ts, const double& Kp) : Ts_(Ts), Kp_(Kp) { ComputeConstants(); }

PID::PID(const double& Ts, const double& Kp, const double& Kd) : Ts_(Ts), Kp_(Kp), Kd_(Kd)
{
  ComputeConstants();
}

PID::PID(const double& Ts, const double& Kp, const double& Kd, const double& Ki)
  : Ts_(Ts), Kp_(Kp), Kd_(Kd), Ki_(Ki)
{
  ComputeConstants();
}

PID::PID(const double& Ts, const double& Kp, const double& Kd, const double& Ki,
         const double& Tf)
  : Ts_(Ts), Kp_(Kp), Kd_(Kd), Ki_(Ki), Tf_(Tf)
{
  ComputeConstants();
}

PID::PID(const double& Ts, const double& Kp, const double& Kd, const double& Ki,
         const double& Tf, const double& max, const double& min)
  : Ts_(Ts), Kp_(Kp), Kd_(Kd), Ki_(Ki), Tf_(Tf), max_(max), min_(min)
{
  ComputeConstants();
}

double PID::Calculate(const double& setpoint, const double& current_value)
{
  // Calculate error
  double error = setpoint - current_value;

  // Calculate total output
  double output = a_ * pre_output_ + b_ * pre_pre_output_ + c_ * error + d_ * pre_error_ +
                  e_ * pre_pre_error_;

  // Restrict to max/min
  if (output > max_)
    output = max_;
  else if (output < min_)
    output = min_;

  // Update previous values
  pre_pre_error_  = pre_error_;
  pre_error_      = error;
  pre_pre_output_ = pre_output_;
  pre_output_     = output;

  return output;
}

void PID::ComputeConstants()
{
  double A = Kp_ * Tf_ + Kd_ + Kp_ * Ts_ + Ki_ * Ts_ * Tf_ + Ki_ * Ts_ * Ts_;
  double B = -(2. * (Kp_ * Tf_ + Kd_) + Kp_ * Ts_ + Ki_ * Ts_ * Tf_);
  double C = Kp_ * Tf_ + Kd_;
  double D = Tf_ + Ts_;

  a_ = 1. + Tf_/ D;
  b_ = -Tf_ / D;
  c_ = A / D;
  d_ = B / D;
  e_ = C / D;
}
