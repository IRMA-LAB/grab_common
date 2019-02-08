#ifndef _GRABCOMMON_PID_H_
#define _GRABCOMMON_PID_H_

#include <limits>

class PID
{
 public:
  PID(const double& Ts, const double& Kp);
  PID(const double& Ts, const double& Kp, const double& Kd);
  PID(const double& Ts, const double& Kp, const double& Kd, const double& Ki);
  PID(const double& Ts, const double& Kp, const double& Kd, const double& Ki,
      const double& Tf);
  PID(const double& Ts, const double& Kp, const double& Kd, const double& Ki,
      const double& Tf, const double& max, const double& min);

  /**
   * @brief Returns the manipulated variable given a setpoint and current process value.
   * @param setpoint
   * @param current_value
   * @return The manipulated variable given a setpoint and current process value
   */
  double Calculate(const double& setpoint, const double& current_value);

 private:
  double Ts_;
  double Kp_  = 0.0;
  double Kd_  = 0.0;
  double Ki_  = 0.0;
  double Tf_  = 0.0;
  double max_ = std::numeric_limits<double>::infinity();
  double min_ = -std::numeric_limits<double>::infinity();

  double pre_error_      = 0.0;
  double pre_pre_error_  = 0.0;
  double pre_output_     = 0.0;
  double pre_pre_output_ = 0.0;

  double a_;
  double b_;
  double c_;
  double d_;
  double e_;

  void ComputeConstants();
};

#endif
