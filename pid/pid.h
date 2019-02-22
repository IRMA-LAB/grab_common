#ifndef GRABCOMMON_PID_H
#define GRABCOMMON_PID_H

#include <limits>
#include <assert.h>

struct ParamsPID
{
  ParamsPID(const double& _Kp, const double& _Ki, const double& _Kd, const double& _Tf,
            const double& _max, const double& _min)
    : Kp(_Kp), Ki(_Ki), Kd(_Kd), Tf(_Tf), max(_max), min(_min)
  {}

  double Kp  = 0.0;
  double Ki  = 0.0;
  double Kd  = 0.0;
  double Tf  = 0.0;
  double max = std::numeric_limits<double>::infinity();
  double min = -std::numeric_limits<double>::infinity();
};


class PID
{
 public:
  PID(const double& Ts);
  PID(const double& Ts, const double& Kp);
  PID(const double& Ts, const double& Kp, const double& Ki);
  PID(const double& Ts, const double& Kp, const double& Ki, const double& Kd);
  PID(const double& Ts, const double& Kp, const double& Ki, const double& Kd,
      const double& Tf);
  PID(const double& Ts, const double& Kp, const double& Ki, const double& Kd,
      const double& Tf, const double& max, const double& min);
  PID(const double& Ts, const ParamsPID& params);

  /**
   * @brief SetParams
   * @param params
   */
  void SetParams(const ParamsPID& params);

  /**
   * @brief GetError
   * @return
   */
  double GetError() const { return pre_error_; }
  /**
   * @brief GetPrevError
   * @return
   */
  double GetPrevError() const { return pre_pre_error_; }

  /**
   * @brief Returns the manipulated variable given a setpoint and current process value.
   * @param setpoint
   * @param current_value
   * @return The manipulated variable given a setpoint and current process value
   */
  double Calculate(const double& setpoint, const double& current_value);

  /**
   * @brief Reset the controller's priors.
   */
  void Reset();
  /**
   * @brief Reset the controller's priors and initialize previous outputs.
   * @param init_val
   */
  void Reset(const double& init_val);

 private:
  double Ts_;
  double Kp_  = 0.0;
  double Ki_  = 0.0;
  double Kd_  = 0.0;
  double Tf_  = 0.0;
  double max_ = std::numeric_limits<double>::infinity();
  double min_ = -std::numeric_limits<double>::infinity();

  bool initialized_      = false;
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

#endif // GRABCOMMON_PID_H
