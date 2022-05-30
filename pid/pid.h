/**
 * @file pid.h
 * @author Simone Comari
 * @date 13 Mar 2019
 * @brief This file include a simple implementation of a PID controller.
 */

#ifndef GRABCOMMON_PID_H
#define GRABCOMMON_PID_H

#include <assert.h>
#include <limits>

/**
 * @brief A structure including PID tuning and sample parameters.
 */
struct ParamsPID
{
  /**
   * @brief ParamsPID full constructor.
   * @param[in] _Kp Propotional gain.
   * @param[in] _Ki Integral gain.
   * @param[in] _Kd Derivative gain.
   * @param[in] _Tf Cutoff period on the derivative filtering action.
   * @param[in] _max Saturation upper bound.
   * @param[in] _min Saturation lower bound.
   */
  ParamsPID(const double& _Kp, const double& _Ki, const double& _Kd, const double& _Tf,
            const double& _max, const double& _min)
    : Kp(_Kp), Ki(_Ki), Kd(_Kd), Tf(_Tf), max(_max), min(_min)
  {}

  double Kp  = 0.0; /**< Propotional gain.*/
  double Ki  = 0.0; /**< Integral gain.*/
  double Kd  = 0.0; /**< Derivative gain.*/
  double Tf  = 0.0; /**< Cutoff period on the derivative filtering action.*/
  double max = std::numeric_limits<double>::infinity();  /**< Saturation upper bound.*/
  double min = -std::numeric_limits<double>::infinity(); /**< Saturation lower bound.*/
};

/**
 * @brief A simple synchronous PID controller with derivative filtering action.
 */
class PID
{
 public:
  /**
   * @brief PID default constructor.
   * @param[in] Ts Sampling time in seconds.
   */
  PID(const double& Ts);
  /**
   * @brief P controller constructor.
   * @param[in] Ts Sampling time in seconds.
   * @param[in] Kp Propotional gain.
   */
  PID(const double& Ts, const double& Kp);
  /**
   * @brief PI controller constructor.
   * @param[in] Ts Sampling time in seconds.
   * @param[in] Kp Propotional gain.
   * @param[in] Ki Integral gain.
   */
  PID(const double& Ts, const double& Kp, const double& Ki);
  /**
   * @brief PID controller constructor.
   * @param[in] Ts Sampling time in seconds.
   * @param[in] Kp Propotional gain.
   * @param[in] Ki Integral gain.
   * @param[in] Kd Derivative gain.
   */
  PID(const double& Ts, const double& Kp, const double& Ki, const double& Kd);
  /**
   * @brief PID controller with filtered derivative action constructor.
   * @param[in] Ts Sampling time in seconds.
   * @param[in] Kp Propotional gain.
   * @param[in] Ki Integral gain.
   * @param[in] Kd Derivative gain.
   * @param[in] Tf Cutoff period on the derivative filtering action.
   */
  PID(const double& Ts, const double& Kp, const double& Ki, const double& Kd,
      const double& Tf);
  /**
   * @brief PID controller with filtered derivative action constructor and saturation.
   * @param[in] Ts Sampling time in seconds.
   * @param[in] Kp Propotional gain.
   * @param[in] Ki Integral gain.
   * @param[in] Kd Derivative gain.
   * @param[in] Tf Cutoff period on the derivative filtering action.
   * @param[in] max Saturation upper bound.
   * @param[in] min Saturation lower bound.
   */
  PID(const double& Ts, const double& Kp, const double& Ki, const double& Kd,
      const double& Tf, const double& max, const double& min);
  /**
   * @brief PID implicit constructor.
   * @param[in] Ts Sampling time in seconds.
   * @param[in] params Tuning parameters.
   */
  PID(const double& Ts, const ParamsPID& params);

  /**
   * @brief Set/change controller parameters.
   * @param[in] params The new tuning parameters set.
   */
  void SetParams(const ParamsPID& params);

  /**
   * @brief Set/change controller parameters.
   * @param[in] params The new tuning parameters set, reset specify if the PID is to be reset.
   */
  void SetParams(const ParamsPID& params, const bool& reset);
  /**
   * @brief Get latest error.
   * @return Latest error.
   */
  double GetError() const { return pre_error_; }
  /**
   * @brief Get second latest error.
   * @return Second latest error.
   */
  double GetPrevError() const { return pre_pre_error_; }

  /**
   * @brief Returns the manipulated variable given a setpoint and current process value.
   * @param[in] setpoint Desired setpoint.
   * @param[in] current_value Current value.
   * @return The manipulated variable given a setpoint and current process value
   */
  double Calculate(const double& setpoint, const double& current_value);

  /**
   * @brief Reset the controller's priors.
   */
  void Reset();
  /**
   * @brief Reset the controller's priors and initialize previous outputs.
   * @param[in] init_val Initial value to initialize priors.
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
