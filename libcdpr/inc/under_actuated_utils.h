/**
 * @file under_actuated_utils.h
 * @author Simone Comari
 * @date 06 Feb 2020
 * @brief This file include several functions and structures strictly related to
 * under-actuated CDPRs.
 */

#ifndef GRABCOMMON_LIBCDPR_UNDER_ACTUATED_UTILS_H
#define GRABCOMMON_LIBCDPR_UNDER_ACTUATED_UTILS_H

#include "cdpr_types.h"
#include "kinematics.h"

/**
 * @brief Namespace for CDPR-related utilities, such as kinematics and dynamics.
 */
namespace grabcdpr {

/**
 * @brief An extended version of PlatformVars structure, including useful fields for the
 * under-actuated case.
 *
 * In particular, thanks to a mask is possible to map all variables into actuated and
 * unactuated for dedicated calculations that take advantage of this distinction.
 */
struct UnderActuatedPlatformVars: PlatformVars
{
  arma::uvec6 mask;

  /** @addtogroup ZeroOrderKinematics
   * @{
   */
  arma::vec actuated_vars;   /**< actuated elements of PlatformVars::pose. */
  arma::vec unactuated_vars; /**< unactuated elements of PlatformVars::pose. */
  /** @} */                  // end of ZeroOrderKinematics group

  /** @addtogroup FirstOrderKinematics
   * @{
   */
  arma::vec actuated_deriv;   /**< actuated elements of PlatformVars::velocity. */
  arma::vec unactuated_deriv; /**< unactuated elements of PlatformVars::velocity. */
  /** @} */                   // end of FirstOrderKinematics group

  /** @addtogroup SecondOrderKinematics
   * @{
   */
  arma::vec actuated_deriv_2;   /**< actuated elements of PlatformVars::acceleration. */
  arma::vec unactuated_deriv_2; /**< unactuated elements of PlatformVars::acceleration. */
  /** @} */                     // end of SecondOrderKinematics group

  /** @addtogroup Dynamics
   * @{
   */
  arma::mat
    mass_matrix_global_a; /**< actuated elements of PlatformVars::mass_mat_glob. */
  arma::mat
    mass_matrix_global_u; /**< unactuated elements of PlatformVars::mass_mat_glob. */

  arma::mat
    mass_matrix_global_ss_a; /**< actuated elements of PlatformVars::mass_mat_glob_ss. */
  arma::mat mass_matrix_global_ss_u; /**< unactuated elements of
                                        PlatformVars::mass_mat_glob_ss. */

  arma::vec external_load_a; /**< actuated elements of PlatformVars::ext_load. */
  arma::vec external_load_u; /**< unactuated elements of PlatformVars::ext_load. */

  arma::vec external_load_ss_a; /**< actuated elements of PlatformVars::ext_load_ss. */
  arma::vec external_load_ss_u; /**< unactuated elements of PlatformVars::ext_load_ss. */

  arma::vec total_load_a; /**< actuated elements of PlatformVars::total_load. */
  arma::vec total_load_u; /**< unactuated elements of PlatformVars::total_load. */

  arma::vec total_load_ss_a; /**< actuated elements of PlatformVars::total_load_ss. */
  arma::vec total_load_ss_u; /**< unactuated elements of PlatformVars::total_load_ss. */
  /** @} */                  // end of Dynamics group

  /**
   * @brief Default constructor, possibly predefining the actuation mask.
   * @param[in] _mask The mask defining which elements are actuated and which are not. If
   * not given this is equivalent to a standard fully actuated platform.
   */
  UnderActuatedPlatformVars(const arma::uvec6& _mask = arma::uvec6(arma::fill::ones));
  /**
   * @brief Constructor for initializing all actuated/unactuated variables from a standard
   * platform according to a given mask.
   * @param[in] vars Variable of a standard platform.
   * @param[in] _mask The mask defining which elements are actuated and which are not. If
   * not given this is equivalent to a standard fully actuated platform.
   */
  UnderActuatedPlatformVars(const PlatformVars vars,
                            const arma::uvec6& _mask = arma::uvec6(arma::fill::ones));
  /**
   * @brief Constructor to possibly define both mask and rotation parametrization.
   * @param[in] _angles_type Desired rotation parametrization.
   * @param[in] _mask The mask defining which elements are actuated and which are not. If
   * not given this is equivalent to a standard fully actuated platform.
   */
  UnderActuatedPlatformVars(const RotParametrization _angles_type = TILT_TORSION,
                            const arma::uvec6& _mask = arma::uvec6(arma::fill::ones));
  /**
   * @brief Constructor to initialize all platform zero-order variables from position,
   * angles and a suitable mask.
   * @param[in] _position [m] Platform global position @f$\mathbf{p}_P@f$.
   * @param[in] _orientation [rad] Platform global orientation expressed by angles
   * @f$\boldsymbol{\varepsilon}@f$.
   * @param[in] _angles_type Desired rotation parametrization.
   * @param[in] _mask The mask defining which elements are actuated and which are not. If
   * not given this is equivalent to a standard fully actuated platform.
   */
  UnderActuatedPlatformVars(const grabnum::Vector3d& _position,
                            const grabnum::Vector3d& _orientation,
                            const RotParametrization _angles_type = TILT_TORSION,
                            const arma::uvec6& _mask = arma::uvec6(arma::fill::ones));
  /**
   * @brief Constructor to initialize all platform zero and first-order variables from
   * position, angles, their first derivatives and a suitable mask.
   * @param[in] _position [m] Platform global position @f$\mathbf{p}_P@f$.
   * @param[in] _velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}_P@f$.
   * @param[in] _orientation [rad] Platform global orientation expressed by angles
   * @f$\boldsymbol{\varepsilon}@f$.
   * @param[in] _orientation_dot [rad/s] Platform orientation time-derivative
   * @f$\dot{\boldsymbol{\varepsilon}}@f$.
   * @param[in] _angles_type Desired rotation parametrization.
   * @param[in] _mask The mask defining which elements are actuated and which are not. If
   * not given this is equivalent to a standard fully actuated platform.
   */
  UnderActuatedPlatformVars(const grabnum::Vector3d& _position,
                            const grabnum::Vector3d& _velocity,
                            const grabnum::Vector3d& _orientation,
                            const grabnum::Vector3d& _orientation_dot,
                            const RotParametrization _angles_type = TILT_TORSION,
                            const arma::uvec6& _mask = arma::uvec6(arma::fill::ones));
  /**
   * @brief Constructor to initialize all platform zero, first and second-order variables
   * from position, angles, their first and second derivatives and a suitable mask.
   * @param[in] _position [m] Platform global position @f$\mathbf{p}_P@f$.
   * @param[in] _velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}_P@f$.
   * @param[in] _acceleration [m/s<sup>2</sup>] Platform global linear acceleration
   * @f$\ddot{\mathbf{p}}_P@f$.
   * @param[in] _orientation [rad] Platform global orientation expressed by angles
   * @f$\boldsymbol{\varepsilon}@f$.
   * @param[in] _orientation_dot [rad/s] Platform orientation time-derivative
   * @f$\dot{\boldsymbol{\varepsilon}}@f$.
   * @param[in] _orientation_ddot [rad/s<sup>2</sup>] Platform orientation 2nd
   * time-derivative @f$\ddot{\boldsymbol{\varepsilon}}@f$.
   * @param[in] _angles_type Desired rotation parametrization.
   * @param[in] _mask The mask defining which elements are actuated and which are not. If
   * not given this is equivalent to a standard fully actuated platform.
   */
  UnderActuatedPlatformVars(const grabnum::Vector3d& _position,
                            const grabnum::Vector3d& _velocity,
                            const grabnum::Vector3d& _acceleration,
                            const grabnum::Vector3d& _orientation,
                            const grabnum::Vector3d& _orientation_dot,
                            const grabnum::Vector3d& _orientation_ddot,
                            const RotParametrization _angles_type = TILT_TORSION,
                            const arma::uvec6& _mask = arma::uvec6(arma::fill::ones));
  /**
   * @brief Set platform pose and distribute it to actuated and unactuated corresponding
   * field.
   * @param[in] pose Platform pose, including both position and orientation.
   * @ingroup ZeroOrderKinematics
   * @see UpdatePose()
   */
  void updatePose(const grabnum::VectorXd<POSE_DIM>& _pose);
  /**
   * @brief Update platform pose with position and angles and distribute it to actuated
   * and unactuated corresponding field.
   * @param[in] _position [m] Platform global position @f$\mathbf{p}_P@f$.
   * @param[in] _orientation [rad] Platform global orientation expressed by angles
   * @f$\boldsymbol{\varepsilon}@f$.
   * @todo handle default case better
   * @ingroup ZeroOrderKinematics
   * @see UpdateVel() UpdateAcc() Update()
   * @note See @ref legend for more details.
   */
  void updatePose(const grabnum::Vector3d& _position,
                  const grabnum::Vector3d& _orientation);
  /**
   * @brief Update platform pose picking elements of given actuated and unactuated
   * variables and sorting them according to current mask.
   * @param _actuated_vars Actuated elements of PlatformVars::pose.
   * @param _unactuated_vars Unactuated elements of PlatformVars::pose.
   * @ingroup ZeroOrderKinematics
   * @see UpdatePose()
   */
  void updatePose(const arma::vec& _actuated_vars, const arma::vec& _unactuated_vars);

  /**
   * @brief Update platform velocities with linear velocity and angles speed and
   * distribute it to actuated and unactuated corresponding field.
   * @param[in] _velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}_P@f$.
   * @param[in] _orientation_dot [rad/s] Vector @f$\dot{\boldsymbol{\varepsilon}}@f$.
   * @param[in] _orientation [rad] Platform global orientation expressed by angles
   * @f$\boldsymbol{\varepsilon}@f$.
   * @ingroup FirstOrderKinematics
   * @see UpdatePose() UpdateAcc() Update()
   * @note See @ref legend for more details.
   */
  void updateVel(const grabnum::Vector3d& _velocity,
                 const grabnum::Vector3d& _orientation_dot,
                 const grabnum::Vector3d& _orientation);
  /**
   * @brief Update platform velocities with linear velocity and angles speed and
   * distribute it to actuated and unactuated corresponding field.
   *
   * Platform orientation needed for the update are infered from current values of
   * structure members, so make sure it is up-to-date by using UpdatePose() first.
   * @param[in] _velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}_P@f$.
   * @param[in] _orientation_dot [rad/s] Vector @f$\dot{\boldsymbol{\varepsilon}}@f$.
   * @ingroup FirstOrderKinematics
   * @see UpdateVel()
   * @note See @ref legend for more details.
   */
  void updateVel(const grabnum::Vector3d& _velocity,
                 const grabnum::Vector3d& _orientation_dot);

  /**
   * @brief Update platform accelerations with linear and angles acceleration and
   * distribute it to actuated and unactuated corresponding field.
   * @param[in] _acceleration [m/s<sup>2</sup>] Vector @f$\ddot{\mathbf{p}}_P@f$.
   * @param[in] _orientation_ddot [rad/s<sup>2</sup>] Vector
   * @f$\ddot{\boldsymbol{\varepsilon}}@f$.
   * @param[in] _orientation_dot [rad/s] Vector @f$\dot{\boldsymbol{\varepsilon}}@f$.
   * @param[in] _orientation [rad] Platform global orientation expressed by angles
   * @f$\boldsymbol{\varepsilon}@f$.
   * @param[in] _h_mat Transformation matrix @f$\mathbf{H}@f$.
   * @ingroup SecondOrderKinematics
   * @see UpdateVel() UpdatePose() Update()
   * @note See @ref legend for more details.
   */
  void updateAcc(const grabnum::Vector3d& _acceleration,
                 const grabnum::Vector3d& _orientation_ddot,
                 const grabnum::Vector3d& _orientation_dot,
                 const grabnum::Vector3d& _orientation, const grabnum::Matrix3d& _h_mat);
  /**
   * @brief Update platform accelerations with linear and quaternion acceleration and
   * distribute it to actuated and unactuated corresponding field.
   *
   * Platform orientation and angular velocity needed for the update are infered from
   * current values of structure members, so make sure they are up-to-date by using
   * UpdateVel() first.
   * @param[in] _acceleration [m/s<sup>2</sup>] Vector @f$\ddot{\mathbf{p}}_P@f$.
   * @param[in] _orientation_ddot [rad/s<sup>2</sup>] Vector
   * @f$\ddot{\boldsymbol{\varepsilon}}@f$.
   * @ingroup SecondOrderKinematics
   * @see UpdateAcc()
   * @note See @ref legend for more details.
   */
  void updateAcc(const grabnum::Vector3d& _acceleration,
                 const grabnum::Vector3d& _orientation_ddot);

  /**
   * @brief Update platform vars with position and angles and their first and second
   * derivatives and distribute it to actuated and unactuated corresponding field.
   * @param[in] _position [m] Platform global position @f$\mathbf{p}_P@f$.
   * @param[in] _velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}_P@f$.
   * @param[in] _acceleration [m/s<sup>2</sup>] Platform global linear acceleration
   * @f$\ddot{\mathbf{p}}_P@f$.
   * @param[in] _orientation [rad] Platform global orientation expressed by angles
   * @f$\boldsymbol{\varepsilon}@f$.
   * @param[in] _orientation_dot [rad/s] Platform orientation time-derivative
   * @f$\dot{\boldsymbol{\varepsilon}}@f$.
   * @param[in] _orientation_ddot [rad/s<sup>2</sup>] Platform orientation 2nd
   * time-derivative
   * @f$\ddot{\boldsymbol{\varepsilon}}@f$.
   * @note See @ref legend for more details.
   * @see UpdatePose() UpdateVel() UpdateAcc()
   */
  void update(const grabnum::Vector3d& _position, const grabnum::Vector3d& _velocity,
              const grabnum::Vector3d& _acceleration,
              const grabnum::Vector3d& _orientation,
              const grabnum::Vector3d& _orientation_dot,
              const grabnum::Vector3d& _orientation_ddot);

  /**
   * @brief Set actuation mask.
   * @param[in] _mask Actuation binary mask, where 1 means _actuated_ and 0 means
   * _unactuated_.
   */
  void setMask(const arma::uvec6& _mask);
};

/**
 * @brief Structure collecting all variables related to an under-actuated 6DoF CDPR.
 * @note This structure employs 3-angle parametrization for the orientation of the
 * platform.
 */
struct UnderActuatedRobotVars
{
  UnderActuatedPlatformVars
    platform;                    /**< variables of a generic 6DoF platform with angles. */
  std::vector<CableVars> cables; /**< vector of variables of a single cables in a CDPR. */

  /** @addtogroup ZeroOrderKinematics
   * @{
   */
  arma::mat geom_jacobian;   /**< geometric jacobian. */
  arma::mat geom_jacobian_a; /**< columns of geometric jacobian corresponsing to actuated
                                 variables. */
  arma::mat geom_jacobian_u; /**< columns of geometric jacobian corresponsing to
                                unactuated variables. */
  arma::mat geom_orthogonal; /**< geometric jacobian orthogonal. */

  arma::mat anal_jacobian;   /**< analytical jacobian. */
  arma::mat anal_jacobian_a; /**< columns of analytical jacobian corresponsing to actuated
                                variables. */
  arma::mat anal_jacobian_u; /**< columns of analytical jacobian corresponsing to
                                unactuated variables. */
  arma::mat anal_orthogonal; /**< analytical jacobian orthogonal. */

  arma::mat gamma_mat; /** @f$\boldsymbol{\Gamma}}@f$ matrix. */
  /** @} */            // end of ZeroOrderKinematics group

  /** @addtogroup FirstOrderKinematics
   * @{
   */
  arma::mat geom_jacobian_d; /**< geometric jacobian first derivative. */
  arma::mat anal_jacobian_d; /**< analytical jacobian first derivative. */
  /** @} */                  // end of FirstOrderKinematics group

  /** @addtogroup Dynamics
   * @{
   */
  arma::vec tension_vector; /**< [N] tensions vector, collecting tension on each cable.*/
  /** @} */                 // end of Dynamics group

  /**
   * @brief Default empty contructor, possibly predefining actuation mask.
   * @param[in] _mask Actuation binary mask, where 1 means _actuated_ and 0 means
   * _unactuated_. By default, this is equivalent to a fully actuated system (i.e. all 1).
   */
  UnderActuatedRobotVars(const arma::uvec6& _mask = arma::uvec6(arma::fill::ones))
    : platform(_mask)
  {}
  /**
   * @brief Constructor to define the rotation parametrization and the actuation mask.
   * @param[in] _angles_type Desired rotation parametrization.
   * @param[in] _mask Actuation binary mask, where 1 means _actuated_ and 0 means
   * _unactuated_. By default, this is equivalent to a fully actuated system (i.e. all 1).
   */
  UnderActuatedRobotVars(const RotParametrization _angles_type,
                         const arma::uvec6& _mask = arma::uvec6(arma::fill::ones))
    : platform(_angles_type, _mask)
  {}
  /**
   * @brief Constructor to predefine number of cables attached to the platform and the
   * actuation mask.
   * @param[in] num_cables Number of cables attached to the platform.
   * @param[in] _mask Actuation binary mask, where 1 means _actuated_ and 0 means
   * _unactuated_. By default, this is equivalent to a fully actuated system (i.e. all 1).
   */
  UnderActuatedRobotVars(const size_t num_cables,
                         const arma::uvec6& _mask = arma::uvec6(arma::fill::ones));
  /**
   * @brief Constructor to predefine number of cables attached to the platform,
   * rotation parametrization and the actuation mask.
   * @param[in] num_cables Number of cables attached to the platform.
   * @param[in] _angles_type Desired rotation parametrization.
   * @param[in] _mask Actuation binary mask, where 1 means _actuated_ and 0 means
   * _unactuated_. By default, this is equivalent to a fully actuated system (i.e. all 1).
   */
  UnderActuatedRobotVars(const size_t num_cables, const RotParametrization _angles_type,
                         const arma::uvec6& _mask = arma::uvec6(arma::fill::ones));

  /**
   * @brief Set actuation mask.
   * @param[in] _mask Actuation binary mask, where 1 means _actuated_ and 0 means
   * _unactuated_.
   */
  void setMask(const arma::uvec6& mask);

  /**
   * @brief Clear and resize jacobians in case number of cables is changed.
   */
  void resize();

  /**
   * @brief Update geometric jacobians according to current CDPR status.
   * @note If a dimension mismatch is detected, resize() is invoked before updating
   * values.
   * @see resize()
   */
  void updateGeometricJacobians();

  /**
   * @brief Update analytical jacobians according to current CDPR status.
   * @note If a dimension mismatch is detected, resize() is invoked before updating
   * values.
   * @see resize()
   */
  void updateAnaliticJacobians();

  /**
   * @brief Update all jacobians according to current CDPR status.
   * @note If a dimension mismatch is detected, resize() is invoked before updating
   * values.
   * @see resize()
   */
  void updateJacobians();
};

/**
 * @brief Update all robots zero-order variables at once (inverse kinematics problem).
 * @param[in] position [m] Platform global position @f$\mathbf{p}_P@f$.
 * @param[in] orientation [rad] Platform global orientation expressed by angles
 * @f$\boldsymbol{\varepsilon}@f$.
 * @param[in] params A reference to the robot parameters structure.
 * @param[out] vars A reference to the underactuated robot variables structure to be
 * updated.
 */
void updateIK0(const Vector3d& position, const Vector3d& orientation,
               const RobotParams& params, UnderActuatedRobotVars& vars);
/**
 * @brief Update all robots zero-order variables at once (inverse kinematics problem).
 * @param[in] pose Platform pose @f$\mathbf{x}@f$in GRAB format.
 * @param[in] params A reference to the robot parameters structure.
 * @param[out] vars A reference to the underactuated robot variables structure to be
 * updated.
 */
void updateIK0(const Vector6d& pose, const RobotParams& params,
               UnderActuatedRobotVars& vars);
/**
 * @brief Update all robots zero-order variables at once (inverse kinematics problem).
 * @param[in] pose Platform pose @f$\mathbf{x}@f$in armadillo format.
 * @param[in] params A reference to the robot parameters structure.
 * @param[out] vars A reference to the underactuated robot variables structure to be
 * updated.
 */
void updateIK0(const arma::vec6& _pose, const RobotParams& params,
               UnderActuatedRobotVars& vars);

/**
 * @brief Update cables static tension given a certain under-actuated CDPR status.
 * @param[in, out] vars A reference to the underactuated robot variables structure to be
 * updated.
 */
void updateCablesStaticTension(UnderActuatedRobotVars& vars);

/**
 * @brief Calculate @f$\mathbf{T}@f$ matrix.
 * @param[in] Generic single cable variables.
 * @return @f$\mathbf{T}@f$ matrix.
 */
grabnum::Matrix3d calcMatrixT(const CableVarsBase& cable);

/**
 * @brief Calculate static constraints for given under-actuated robot status
 * @param[in] vars CDPR variables/status.
 * @return A vector of static constraints.
 */
arma::vec calcStaticConstraint(const UnderActuatedRobotVars& vars);

/**
 * @brief Calculate cable length jacobian.
 * @param[in] vars Under-actuated CDPR variables/status.
 * @return Cable length jacobian matrix.
 */
inline arma::mat calcJacobianL(const UnderActuatedRobotVars& vars);

/**
 * @brief Calculate swivel angles jacobian.
 * @param[in] vars Under-actuated CDPR variables/status.
 * @return Swivel angles jacobian matrix.
 */
arma::mat calcJacobianSw(const UnderActuatedRobotVars& vars);

/**
 * @brief Calculate geometric-static jacobian.
 * @param[in] vars Under-actuated CDPR variables/status.
 * @return Geometric-static jacobian matrix.
 */
arma::mat calcJacobianGS(const UnderActuatedRobotVars& vars);

/**
 * @brief Optimization function to be iterated in order to solve geometric-static problem.
 * @param[in] params A reference to the robot parameters structure.
 * @param[in] act_vars A vector of actuated variables (pose elements).
 * @param[in] unact_vars A vector of unactuated variables (pose elements).
 * @param[out] fun_jacobian Resulting jacobian function for this one optimization cycle.
 * @param[out] fun_val Resulting vectorial function for this one optimization cycle.
 * @see optFunDK0GS()
 */
void optFunGS(const RobotParams& params, const arma::vec& act_vars,
              const arma::vec& unact_vars, arma::mat& fun_jacobian, arma::vec& fun_val);

/**
 * @brief Optimization function to be iterated in order to solve direct kinematics problem
 * using both geometric and static constraints.
 * @param[in] params A reference to the robot parameters structure.
 * @param[in] cables_length A vector of cable lengths.
 * @param[in] swivel_angles A vector of swivel angles.
 * @param[out] fun_jacobian Resulting jacobian function for this one optimization cycle.
 * @param[out] fun_val Resulting vectorial function for this one optimization cycle.
 * @see solveDK0GS() optFunGS()
 */
void optFunDK0GS(const RobotParams& params, const arma::vec& cables_length,
                 const arma::vec& swivel_angles, const arma::vec6& pose,
                 arma::mat& fun_jacobian, arma::vec& fun_val);

/**
 * @brief Solve direct kinematics problem for an under-actuated CDPR using both geometric
 * and static constraints.
 *
 * Direct kinematics problem consists of computing the platform pose out of cable lengths
 * and swivel angles values. Because for a generic CDPR there are multiple analytical
 * soluion to this problem, an optimization routine is employed to provide a single
 * outcome.
 * @param[in] cables_length A vector of cable lengths.
 * @param[in] swivel_angles A vector of swivel angles.
 * @param[in] init_guess_pose Initial pose guess, typically the previous computed result.
 * @param[in] params A reference to the robot parameters structure.
 * @param[out] platform_pose Platform pose resulting from the optimal solution of the
 * problem.
 * @param[int] nmax Maximum number of iterations.
 * @param[out] iter_out Number of iterations done to converge to optimal solution, if
 * found.
 * @return _True_ if problem converged to an optimal solution (local minimum), _False_
 * otherwise.
 * @see updateDK0()
 */
bool solveDK0GS(const std::vector<double>& cables_length,
                const std::vector<double>& swivel_angles,
                const grabnum::VectorXd<POSE_DIM>& init_guess_pose,
                const RobotParams& params, VectorXd<POSE_DIM>& platform_pose,
                const uint8_t nmax = 100, uint8_t* iter_out = nullptr);

/**
 * @brief Use current under-actuated CDPR status to update platform pose (direct
 * kinematics problem).
 *
 * Cable lengths and swivel angles within the given input structure are used to solve the
 * optimization problem starting from an initial guess given by the current (i.e. latest)
 * pose.
 * @param[in] params A reference to the robot parameters structure.
 * @param[in] vars CDPR variables/status.
 * @return _True_ if direct kinematics converged to an optimal solution (local minimum),
 * _False_ otherwise.
 * @see solveDK0()
 */
bool updateDK0(const RobotParams& params, UnderActuatedRobotVars& vars);

} // end namespace grabcdpr

#endif // GRABCOMMON_LIBCDPR_UNDER_ACTUATED_UTILS_H
