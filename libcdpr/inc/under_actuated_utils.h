#ifndef GRABCOMMON_LIBCDPR_UNDER_ACTUATED_UTILS_H
#define GRABCOMMON_LIBCDPR_UNDER_ACTUATED_UTILS_H

#include "cdpr_types.h"
#include "kinematics.h"

/**
 * @brief Namespace for CDPR-related utilities, such as kinematics and dynamics.
 */
namespace grabcdpr {

/**
 * @brief The UnderActuatedPlatformVars struct
 */
struct UnderActuatedPlatformVars: PlatformVars
{
  arma::uvec6 mask;

  /** @addtogroup ZeroOrderKinematics
   * @{
   */
  arma::vec actuated_vars;
  arma::vec unactuated_vars;
  /** @} */ // end of ZeroOrderKinematics group

  /** @addtogroup FirstOrderKinematics
   * @{
   */
  arma::vec actuated_deriv;
  arma::vec unactuated_deriv;
  /** @} */ // end of FirstOrderKinematics group

  /** @addtogroup SecondOrderKinematics
   * @{
   */
  arma::vec actuated_deriv_2;
  arma::vec unactuated_deriv_2;
  /** @} */ // end of SecondOrderKinematics group

  /** @addtogroup Dynamics
   * @{
   */
  arma::mat mass_matrix_global_a;
  arma::mat mass_matrix_global_u;

  arma::mat mass_matrix_global_ss_a;
  arma::mat mass_matrix_global_ss_u;

  arma::vec total_load_a;
  arma::vec total_load_u;

  arma::vec total_load_ss_a;
  arma::vec total_load_ss_u;

  arma::vec external_load_a;
  arma::vec external_load_u;

  arma::vec external_load_ss_a;
  arma::vec external_load_ss_u;
  /** @} */ // end of Dynamics group

  UnderActuatedPlatformVars(const arma::uvec6& _mask = arma::uvec6(arma::fill::ones));
  UnderActuatedPlatformVars(const RotParametrization _angles_type = TILT_TORSION,
                            const arma::uvec6& _mask = arma::uvec6(arma::fill::ones));
  UnderActuatedPlatformVars(const grabnum::Vector3d& _position,
                            const grabnum::Vector3d& _orientation,
                            const RotParametrization _angles_type = TILT_TORSION,
                            const arma::uvec6& _mask = arma::uvec6(arma::fill::ones));
  UnderActuatedPlatformVars(const grabnum::Vector3d& _position,
                            const grabnum::Vector3d& _velocity,
                            const grabnum::Vector3d& _orientation,
                            const grabnum::Vector3d& _orientation_dot,
                            const RotParametrization _angles_type = TILT_TORSION,
                            const arma::uvec6& _mask = arma::uvec6(arma::fill::ones));
  UnderActuatedPlatformVars(const grabnum::Vector3d& _position,
                            const grabnum::Vector3d& _velocity,
                            const grabnum::Vector3d& _acceleration,
                            const grabnum::Vector3d& _orientation,
                            const grabnum::Vector3d& _orientation_dot,
                            const grabnum::Vector3d& _orientation_ddot,
                            const RotParametrization _angles_type = TILT_TORSION,
                            const arma::uvec6& _mask = arma::uvec6(arma::fill::ones));

  void updatePose(const grabnum::VectorXd<POSE_DIM>& _pose);
  void updatePose(const grabnum::Vector3d& _position,
                  const grabnum::Vector3d& _orientation);
  void updatePose(const arma::vec& _actuated_vars, const arma::vec& _unactuated_vars);

  void updateVel(const grabnum::Vector3d& _velocity,
                 const grabnum::Vector3d& _orientation_dot,
                 const grabnum::Vector3d& _orientation);
  void updateVel(const grabnum::Vector3d& _velocity,
                 const grabnum::Vector3d& _orientation_dot);

  void updateAcc(const grabnum::Vector3d& _acceleration,
                 const grabnum::Vector3d& _orientation_ddot,
                 const grabnum::Vector3d& _orientation_dot,
                 const grabnum::Vector3d& _orientation,
                 const grabnum::Matrix3d& _h_mat);
  void updateAcc(const grabnum::Vector3d& _acceleration,
                 const grabnum::Vector3d& _orientation_ddot);

  void update(const grabnum::Vector3d& _position, const grabnum::Vector3d& _velocity,
              const grabnum::Vector3d& _acceleration,
              const grabnum::Vector3d& _orientation,
              const grabnum::Vector3d& _orientation_dot,
              const grabnum::Vector3d& _orientation_ddot);

  void setMask(const arma::uvec6& _mask);
};

struct UnderActuatedRobotVars
{
  UnderActuatedPlatformVars
    platform;                    /**< variables of a generic 6DoF platform with angles. */
  std::vector<CableVars> cables; /**< vector of variables of a single cables in a CDPR. */

  /** @addtogroup ZeroOrderKinematics
   * @{
   */
  arma::mat geom_jacobian;
  arma::mat geom_jacobian_a;
  arma::mat geom_jacobian_u;
  arma::mat geom_orthogonal;

  arma::mat anal_jacobian;
  arma::mat anal_jacobian_a;
  arma::mat anal_jacobian_u;
  arma::mat anal_orthogonal;

  arma::mat gamma_mat;
  /** @} */ // end of ZeroOrderKinematics group

  /** @addtogroup FirstOrderKinematics
   * @{
   */
  arma::mat geom_jacobian_d;
  arma::mat anal_jacobian_d;
  /** @} */ // end of FirstOrderKinematics group

  arma::vec tension_vector; /**< @addtogroup Dynamics */

  UnderActuatedRobotVars(const arma::uvec6& _mask = arma::uvec6(arma::fill::ones))
    : platform(_mask)
  {}
  UnderActuatedRobotVars(const RotParametrization _angles_type,
                         const arma::uvec6& _mask = arma::uvec6(arma::fill::ones))
    : platform(_angles_type, _mask)
  {}
  UnderActuatedRobotVars(const size_t num_cables,
                         const arma::uvec6& _mask = arma::uvec6(arma::fill::ones));
  UnderActuatedRobotVars(const size_t num_cables, const RotParametrization _angles_type,
                         const arma::uvec6& _mask = arma::uvec6(arma::fill::ones));

  void setMask(const arma::uvec6& mask);

  void resize();

  void updateGeometricJacobians();
  void updateAnaliticJacobians();
  void updateJacobians();
};

grabnum::Matrix3d calcMatrixT(const CableVarsBase& cable);

arma::mat calcJacobianGS(const UnderActuatedRobotVars& vars);

void updateIK0(const Vector3d& position, const Vector3d& orientation,
               const RobotParams& params, UnderActuatedRobotVars& vars);

void updateCablesStaticTension(UnderActuatedRobotVars& vars);

arma::vec calcStaticConstraint(const UnderActuatedRobotVars& vars);

void OptFunGS(const RobotParams& params, const arma::vec& act_vars,
              const arma::vec& unact_vars, arma::mat& fun_jacobian, arma::vec& fun_val);

} // end namespace grabcdpr

#endif // GRABCOMMON_LIBCDPR_UNDER_ACTUATED_UTILS_H
