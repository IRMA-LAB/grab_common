#ifndef GRABCOMMON_LIBCDPR_STATICS_H
#define GRABCOMMON_LIBCDPR_STATICS_H

#include "grabcommon.h"
#include "matrix_utilities.h"

#include "cdpr_types.h"
#include "dynamics.h"
#include "kinematics.h"

using namespace grabnum;

/**
 * @brief Namespace for CDPR-related utilities, such as kinematics and dynamics.
 */
namespace grabcdpr {
/** @defgroup Dynamics Dynamics
 * This group collects all elements related to the dynamics of a generic 6DoF CDPR.
 * @{
 */

arma::vec calcCablesStaticTension(const arma::mat& geom_jacobian,
                                  const Vector6d& ext_load);

/**
 * @brief CalCablesTensionStat
 * @param vars
 */
void updateCablesStaticTension(RobotVars& vars);
/**
 * @brief CalCablesTensionStat
 * @param vars
 */
void updateCablesStaticTension(RobotVarsQuat& vars);

/**
 * @brief calcGeometricStatic
 * @param params
 * @param fixed_coord
 * @param var_coord
 * @param mask
 * @param mat
 * @param vector
 */
void calcGeometricStatic(const RobotParams& params, const arma::vec& fixed_coord,
                         const arma::vec& var_coord, const VectorXi<POSE_DIM>& mask,
                         arma::mat& fun_jacobian, arma::vec& func_val);
void calcGeometricStatic(const RobotParams& params, const arma::vec& fixed_coord,
                         const arma::vec& var_coord, const VectorXi<POSE_QUAT_DIM>& mask,
                         arma::mat& fun_jacobian, arma::vec& func_val);

/**
 * @brief CalcGsJacobians
 * @param vars
 * @param Ja
 * @param Ju
 * @param mg
 * @return
 */
void calcGsJacobians(const RobotVars& vars, const arma::mat& Ja, const arma::mat& Ju,
                     const Vector3d& mg, arma::mat& J_q);
void calcGsJacobians(const RobotVars& vars, const arma::mat& Ja, const arma::mat& Ju,
                     const Vector3d& mg, arma::mat& J_q, arma::mat& J_sl);
void calcGsJacobians(const RobotVarsQuat& vars, const arma::mat& Ja, const arma::mat& Ju,
                     const Vector3d& mg, arma::mat& J_q);
void calcGsJacobians(const RobotVarsQuat& vars, const arma::mat& Ja, const arma::mat& Ju,
                     const Vector3d& mg, arma::mat& J_q, arma::mat& J_sl);
arma::mat calcGsJacobiansOld(const RobotVars& vars, const arma::mat& Ja,
                             const arma::mat& Ju, const Vector3d& mg);
arma::mat calcGsJacobiansOld(const RobotVarsQuat& vars, const arma::mat& Ja,
                             const arma::mat& Ju, const Vector3d& mg);

/** @} */ // end of Dynamics group

} // end namespace grabcdpr

#endif // GRABCOMMON_LIBCDPR_STATICS_H
