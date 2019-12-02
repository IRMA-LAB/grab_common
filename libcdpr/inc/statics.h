#ifndef GRABCOMMON_LIBCDPR_STATICS_H
#define GRABCOMMON_LIBCDPR_STATICS_H

#include "grabcommon.h"
#include "matrix_utilities.h"

#include "cdpr_types.h"
#include "kinematics.h"
#include "dynamics.h"

using namespace grabnum;

/**
 * @brief toCvMat
 * @param vect
 * @return
 */
arma::vec toArmaVec(Vector3d vect, bool copy = true);
/**
 * @brief toCvMat
 * @param vect
 * @return
 */
arma::vec toArmaVec(VectorXd<POSE_DIM> vect, bool copy = true);
/**
 * @brief toCvMat
 * @param vect
 * @return
 */
arma::vec toArmaVec(VectorXd<POSE_QUAT_DIM> vect, bool copy = true);
/**
 * @brief toCvMat
 * @param vect
 * @return
 */
arma::mat toArmaMat(Matrix3d mat, bool copy = true);

/**
 * @brief fromArmaVec3
 * @param vect
 * @return
 */
grabnum::Vector3d fromArmaVec3(const arma::vec3& vect);

/**
 * @brief Namespace for CDPR-related utilities, such as kinematics and dynamics.
 */
namespace grabcdpr {
/** @defgroup Dynamics Dynamics
 * This group collects all elements related to the dynamics of a generic 6DoF CDPR.
 * @{
 */

/**
 * @brief CalCablesTensionStat
 * @param vars
 */
void CalCablesStaticTension(RobotVars& vars);
/**
 * @brief CalCablesTensionStat
 * @param vars
 */
void CalCablesStaticTension(RobotVarsQuat& vars);

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
void CalcGsJacobians(const RobotVars& vars, const arma::mat& Ja, const arma::mat& Ju,
                     const Vector3d& mg, arma::mat& J_q);
void CalcGsJacobians(const RobotVars& vars, const arma::mat& Ja, const arma::mat& Ju,
                     const Vector3d& mg, arma::mat& J_q, arma::mat& J_sl);
void CalcGsJacobians(const RobotVarsQuat& vars, const arma::mat& Ja, const arma::mat& Ju,
                     const Vector3d& mg, arma::mat& J_q);
void CalcGsJacobians(const RobotVarsQuat& vars, const arma::mat& Ja, const arma::mat& Ju,
                     const Vector3d& mg, arma::mat& J_q, arma::mat& J_sl);
arma::mat CalcGsJacobiansOld(const RobotVars& vars, const arma::mat& Ja,
                             const arma::mat& Ju, const Vector3d& mg);
arma::mat CalcGsJacobiansOld(const RobotVarsQuat& vars, const arma::mat& Ja,
                             const arma::mat& Ju, const Vector3d& mg);

/** @} */ // end of Dynamics group

} // end namespace grabcdpr

#endif // GRABCOMMON_LIBCDPR_STATICS_H
