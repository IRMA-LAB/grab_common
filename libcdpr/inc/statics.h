#ifndef GRABCOMMON_LIBCDPR_STATICS_H
#define GRABCOMMON_LIBCDPR_STATICS_H

#include "matrix_utilities.h"

#include "cdpr_types.h"
#include "kinematics.h"

using namespace grabnum;

/**
 * @brief toCvMat
 * @param vect
 * @return
 */
arma::vec toArmaMat(Vector3d vect, bool copy = true);
/**
 * @brief toCvMat
 * @param vect
 * @return
 */
arma::vec toArmaMat(VectorXd<POSE_DIM> vect, bool copy = true);
/**
 * @brief toCvMat
 * @param vect
 * @return
 */
arma::vec toArmaMat(VectorXd<POSE_QUAT_DIM> vect, bool copy = true);
/**
 * @brief toCvMat
 * @param vect
 * @return
 */
arma::mat toArmaMat(Matrix3d vect, bool copy = true);

/**
 * @brief Namespace for CDPR-related utilities, such as kinematics and dynamics.
 */
namespace grabcdpr {

/**
 * @brief Computes the components of the external loads in terms of external forces and
 * moments acting on the platform.
 * @param[in] R A 3-by-3 matrix, that premultiplies the equation of dynamic equilibrium in
 * order to make the mass matrix symmetric.
 * @param[in] params A structure containing platform parameters.
 * @param[in,out] platform A structure containing updated platform pose and the external
 * loads to be updated.
 */
void UpdateExternalLoads(const grabnum::Matrix3d& R, const PlatformParams& params,
                         PlatformVars& platform);
/**
 * @brief Computes the components of the external loads in terms of external forces and
 * moments acting on the platform.
 * @param[in] R A 3-by-3 matrix, that premultiplies the equation of dynamic equilibrium in
 * order to make the mass matrix symmetric.
 * @param[in] params A structure containing platform parameters.
 * @param[in,out] platform A structure containing updated platform pose and the external
 * loads to be updated.
 */
void UpdateExternalLoads(const grabnum::Matrix3d& R, const PlatformParams& params,
                         PlatformQuatVars& platform);

/**
 * @brief CalCablesTensionStat
 * @param vars
 */
void CalCablesTensionStat(RobotVars& vars);
/**
 * @brief CalCablesTensionStat
 * @param vars
 */
void CalCablesTensionStat(RobotVarsQuat& vars);

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
arma::mat CalcGsJacobians(const RobotVars& vars, const arma::mat& Ja, const arma::mat& Ju,
                          const Vector3d& mg);
arma::mat CalcGsJacobians(const RobotVarsQuat& vars, const arma::mat& Ja,
                          const arma::mat& Ju, const Vector3d& mg);

} // end namespace grabcdpr

#endif // GRABCOMMON_LIBCDPR_STATICS_H
