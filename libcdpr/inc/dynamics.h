/**
 * @file dynamics.h
 * @author Simone Comari
 * @date 07 Feb 2020
 * @brief This file include functions strictly related to CDPR dynamics.
 */

#ifndef GRABCOMMON_LIBCDPR_DYNAMICS_H
#define GRABCOMMON_LIBCDPR_DYNAMICS_H

#include "cdpr_types.h"

using namespace grabnum;

/**
 * @brief Namespace for CDPR-related utilities, such as kinematics and dynamics.
 */
namespace grabcdpr {
/** @defgroup Dynamics Dynamics
 * This group collects all elements related to the dynamics of a generic 6DoF CDPR.
 * @{
 */

/**
 * @brief Calculate global inertia matrix.
 * @param[in] params A reference to the platform parameters structure.
 * @param[in] pos_PG_glob [m] Global CoG position @f$\mathbf{p}'_G@f$.
 * @param[in] rot_mat Rotation matrix describing platform global orientation.
 * @return Global inertia matrix.
 */
grabnum::Matrix3d calcInertiaMatrixGlob(const PlatformParams& params,
                                        const Vector3d& pos_PG_glob,
                                        const Matrix3d& rot_mat);

/**
 * @brief Calculate global inertial matrices.
 *
 * Calculate global inertia matrix and mass matrix at once explicitly.
 * @param[in] params A reference to the platform parameters structure.
 * @param[in] pos_PG_glob [m] Global CoG position @f$\mathbf{p}'_G@f$.
 * @param[in] rot_mat Rotation matrix describing platform global orientation.
 * @param[out] inertia_mat_glob Global inertia matrix.
 * @param[out] mass_mat_glob Global mass matrix.
 * @see updateInertialMatricesGlob() calcInertiaMatrixGlob()
 */
void calcInertialMatricesGlob(const PlatformParams& params, const Vector3d& pos_PG_glob,
                              const Matrix3d& rot_mat, Matrix3d& inertia_mat_glob,
                              Matrix6d& mass_mat_glob);

/**
 * @brief Update global inertial matrices.
 *
 * Calculate global inertia matrix and mass matrix at once implicitly.
 * @param[in] params Platform parameters.
 * @param[in, out] platform CDPR platform to be updated.
 * @see calcInertialMatricesGlob()
 */
void updateInertialMatricesGlob(const PlatformParams& params, PlatformVarsBase& platform);

/**
 * @brief Calculate state-space global mass matrix for the 3-angle parametrization.
 *
 * Explicit implementation.
 * @param[in] mass_mat_glob Global mass matrix.
 * @param[in] h_mat Matrix @f$\mathbf{H}@f$.
 * @param[out] mass_mat_glob_ss State-space global mass matrix.
 * @see updateInertialMatricesGlobSS()
 */
void calcMassMatrixGlobSS(const Matrix6d& mass_mat_glob, const Matrix3d& h_mat,
                          Matrix6d& mass_mat_glob_ss);
/**
 * @brief Calculate state-space global mass matrix for the quaternion parametrization.
 *
 * Explicit implementation.
 * @param[in] mass_mat_glob Global mass matrix.
 * @param[in] h_mat Matrix @f$\mathbf{H}@f$.
 * @param[out] mass_mat_glob_ss State-space global mass matrix.
 * @see updateInertialMatricesGlobSS()
 */
void calcMassMatrixGlobSS(const Matrix6d& mass_mat_glob, const MatrixXd<3, 4>& h_mat,
                          MatrixXd<POSE_QUAT_DIM, POSE_QUAT_DIM>& mass_mat_glob_ss);

/**
 * @brief Update state-space global mass matrix for the 3-angle parametrization.
 *
 * Implicit implementation: extracts necessary inputs from platform variables.
 * @param[in, out] platform CDPR platform to be updated.
 * @see updateInertialMatricesGlobSS()
 */
void updateInertialMatricesGlobSS(PlatformVars& platform);
/**
 * @brief Update state-space global mass matrix for the 3-angle parametrization.
 *
 * Implicit implementation: extracts necessary inputs from platform variables.
 * @param[in, out] platform CDPR platform to be updated.
 * @see updateInertialMatricesGlobSS()
 */
void updateInertialMatricesGlobSS(PlatformVarsQuat& platform);

/**
 * @brief Update all inertial matrices in a 3-angle-parametrized CDPR platform.
 *
 * Update both inertia and mass matrix for the state-space and the standard configuration.
 * @param[in] params Platform parameters.
 * @param[in, out] platform CDPR platform to be updated.
 */
void updateAllInertialMatrices(const PlatformParams& params, PlatformVars& platform);
/**
 * @brief Update all inertial matrices in a quaternion-parametrized CDPR platform.
 *
 * Update both inertia and mass matrix for the state-space and the standard configuration.
 * @param[in] params Platform parameters.
 * @param[in, out] platform CDPR platform to be updated.
 */
void updateAllInertialMatrices(const PlatformParams& params, PlatformVarsQuat& platform);

/**
 * @brief Calculate external loads given current platform pose.
 *
 * Explicit implementation.
 * @param[in] params Platform parameters.
 * @param[in] pos_PG_glob [m] Global CoG position @f$\mathbf{p}'_G@f$.
 * @param[in] rot_mat Rotation matrix describing platform global orientation.
 * @return External load wrench.
 */
Vector6d calcExternalLoads(const PlatformParams& params, const Vector3d& pos_PG_glob,
                           const Matrix3d& rot_mat);

/**
 * @brief Computes the components of the external loads in terms of external forces and
 * moments acting on the platform.
 * @param[in] params A structure containing platform parameters.
 * @param[in, out] platform CDPR platform to be updated.
 */
void updateExternalLoads(const PlatformParams& params, PlatformVarsBase& platform);

/**
 * @brief Calculate external loads in state-space form for 3-angle parametrization.
 *
 * Explicit implementation.
 * @param[in] ext_load External load wrench.
 * @param[in] h_mat Matrix @f$\mathbf{H}@f$.
 * @return External load wrench in state-space form.
 */
Vector6d calcExternalLoadsSS(const Vector6d& ext_load, const Matrix3d& h_mat);
/**
 * @brief Calculate external loads in state-space form for quaternion parametrization.
 *
 * Explicit implementation.
 * @param[in] ext_load External load wrench.
 * @param[in] h_mat Matrix @f$\mathbf{H}@f$.
 * @return External load wrench in state-space form.
 */
VectorXd<POSE_QUAT_DIM> calcExternalLoadsSS(const Vector6d& ext_load,
                                            const MatrixXd<3, 4>& h_mat);

/**
 * @brief Calculate external loads in state-space form for 3-angle parametrization.
 *
 * Implicit implementation.
 * @param[in, out] platform CDPR platform to be updated.
 */
void updateExternalLoadsSS(PlatformVars& platform);
/**
 * @brief Calculate external loads in state-space form for quaternion parametrization.
 *
 * Implicit implementation.
 * @param[in, out] platform CDPR platform to be updated.
 */
void updateExternalLoadsSS(PlatformVarsQuat& platform);

/**
 * @brief Computes all external loads forms at once for 3-angle parametrization.
 * @param[in] params A structure containing platform parameters.
 * @param[in, out] platform CDPR platform to be updated.
 */
void updateAllExternalLoads(const PlatformParams& params, PlatformVars& platform);
/**
 * @brief Computes all external loads forms at once for quaternion parametrization.
 * @param[in] params A structure containing platform parameters.
 * @param[in, out] platform CDPR platform to be updated.
 */
void updateAllExternalLoads(const PlatformParams& params, PlatformVarsQuat& platform);

/** @} */ // end of Dynamics group

} // end namespace grabcdpr

#endif // GRABCOMMON_LIBCDPR_DYNAMICS_H
