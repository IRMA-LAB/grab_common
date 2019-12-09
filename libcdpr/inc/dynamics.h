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

grabnum::Matrix3d calcInertiaMatrixGlob(const PlatformParams& params,
                                        const Vector3d& pos_PG_glob,
                                        const Matrix3d& rot_mat);

void calcInertialMatricesGlob(const PlatformParams& params, const Vector3d& pos_PG_glob,
                              const Matrix3d& rot_mat, Matrix3d& inertia_mat_glob,
                              Matrix6d& mass_mat_glob);

/**
 * @brief Update inertial matrices, that is mass and inertia matrix.
 * @param[in] params Platform parameters.
 */
void updateInertialMatricesGlob(const PlatformParams& params, PlatformVarsBase& platform);

void calcMassMatrixGlobSS(const Matrix6d& mass_mat_glob, const Matrix3d& h_mat,
                          Matrix6d& mass_mat_glob_ss);
void calcMassMatrixGlobSS(const Matrix6d& mass_mat_glob, const Matrix3d& h_mat,
                          MatrixXd<POSE_QUAT_DIM, POSE_QUAT_DIM>& mass_mat_glob_ss);

void updateInertialMatricesGlobSS(PlatformVars& platform);
void updateInertialMatricesGlobSS(PlatformVarsQuat& platform);

void updateAllInertialMatrices(const PlatformParams& params, PlatformVars& platform);
void updateAllInertialMatrices(const PlatformParams& params, PlatformVarsQuat& platform);

Vector6d calcExternalLoads(const PlatformParams& params, const Vector3d& pos_PG_glob,
                           const Matrix3d& rot_mat);

/**
 * @brief Computes the components of the external loads in terms of external forces and
 * moments acting on the platform.
 * @param[in] params A structure containing platform parameters.
 */
void updateExternalLoads(const PlatformParams& params, PlatformVarsBase& platform);

Vector6d calcExternalLoadsSS(const Vector6d& ext_load, const Matrix3d& h_mat);
VectorXd<POSE_QUAT_DIM> calcExternalLoadsSS(const Vector6d& ext_load,
                                            const MatrixXd<3, 4>& h_mat);

void updateExternalLoadsSS(PlatformVars& platform);
void updateExternalLoadsSS(PlatformVarsQuat& platform);

void updateAllExternalLoads(const PlatformParams& params, PlatformVars& platform);
void updateAllExternalLoads(const PlatformParams& params, PlatformVarsQuat& platform);

/** @} */ // end of Dynamics group

} // end namespace grabcdpr

#endif // GRABCOMMON_LIBCDPR_DYNAMICS_H
