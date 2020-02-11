/**
 * @file dynamics.cpp
 * @author Simone Comari
 * @date 06 Feb 2020
 * @brief File containing definitions of functions declared in dynamics.h.
 */

#include "dynamics.h"

namespace grabcdpr {

Matrix3d calcInertiaMatrixGlob(const PlatformParams& params, const Vector3d& pos_PG_glob,
                               const Matrix3d& rot_mat)
{
  Matrix3d anti_com         = Skew(pos_PG_glob);
  Matrix3d inertia_mat_glob = rot_mat * params.inertia_mat_G_loc * rot_mat.Transpose() -
                              params.mass * anti_com * anti_com;
  return inertia_mat_glob;
}

void calcInertialMatricesGlob(const PlatformParams& params, const Vector3d& pos_PG_glob,
                              const Matrix3d& rot_mat, Matrix3d& inertia_mat_glob,
                              Matrix6d& mass_mat_glob)
{
  Matrix3d anti_com = Skew(pos_PG_glob);
  inertia_mat_glob  = rot_mat * params.inertia_mat_G_loc * rot_mat.Transpose() -
                     params.mass * anti_com * anti_com;
  mass_mat_glob.SetBlock<3, 3>(1, 1, Matrix3d(params.mass));
  mass_mat_glob.SetBlock<3, 3>(1, 4, -params.mass * anti_com);
  mass_mat_glob.SetBlock<3, 3>(4, 1, params.mass * anti_com);
  mass_mat_glob.SetBlock<3, 3>(4, 4, inertia_mat_glob);
}

void updateInertialMatricesGlob(const PlatformParams& params, PlatformVarsBase& platform)
{
  calcInertialMatricesGlob(params, platform.pos_PG_glob, platform.rot_mat,
                           platform.inertia_mat_glob, platform.mass_mat_glob);
}

void calcMassMatrixGlobSS(const Matrix6d& mass_mat_glob, const Matrix3d& h_mat,
                          Matrix6d& mass_mat_glob_ss)
{
  mass_mat_glob_ss.SetBlock<3, 3>(1, 1, mass_mat_glob.GetBlock<3, 3>(1, 1));
  mass_mat_glob_ss.SetBlock<3, 3>(1, 4, mass_mat_glob.GetBlock<3, 3>(1, 4) * h_mat);
  mass_mat_glob_ss.SetBlock<3, 3>(4, 1,
                                  h_mat.Transpose() * mass_mat_glob.GetBlock<3, 3>(4, 1));
  mass_mat_glob_ss.SetBlock<3, 3>(
    4, 4, h_mat.Transpose() * mass_mat_glob.GetBlock<3, 3>(4, 4) * h_mat);
}

void calcMassMatrixGlobSS(const Matrix6d& mass_mat_glob, const MatrixXd<3, 4>& h_mat,
                          MatrixXd<POSE_QUAT_DIM, POSE_QUAT_DIM>& mass_mat_glob_ss)
{
  mass_mat_glob_ss.SetBlock<3, 3>(1, 1, mass_mat_glob.GetBlock<3, 3>(1, 1));
  mass_mat_glob_ss.SetBlock<3, 4>(1, 4, mass_mat_glob.GetBlock<3, 3>(1, 4) * h_mat);
  mass_mat_glob_ss.SetBlock<4, 3>(4, 1,
                                  h_mat.Transpose() * mass_mat_glob.GetBlock<3, 3>(4, 1));
  mass_mat_glob_ss.SetBlock<4, 4>(
    4, 4, h_mat.Transpose() * mass_mat_glob.GetBlock<3, 3>(4, 4) * h_mat);
}

void updateInertialMatricesGlobSS(PlatformVars& platform)
{
  calcMassMatrixGlobSS(platform.mass_mat_glob, platform.h_mat, platform.mass_mat_glob_ss);
}

void updateInertialMatricesGlobSS(PlatformVarsQuat& platform)
{
  calcMassMatrixGlobSS(platform.mass_mat_glob, platform.h_mat, platform.mass_mat_glob_ss);
}

void updateAllInertialMatrices(const PlatformParams& params, PlatformVars& platform)
{
  updateInertialMatricesGlob(params, platform);
  updateInertialMatricesGlobSS(platform);
}

void updateAllInertialMatrices(const PlatformParams& params, PlatformVarsQuat& platform)
{
  updateInertialMatricesGlob(params, platform);
  updateInertialMatricesGlobSS(platform);
}

Vector6d calcExternalLoads(const PlatformParams& params, const Vector3d& pos_PG_glob,
                           const Matrix3d& rot_mat)
{
  Vector6d ext_load;
  ext_load.SetBlock<3, 1>(1, 1,
                          params.mass * params.gravity_acc +
                            rot_mat * params.ext_force_loc + params.ext_force_glob);
  ext_load.SetBlock<3, 1>(4, 1,
                          grabnum::Skew(pos_PG_glob) * ext_load.GetBlock<3, 1>(1, 1) +
                            rot_mat * params.ext_torque_loc + params.ext_torque_glob);
  return ext_load;
}

void updateExternalLoads(const PlatformParams& params, PlatformVarsBase& platform)
{
  platform.ext_load = calcExternalLoads(params, platform.pos_PG_glob, platform.rot_mat);
}

Vector6d calcExternalLoadsSS(const Vector6d& ext_load, const Matrix3d& h_mat)
{
  Vector6d ext_load_ss(ext_load);
  ext_load_ss.SetBlock<3, 1>(4, 1, h_mat.Transpose() * ext_load.GetBlock<3, 1>(4, 1));
  return ext_load_ss;
}

VectorXd<POSE_QUAT_DIM> calcExternalLoadsSS(const Vector6d& ext_load,
                                            const MatrixXd<3, 4>& h_mat)
{
  VectorXd<POSE_QUAT_DIM> ext_load_ss;
  ext_load_ss.SetBlock<3, 3>(1, 1, ext_load.GetBlock<3, 3>(1, 1));
  ext_load_ss.SetBlock<4, 1>(4, 1, h_mat.Transpose() * ext_load.GetBlock<3, 1>(4, 1));
  return ext_load_ss;
}

void updateExternalLoadsSS(PlatformVars& platform)
{
  platform.ext_load_ss = calcExternalLoadsSS(platform.ext_load, platform.h_mat);
}

void updateExternalLoadsSS(PlatformVarsQuat& platform)
{
  platform.ext_load_ss = calcExternalLoadsSS(platform.ext_load, platform.h_mat);
}

void updateAllExternalLoads(const PlatformParams& params, PlatformVars& platform)
{
  updateExternalLoads(params, platform);
  updateExternalLoadsSS(platform);
}

void updateAllExternalLoads(const PlatformParams& params, PlatformVarsQuat& platform)
{
  updateExternalLoads(params, platform);
  updateExternalLoadsSS(platform);
}

} // end namespace grabcdpr
