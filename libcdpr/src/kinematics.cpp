/**
 * @file kinematics.cpp
 * @author Edoardo Idà, Simone Comari
 * @date 01 Aug 2019
 * @brief File containing definitions of functions declared in kinematics.h.
 */

#include "kinematics.h"

namespace grabcdpr {

void UpdatePlatformPose(const grabnum::Vector3d& position,
                        const grabnum::Vector3d& orientation,
                        const grabnum::Vector3d& pos_PG_loc, PlatformVars& platform)
{
  // Update platform pose.
  platform.UpdatePose(position, orientation);
  // Calculate platform baricenter positions expressed in global frame.
  platform.pos_PG_glob = platform.rot_mat * pos_PG_loc;
  platform.pos_OG_glob = platform.position + platform.pos_PG_glob;
}

void UpdatePlatformPose(const grabnum::Vector3d& position,
                        const grabgeom::Quaternion& orientation,
                        const grabnum::Vector3d& pos_PG_loc, PlatformQuatVars& platform)
{
  // Update platform pose.
  platform.UpdatePose(position, orientation);
  // Calculate platform baricenter positions expressed in global frame.
  platform.pos_PG_glob = platform.rot_mat * pos_PG_loc;
  platform.pos_OG_glob = platform.position + platform.pos_PG_glob;
}

void UpdatePlatformPose(const grabnum::Vector3d& position,
                        const grabnum::Vector3d& orientation,
                        const PlatformParams& params, PlatformVars& platform)
{
  UpdatePlatformPose(position, orientation, params.pos_PG_loc, platform);
}

void UpdatePlatformPose(const grabnum::Vector3d& position,
                        const grabgeom::Quaternion& orientation,
                        const PlatformParams& params, PlatformQuatVars& platform)
{
  UpdatePlatformPose(position, orientation, params.pos_PG_loc, platform);
}

void UpdatePosA(const ActuatorParams& params, const PlatformVars& platform,
                CableVars& cable)
{
  cable.pos_PA_glob = platform.rot_mat * params.winch.pos_PA_loc;
  cable.pos_OA_glob = platform.position + cable.pos_PA_glob;
  cable.pos_DA_glob = cable.pos_OA_glob - params.pulley.pos_OD_glob;
}

void UpdatePosA(const ActuatorParams& params, const PlatformQuatVars& platform,
                CableVarsQuat& cable)
{
  cable.pos_PA_glob = platform.rot_mat * params.winch.pos_PA_loc;
  cable.pos_OA_glob = platform.position + cable.pos_PA_glob;
  cable.pos_DA_glob = cable.pos_OA_glob - params.pulley.pos_OD_glob;
}

void CalcPulleyVersors(const PulleyParams& params, const double swivel_ang,
                       CableVarsBase& cable)
{
  double cos_sigma = cos(swivel_ang);
  double sin_sigma = sin(swivel_ang);
  cable.vers_u     = params.vers_i * cos_sigma + params.vers_j * sin_sigma;
  cable.vers_w     = -params.vers_i * sin_sigma + params.vers_j * cos_sigma;
}

void CalcPulleyVersors(const PulleyParams& params, CableVarsBase& cable)
{
  CalcPulleyVersors(params, cable.swivel_ang, cable);
}

double CalcSwivelAngle(const PulleyParams& params, const grabnum::Vector3d& pos_DA_glob)
{
  return atan2(grabnum::Dot(params.vers_j, pos_DA_glob),
               grabnum::Dot(params.vers_i, pos_DA_glob));
}

double CalcSwivelAngle(const PulleyParams& params, const CableVarsBase& cable)
{
  return CalcSwivelAngle(params, cable.pos_DA_glob);
}

double CalcTangentAngle(const PulleyParams& params, const grabnum::Vector3d& vers_u,
                        const grabnum::Vector3d& pos_DA_glob)
{
  double s       = grabnum::Dot(vers_u, pos_DA_glob);
  double app_var = grabnum::Dot(params.vers_k, pos_DA_glob) / s;
  double psi = 2. * atan(app_var + sqrt(1. - 2. * params.radius / s + SQUARE(app_var)));
  return psi;
}

double CalcTangentAngle(const PulleyParams& params, const CableVarsBase& cable)
{
  return CalcTangentAngle(params, cable.vers_u, cable.pos_DA_glob);
}

void CalcCableVectors(const PulleyParams& params, const grabnum::Vector3d& vers_u,
                      const grabnum::Vector3d& pos_DA_glob, const double tan_ang,
                      CableVarsBase& cable)
{
  // Versors describing cable exit direction from swivel pulley.
  double cos_psi = cos(tan_ang);
  double sin_psi = sin(tan_ang);
  cable.vers_n   = vers_u * cos_psi + params.vers_k * sin_psi;
  cable.vers_t   = vers_u * sin_psi - params.vers_k * cos_psi;
  // Vector from swivel pulley exit point to platform attaching point.
  cable.pos_BA_glob = pos_DA_glob - params.radius * (vers_u + cable.vers_n);
}

void CalcCableVectors(const PulleyParams& params, CableVarsBase& cable)
{
  CalcCableVectors(params, cable.vers_u, cable.pos_DA_glob, cable.tan_ang, cable);
}

double CalcCableLen(const grabnum::Vector3d& pos_BA_glob)
{
  return grabnum::Norm(pos_BA_glob);
}

double CalcMotorCounts(const double tau, const double cable_len,
                       const double pulley_radius, const double tan_ang)
{
  return (cable_len + pulley_radius * (M_PI - tan_ang)) / tau;
}

double CalcMotorCounts(ActuatorParams& params, const CableVarsBase& cable)
{
  return CalcMotorCounts(params.winch.CountsToLengthFactor(), cable.length,
                         params.pulley.radius, cable.tan_ang);
}

void UpdateJacobiansRow(const grabnum::Matrix3d H_mat, CableVars& cable)
{
  cable.geom_jacob_row.SetBlock<1, 3>(1, 1, cable.vers_t.Transpose());
  cable.geom_jacob_row.SetBlock<1, 3>(
    1, 4, cable.vers_t.Transpose() * grabnum::Skew(cable.pos_PA_glob));

  cable.anal_jacob_row = cable.geom_jacob_row;
  cable.anal_jacob_row.SetBlock<1, 3>(1, 4,
                                      cable.anal_jacob_row.GetBlock<1, 3>(1, 4) * H_mat);
}

void UpdateJacobiansRow(const grabnum::MatrixXd<3, 4> H_mat, CableVarsQuat& cable)
{
  cable.geom_jacob_row.SetBlock<1, 3>(1, 1, cable.vers_t.Transpose());
  cable.geom_jacob_row.SetBlock<1, 3>(
    1, 4, cable.vers_t.Transpose() * grabnum::Skew(cable.pos_PA_glob));

  cable.anal_jacob_row.SetBlock<1, 3>(1, 1, cable.geom_jacob_row.GetBlock<1, 3>(1, 1));
  cable.anal_jacob_row.SetBlock<1, 4>(1, 4,
                                      cable.geom_jacob_row.GetBlock<1, 3>(1, 4) * H_mat);
}

void UpdateCableZeroOrd(const ActuatorParams& params, const PlatformVars& platform,
                        CableVars& cable)
{
  UpdatePosA(params, platform, cable); // update segments ending with point A_i.
  cable.swivel_ang =
    CalcSwivelAngle(params.pulley, cable); // from 1st kinematic constraint.
  CalcPulleyVersors(params.pulley, cable);
  cable.tan_ang =
    CalcTangentAngle(params.pulley, cable);       // from 2nd kinematic constraint.
  CalcCableVectors(params.pulley, cable);         // from 1st kinematic constraint.
  cable.length = CalcCableLen(cable.pos_BA_glob); // from 3rd kinematic constraint.
  UpdateJacobiansRow(platform.h_mat, cable);
}

void UpdateCableZeroOrd(const ActuatorParams& params, const PlatformQuatVars& platform,
                        CableVarsQuat& cable)
{
  UpdatePosA(params, platform, cable); // update segments ending with point A_i.
  cable.swivel_ang =
    CalcSwivelAngle(params.pulley, cable); // from 1st kinematic constraint.
  CalcPulleyVersors(params.pulley, cable);
  cable.tan_ang =
    CalcTangentAngle(params.pulley, cable);       // from 2nd kinematic constraint.
  CalcCableVectors(params.pulley, cable);         // from 1st kinematic constraint.
  cable.length = CalcCableLen(cable.pos_BA_glob); // from 3rd kinematic constraint.
  UpdateJacobiansRow(platform.h_mat, cable);
}

void UpdateIK0(const grabnum::Vector3d& position, const grabnum::Vector3d& orientation,
               const RobotParams& params, RobotVars& vars)
{
  UpdatePlatformPose(position, orientation, params.platform, vars.platform);
  // Safety check
  std::vector<id_t> active_actuators_id = params.activeActuatorsId();
  if (vars.geom_jabobian.n_rows != active_actuators_id.size())
    vars.geom_jabobian.resize(active_actuators_id.size(), POSE_DIM);
  if (vars.anal_jabobian.n_rows != active_actuators_id.size())
    vars.anal_jabobian.resize(active_actuators_id.size(), POSE_DIM);
  for (uint8_t i = 0; i < active_actuators_id.size(); ++i)
  {
    UpdateCableZeroOrd(params.actuators[active_actuators_id[i]], vars.platform,
                       vars.cables[i]);
    vars.geom_jabobian.row(i) = arma::rowvec6(vars.cables[i].geom_jacob_row.Data());
    vars.anal_jabobian.row(i) = arma::rowvec6(vars.cables[i].geom_jacob_row.Data());
  }
}

void UpdateIK0(const grabnum::Vector3d& position, const grabgeom::Quaternion& orientation,
               const RobotParams& params, RobotVarsQuat& vars)
{
  UpdatePlatformPose(position, orientation, params.platform, vars.platform);
  // Safety check
  std::vector<id_t> active_actuators_id = params.activeActuatorsId();
  if (vars.geom_jabobian.n_rows != active_actuators_id.size())
    vars.geom_jabobian.resize(active_actuators_id.size(), POSE_DIM);
  if (vars.anal_jabobian.n_rows != active_actuators_id.size())
    vars.anal_jabobian.resize(active_actuators_id.size(), POSE_QUAT_DIM);
  for (uint8_t i = 0; i < active_actuators_id.size(); ++i)
  {
    UpdateCableZeroOrd(params.actuators[active_actuators_id[i]], vars.platform,
                       vars.cables[i]);
    vars.geom_jabobian.row(i) = arma::rowvec6(vars.cables[i].geom_jacob_row.Data());
    vars.anal_jabobian.row(i) = arma::rowvec7(vars.cables[i].geom_jacob_row.Data());
  }
}

} // end namespace grabcdpr
