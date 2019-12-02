/**
 * @file diffkinematics.cpp
 * @author Edoardo Idà, Simone Comari
 * @date 02 Dec 2019
 * @brief File containing definitions of functions declared in diffkinematics.h.
 */

#include "diffkinematics.h"

namespace grabcdpr {

void updatePlatformVel(const Vector3d& velocity, const Vector3d& orientation_dot,
                       const Vector3d& pos_PG_glob, PlatformVars& platform)
{
  // Update platform velocities.
  platform.updateVel(velocity, orientation_dot);
  // Calculate platform baricenter velocity expressed in global frame.
  platform.vel_OG_glob = platform.velocity + Cross(platform.angular_vel, pos_PG_glob);
}

void updatePlatformVel(const Vector3d& velocity,
                       const grabgeom::Quaternion& orientation_dot,
                       const Vector3d& pos_PG_glob, PlatformQuatVars& platform)
{
  // Update platform velocities.
  platform.updateVel(velocity, orientation_dot);
  // Calculate platform baricenter velocity expressed in global frame.
  platform.vel_OG_glob = platform.velocity + Cross(platform.angular_vel, pos_PG_glob);
}

void updatePlatformVel(const Vector3d& velocity, const Vector3d& orientation_dot,
                       PlatformVars& platform)
{
  updatePlatformVel(velocity, orientation_dot, platform.pos_PG_glob, platform);
}

void updatePlatformVel(const Vector3d& velocity,
                       const grabgeom::Quaternion& orientation_dot,
                       PlatformQuatVars& platform)
{
  updatePlatformVel(velocity, orientation_dot, platform.pos_PG_glob, platform);
}

Vector3d calcVelA(const Vector3d& pos_PA_glob, const PlatformVarsBase& platform)
{
  return platform.velocity + Cross(platform.angular_vel, pos_PA_glob);
}

void updateVelA(const PlatformVarsBase& platform, CableVarsBase& cable)
{
  cable.vel_OA_glob = calcVelA(cable.pos_PA_glob, platform);
}

void calcPulleyVersorsDot(const Vector3d& vers_u, const Vector3d& vers_w,
                          const double swivel_ang_vel, Vector3d& vers_u_dot,
                          Vector3d& vers_w_dot)
{
  vers_u_dot = vers_w * swivel_ang_vel;
  vers_w_dot = -vers_u * swivel_ang_vel;
}

void updatePulleyVersorsDot(CableVarsBase& cable)
{
  calcPulleyVersorsDot(cable.vers_u, cable.vers_w, cable.swivel_ang_vel, cable.vers_u_dot,
                       cable.vers_w_dot);
}

double calcSwivelAngSpeed(const Vector3d& vers_u, const Vector3d& vers_w,
                          const Vector3d& vel_OA_glob, const Vector3d& pos_DA_glob)
{
  return Dot(vers_w, vel_OA_glob) / Dot(vers_u, pos_DA_glob);
}

void updateSwivelAngSpeed(CableVarsBase& cable)
{
  cable.swivel_ang_vel =
    calcSwivelAngSpeed(cable.vers_u, cable.vers_w, cable.vel_OA_glob, cable.pos_DA_glob);
}

double calcTangAngSpeed(const Vector3d& vers_n, const Vector3d& vel_OA_glob,
                        const Vector3d& pos_BA_glob)
{
  return Dot(vers_n, vel_OA_glob) / Norm(pos_BA_glob);
}

void updateTangAngSpeed(CableVarsBase& cable)
{
  cable.tan_ang_vel =
    calcTangAngSpeed(cable.vers_n, cable.vel_OA_glob, cable.pos_BA_glob);
}

void calcCableVersorsDot(const double pulley_radius, const Vector3d& vers_w,
                         const Vector3d& vers_n, const Vector3d& vers_t,
                         const Vector3d& vel_OA_glob, const double tan_ang,
                         const double tan_ang_vel, const double swivel_ang_vel,
                         Vector3d& vers_n_dot, Vector3d& vers_t_dot,
                         Vector3d& vel_BA_glob)
{
  vers_n_dot  = vers_w * cos(tan_ang) * swivel_ang_vel - vers_t * tan_ang_vel;
  vers_t_dot  = vers_w * sin(tan_ang) * swivel_ang_vel + vers_n * tan_ang_vel;
  vel_BA_glob = vel_OA_glob -
                pulley_radius *
                  (vers_w * (1. + cos(tan_ang)) * swivel_ang_vel - vers_t * tan_ang_vel);
}

void updateCableVersorsDot(const PulleyParams& params, CableVarsBase& cable)
{
  calcCableVersorsDot(params.radius, cable.vers_w, cable.vers_n, cable.vers_t,
                      cable.vel_OA_glob, cable.tan_ang, cable.tan_ang_vel,
                      cable.swivel_ang_vel, cable.vers_n_dot, cable.vers_t_dot,
                      cable.vel_BA_glob);
}

double calcCableSpeed(const Vector3d& vers_t, const Vector3d& vel_OA_glob)
{
  return Dot(vers_t, vel_OA_glob);
}

void updateCableSpeed(CableVarsBase& cable)
{
  cable.speed = calcCableSpeed(cable.vers_t, cable.vel_OA_glob);
}

void updateJacobiansRowD(const PlatformVars& platform, CableVars& cable)
{
  cable.geom_jacob_d_row.SetBlock<1, 3>(1, 1, cable.vers_t_dot.Transpose());
  cable.geom_jacob_d_row.SetBlock<1, 3>(
    1, 4,
    -cable.vers_t.Transpose() * Skew(platform.angular_vel) * Skew(cable.pos_PA_glob));

  cable.anal_jacob_d_row = cable.geom_jacob_d_row;
  cable.anal_jacob_d_row.SetBlock<1, 3>(
    1, 4,
    cable.anal_jacob_row.GetBlock<1, 3>(1, 4) * platform.h_mat -
      cable.vers_t.Transpose() * Skew(cable.pos_PA_glob) * platform.dh_mat);
}

void updateJacobiansRowD(const PlatformQuatVars& platform, CableVarsQuat& cable)
{
  cable.geom_jacob_d_row.SetBlock<1, 3>(1, 1, cable.vers_t_dot.Transpose());
  cable.geom_jacob_d_row.SetBlock<1, 3>(
    1, 4,
    -cable.vers_t.Transpose() * Skew(platform.angular_vel) * Skew(cable.pos_PA_glob));

  cable.anal_jacob_row.SetBlock<1, 3>(1, 1, cable.geom_jacob_row.GetBlock<1, 3>(1, 1));
  cable.anal_jacob_d_row.SetBlock<1, 4>(
    1, 4,
    cable.anal_jacob_row.GetBlock<1, 3>(1, 4) * platform.h_mat -
      cable.vers_t.Transpose() * Skew(cable.pos_PA_glob) * platform.dh_mat);
}

void updateCableFirstOrd(const PulleyParams& params, const PlatformVars& platform,
                         CableVars& cable)
{
  updateVelA(platform, cable);
  updateSwivelAngSpeed(cable);
  updatePulleyVersorsDot(cable);
  updateTangAngSpeed(cable);
  updateCableVersorsDot(params, cable);
  updateCableSpeed(cable);
  updateJacobiansRowD(platform, cable);
}

void updateCableFirstOrd(const PulleyParams& params, const PlatformQuatVars& platform,
                         CableVarsQuat& cable)
{
  updateVelA(platform, cable);
  updateSwivelAngSpeed(cable);
  updatePulleyVersorsDot(cable);
  updateTangAngSpeed(cable);
  updateCableVersorsDot(params, cable);
  updateCableSpeed(cable);
  updateJacobiansRowD(platform, cable);
}

void updateIK1(const Vector3d& velocity, const Vector3d& orientation_dot,
               const RobotParams& params, RobotVars& vars)
{
  updatePlatformVel(velocity, orientation_dot, vars.platform);
  // Safety check
  std::vector<id_t> active_actuators_id = params.activeActuatorsId();
  if (vars.geom_jacobian.n_rows != active_actuators_id.size())
    vars.geom_jacobian.resize(active_actuators_id.size(), POSE_DIM);
  if (vars.anal_jacobian.n_rows != active_actuators_id.size())
    vars.anal_jacobian.resize(active_actuators_id.size(), POSE_DIM);
  for (uint8_t i = 0; i < vars.cables.size(); ++i)
  {
    updateCableFirstOrd(params.actuators[active_actuators_id[i]].pulley, vars.platform,
                        (vars.cables[i]));
    vars.geom_jacobian_d.row(i) = arma::rowvec6(vars.cables[i].geom_jacob_d_row.Data());
    vars.anal_jacobian_d.row(i) = arma::rowvec6(vars.cables[i].anal_jacob_d_row.Data());
  }
}

void updateIK1(const Vector3d& velocity, const Vector4d& orientation_dot,
               const RobotParams& params, RobotVarsQuat& vars)
{
  updatePlatformVel(velocity, orientation_dot, vars.platform);
  // Safety check
  std::vector<id_t> active_actuators_id = params.activeActuatorsId();
  if (vars.geom_jacobian.n_rows != active_actuators_id.size())
    vars.geom_jacobian.resize(active_actuators_id.size(), POSE_DIM);
  if (vars.anal_jacobian.n_rows != active_actuators_id.size())
    vars.anal_jacobian.resize(active_actuators_id.size(), POSE_QUAT_DIM);
  for (uint8_t i = 0; i < vars.cables.size(); ++i)
  {
    updateCableFirstOrd(params.actuators[active_actuators_id[i]].pulley, vars.platform,
                        (vars.cables[i]));
    vars.geom_jacobian_d.row(i) = arma::rowvec6(vars.cables[i].geom_jacob_d_row.Data());
    vars.anal_jacobian_d.row(i) = arma::rowvec7(vars.cables[i].anal_jacob_d_row.Data());
  }
}

} // end namespace grabcdpr
