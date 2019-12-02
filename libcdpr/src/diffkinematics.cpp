/**
 * @file diffkinematics.cpp
 * @author Edoardo Idà, Simone Comari
 * @date 02 Dec 2019
 * @brief File containing definitions of functions declared in diffkinematics.h.
 */

#include "diffkinematics.h"

namespace grabcdpr {

///////////////////////////////////////////////////////////////////////////////
//// Functions first-order kinematics
///////////////////////////////////////////////////////////////////////////////

void UpdatePlatformVel(const grabnum::Vector3d& velocity, const Vector3d& orientation_dot,
                       const grabnum::Vector3d& pos_PG_glob, PlatformVars& platform)
{
  // Update platform velocities.
  platform.UpdateVel(velocity, orientation_dot);
  // Calculate platform baricenter velocity expressed in global frame.
  platform.vel_OG_glob =
    platform.velocity + grabnum::Cross(platform.angular_vel, pos_PG_glob);
}

void UpdatePlatformVel(const grabnum::Vector3d& velocity,
                       const grabgeom::Quaternion& orientation_dot,
                       const grabnum::Vector3d& pos_PG_glob, PlatformQuatVars& platform)
{
  // Update platform velocities.
  platform.UpdateVel(velocity, orientation_dot);
  // Calculate platform baricenter velocity expressed in global frame.
  platform.vel_OG_glob =
    platform.velocity + grabnum::Cross(platform.angular_vel, pos_PG_glob);
}

void UpdatePlatformVel(const grabnum::Vector3d& velocity,
                       const grabnum::Vector3d& orientation_dot, PlatformVars& platform)
{
  UpdatePlatformVel(velocity, orientation_dot, platform.pos_PG_glob, platform);
}

void UpdatePlatformVel(const grabnum::Vector3d& velocity,
                       const grabgeom::Quaternion& orientation_dot,
                       PlatformQuatVars& platform)
{
  UpdatePlatformVel(velocity, orientation_dot, platform.pos_PG_glob, platform);
}

void UpdateVelA(const grabnum::Vector3d& pos_PA_glob, const PlatformVarsBase& platform,
                CableVarsBase& cable)
{
  cable.vel_OA_glob =
    platform.velocity + grabnum::Cross(platform.angular_vel, pos_PA_glob);
}

void UpdateVelA(const PlatformVarsBase& platform, CableVarsBase& cable)
{
  UpdateVelA(cable.pos_PA_glob, platform, cable);
}

void CalcPulleyVersorsDot(const grabnum::Vector3d& vers_u,
                          const grabnum::Vector3d& vers_w, const double swivel_ang_vel,
                          CableVarsBase& cable)
{
  cable.vers_u_dot = vers_w * swivel_ang_vel;
  cable.vers_w_dot = -vers_u * swivel_ang_vel;
}

void CalcPulleyVersorsDot(CableVarsBase& cable)
{
  CalcPulleyVersorsDot(cable.vers_u, cable.vers_w, cable.swivel_ang_vel, cable);
}

double CalcSwivelAngSpeed(const grabnum::Vector3d& vers_u,
                          const grabnum::Vector3d& vers_w,
                          const grabnum::Vector3d& vel_OA_glob,
                          const grabnum::Vector3d& pos_DA_glob)
{
  return grabnum::Dot(vers_w, vel_OA_glob) / grabnum::Dot(vers_u, pos_DA_glob);
}

double CalcSwivelAngSpeed(const CableVarsBase& cable)
{
  return CalcSwivelAngSpeed(cable.vers_u, cable.vers_w, cable.vel_OA_glob,
                            cable.pos_DA_glob);
}

double CalcTangAngSpeed(const grabnum::Vector3d& vers_n,
                        const grabnum::Vector3d& vel_OA_glob,
                        const grabnum::Vector3d& pos_BA_glob)
{
  return grabnum::Dot(vers_n, vel_OA_glob) / grabnum::Norm(pos_BA_glob);
}

double CalcTangAngSpeed(const CableVarsBase& cable)
{
  return CalcTangAngSpeed(cable.vers_n, cable.vel_OA_glob, cable.pos_BA_glob);
}

void CalcCableVersorsDot(const double pulley_radius, const grabnum::Vector3d& vers_w,
                         const grabnum::Vector3d& vers_n, const grabnum::Vector3d& vers_t,
                         const double tan_ang, const double tan_ang_vel,
                         const double swivel_ang_vel, CableVarsBase& cable)
{
  cable.vers_n_dot = vers_w * cos(tan_ang) * swivel_ang_vel - vers_t * tan_ang_vel;
  cable.vers_n_dot = vers_w * sin(tan_ang) * swivel_ang_vel + vers_n * tan_ang_vel;
  cable.vel_BA_glob =
    cable.vel_OA_glob -
    pulley_radius * (cable.vers_w * (1. + cos(cable.tan_ang)) * cable.swivel_ang_vel -
                     cable.vers_t * cable.tan_ang_vel);
}

void CalcCableVersorsDot(const PulleyParams& params, CableVarsBase& cable)
{
  CalcCableVersorsDot(params.radius, cable.vers_w, cable.vers_n, cable.vers_t,
                      cable.tan_ang, cable.tan_ang_vel, cable.swivel_ang_vel, cable);
}

double CalcCableSpeed(const grabnum::Vector3d& vers_t,
                      const grabnum::Vector3d& vel_OA_glob)
{
  return grabnum::Dot(vers_t, vel_OA_glob);
}

double CalcCableSpeed(const CableVarsBase& cable)
{
  return CalcCableSpeed(cable.vers_t, cable.vel_OA_glob);
}

void UpdateJacobiansRowD(const PlatformVars& platform, CableVars& cable)
{
  cable.geom_jacob_d_row.SetBlock<1, 3>(1, 1, cable.vers_t_dot.Transpose());
  cable.geom_jacob_d_row.SetBlock<1, 3>(1, 4,
                                        -cable.vers_t.Transpose() *
                                          grabnum::Skew(platform.angular_vel) *
                                          grabnum::Skew(cable.pos_PA_glob));

  cable.anal_jacob_d_row = cable.geom_jacob_d_row;
  cable.anal_jacob_d_row.SetBlock<1, 3>(
    1, 4,
    cable.anal_jacob_row.GetBlock<1, 3>(1, 4) * platform.h_mat -
      cable.vers_t.Transpose() * grabnum::Skew(cable.pos_PA_glob) * platform.dh_mat);
}

void UpdateJacobiansRowD(const PlatformQuatVars& platform, CableVarsQuat& cable)
{
  cable.geom_jacob_d_row.SetBlock<1, 3>(1, 1, cable.vers_t_dot.Transpose());
  cable.geom_jacob_d_row.SetBlock<1, 3>(1, 4,
                                        -cable.vers_t.Transpose() *
                                          grabnum::Skew(platform.angular_vel) *
                                          grabnum::Skew(cable.pos_PA_glob));

  cable.anal_jacob_row.SetBlock<1, 3>(1, 1, cable.geom_jacob_row.GetBlock<1, 3>(1, 1));
  cable.anal_jacob_d_row.SetBlock<1, 4>(
    1, 4,
    cable.anal_jacob_row.GetBlock<1, 3>(1, 4) * platform.h_mat -
      cable.vers_t.Transpose() * grabnum::Skew(cable.pos_PA_glob) * platform.dh_mat);
}

void UpdateCableFirstOrd(const PulleyParams& params, const PlatformVars& platform,
                         CableVars& cable)
{
  UpdateVelA(platform, cable);
  cable.swivel_ang_vel = CalcSwivelAngSpeed(cable);
  CalcPulleyVersorsDot(cable);
  cable.tan_ang_vel = CalcTangAngSpeed(cable);
  CalcCableVersorsDot(params, cable);
  cable.speed = CalcCableSpeed(cable);
  UpdateJacobiansRowD(platform, cable);
}

void UpdateCableFirstOrd(const PulleyParams& params, const PlatformQuatVars& platform,
                         CableVarsQuat& cable)
{
  UpdateVelA(platform, cable);
  cable.swivel_ang_vel = CalcSwivelAngSpeed(cable);
  CalcPulleyVersorsDot(cable);
  cable.tan_ang_vel = CalcTangAngSpeed(cable);
  CalcCableVersorsDot(params, cable);
  cable.speed = CalcCableSpeed(cable);
  UpdateJacobiansRowD(platform, cable);
}

void UpdateIK1(const grabnum::Vector3d& velocity, const Vector3d& orientation_dot,
               const RobotParams& params, RobotVars& vars)
{
  UpdatePlatformVel(velocity, orientation_dot, vars.platform);
  // Safety check
  std::vector<id_t> active_actuators_id = params.activeActuatorsId();
  if (vars.geom_jacobian.n_rows != active_actuators_id.size())
    vars.geom_jacobian.resize(active_actuators_id.size(), POSE_DIM);
  if (vars.anal_jacobian.n_rows != active_actuators_id.size())
    vars.anal_jacobian.resize(active_actuators_id.size(), POSE_DIM);
  for (uint8_t i = 0; i < vars.cables.size(); ++i)
  {
    UpdateCableFirstOrd(params.actuators[active_actuators_id[i]].pulley, vars.platform,
                        (vars.cables[i]));
    vars.geom_jacobian_d.row(i) = arma::rowvec6(vars.cables[i].geom_jacob_d_row.Data());
    vars.anal_jacobian_d.row(i) = arma::rowvec6(vars.cables[i].anal_jacob_d_row.Data());
  }
}

void UpdateIK1(const grabnum::Vector3d& velocity, const Vector4d& orientation_dot,
               const RobotParams& params, RobotVarsQuat& vars)
{
  UpdatePlatformVel(velocity, orientation_dot, vars.platform);
  // Safety check
  std::vector<id_t> active_actuators_id = params.activeActuatorsId();
  if (vars.geom_jacobian.n_rows != active_actuators_id.size())
    vars.geom_jacobian.resize(active_actuators_id.size(), POSE_DIM);
  if (vars.anal_jacobian.n_rows != active_actuators_id.size())
    vars.anal_jacobian.resize(active_actuators_id.size(), POSE_QUAT_DIM);
  for (uint8_t i = 0; i < vars.cables.size(); ++i)
  {
    UpdateCableFirstOrd(params.actuators[active_actuators_id[i]].pulley, vars.platform,
                        (vars.cables[i]));
    vars.geom_jacobian_d.row(i) = arma::rowvec6(vars.cables[i].geom_jacob_d_row.Data());
    vars.anal_jacobian_d.row(i) = arma::rowvec7(vars.cables[i].anal_jacob_d_row.Data());
  }
}

///////////////////////////////////////////////////////////////////////////////
//// Functions second-order kinematics
///////////////////////////////////////////////////////////////////////////////

void UpdatePlatformAcc(const grabnum::Vector3d& acceleration,
                       const grabnum::Vector3d& orientation_ddot,
                       const grabnum::Vector3d& pos_PG_glob, PlatformVars& platform)
{
  // Update platform velocities.
  platform.UpdateAcc(acceleration, orientation_ddot);
  // Calculate platform baricenter velocity expressed in global frame.
  grabnum::Matrix3d Omega = grabnum::Skew(platform.angular_vel);
  platform.acc_OG_glob =
    acceleration + (grabnum::Skew(platform.angular_acc) + Omega * Omega) * pos_PG_glob;
}

void UpdatePlatformAcc(const grabnum::Vector3d& acceleration,
                       const grabgeom::Quaternion& quat_acc,
                       const grabnum::Vector3d& pos_PG_glob, PlatformQuatVars& platform)
{
  // Update platform velocities.
  platform.UpdateAcc(acceleration, quat_acc);
  // Calculate platform baricenter velocity expressed in global frame.
  grabnum::Matrix3d Omega = grabnum::Skew(platform.angular_vel);
  platform.acc_OG_glob =
    acceleration + (grabnum::Skew(platform.angular_acc) + Omega * Omega) * pos_PG_glob;
}

void UpdatePlatformAcc(const grabnum::Vector3d& acceleration,
                       const grabnum::Vector3d& orientation_ddot, PlatformVars& platform)
{
  UpdatePlatformAcc(acceleration, orientation_ddot, platform.pos_PG_glob, platform);
}

void UpdatePlatformAcc(const grabnum::Vector3d& acceleration,
                       const grabgeom::Quaternion& quat_acc, PlatformQuatVars& platform)
{
  UpdatePlatformAcc(acceleration, quat_acc, platform.pos_PG_glob, platform);
}

void UpdateAccA(const grabnum::Vector3d& pos_PA_glob, const PlatformVarsBase& platform,
                CableVarsBase& cable)
{
  grabnum::Matrix3d Omega = grabnum::Skew(platform.angular_vel);
  cable.acc_OA_glob       = platform.acceleration +
                      (grabnum::Skew(platform.angular_acc) + Omega * Omega) * pos_PA_glob;
}

void UpdateAccA(const PlatformVarsBase& platform, CableVarsBase& cable)
{
  UpdateAccA(cable.pos_PA_glob, platform, cable);
}

double CalcSwivelAngAcc(const grabnum::Vector3d& vers_u, const grabnum::Vector3d& vers_w,
                        const grabnum::Vector3d& vel_OA_glob,
                        const grabnum::Vector3d& pos_DA_glob,
                        const grabnum::Vector3d& acc_OA_glob, const double swivel_ang_vel)
{
  return (grabnum::Dot(vers_w, acc_OA_glob) -
          2. * grabnum::Dot(vers_u, vel_OA_glob) * swivel_ang_vel) /
         grabnum::Dot(vers_u, pos_DA_glob);
}

double CalcSwivelAngAcc(const CableVarsBase& cable)
{
  return CalcSwivelAngAcc(cable.vers_u, cable.vers_w, cable.vel_OA_glob,
                          cable.pos_DA_glob, cable.acc_OA_glob, cable.swivel_ang_vel);
}

double CalcTangAngAcc(const double pulley_radius, const grabnum::Vector3d& vers_u,
                      const grabnum::Vector3d& vers_n,
                      const grabnum::Vector3d& pos_DA_glob,
                      const grabnum::Vector3d& pos_BA_glob,
                      const grabnum::Vector3d& acc_OA_glob, const double speed,
                      const double tan_ang, const double tan_ang_vel,
                      const double swivel_ang_vel)
{
  return (grabnum::Dot(vers_n, acc_OA_glob) +
          grabnum::Dot(vers_u, pos_DA_glob) * cos(tan_ang) * swivel_ang_vel *
            swivel_ang_vel -
          (2. * speed + pulley_radius * tan_ang_vel) * tan_ang_vel) /
         grabnum::Norm(pos_BA_glob);
}

double CalcTangAngAcc(const double pulley_radius, const CableVarsBase& cable)
{
  return CalcTangAngAcc(pulley_radius, cable.vers_u, cable.vers_n, cable.pos_DA_glob,
                        cable.pos_BA_glob, cable.acc_OA_glob, cable.speed, cable.tan_ang,
                        cable.tan_ang_vel, cable.swivel_ang_vel);
}

double CalcCableAcc(const grabnum::Vector3d& vers_u, const grabnum::Vector3d& vers_t,
                    const grabnum::Vector3d& pos_DA_glob,
                    const grabnum::Vector3d& pos_BA_glob,
                    const grabnum::Vector3d& acc_OA_glob, const double tan_ang,
                    const double tan_ang_vel, const double swivel_ang_vel)
{
  return grabnum::Dot(vers_u, pos_DA_glob) * sin(tan_ang) * SQUARE(swivel_ang_vel) +
         grabnum::Norm(pos_BA_glob) * SQUARE(tan_ang_vel) +
         grabnum::Dot(vers_t, acc_OA_glob);
}

double CalcCableAcc(const CableVarsBase& cable)
{
  return CalcCableAcc(cable.vers_u, cable.vers_t, cable.pos_DA_glob, cable.pos_BA_glob,
                      cable.acc_OA_glob, cable.tan_ang, cable.tan_ang_vel,
                      cable.swivel_ang_vel);
}

void UpdateCableSecondOrd(const PulleyParams& params, const PlatformVarsBase& platform,
                          CableVarsBase& cable)
{
  UpdateAccA(cable.pos_PA_glob, platform, cable);
  cable.tan_ang_acc    = CalcTangAngAcc(params.radius, cable);
  cable.swivel_ang_acc = CalcSwivelAngAcc(cable);
  cable.acceleration   = CalcCableAcc(cable);
}

void UpdateIK2(const grabnum::Vector3d& acceleration,
               const grabnum::Vector3d& orientation_ddot, const RobotParams& params,
               RobotVars& vars)
{
  UpdatePlatformAcc(acceleration, orientation_ddot, vars.platform);
  for (uint8_t i = 0; i < vars.cables.size(); ++i)
    UpdateCableSecondOrd(params.actuators[i].pulley, vars.platform, vars.cables[i]);
}

void UpdateIK2(const grabnum::Vector3d& acceleration,
               const grabgeom::Quaternion& orientation_ddot, const RobotParams& params,
               RobotVarsQuat& vars)
{
  UpdatePlatformAcc(acceleration, orientation_ddot, vars.platform);
  for (uint8_t i = 0; i < vars.cables.size(); ++i)
    UpdateCableSecondOrd(params.actuators[i].pulley, vars.platform, vars.cables[i]);
}

void UpdateIK(const grabnum::Vector3d& position, const grabnum::Vector3d& orientation,
              const grabnum::Vector3d& velocity, const grabnum::Vector3d& orientation_dot,
              const grabnum::Vector3d& acceleration,
              const grabnum::Vector3d& orientation_ddot, const RobotParams& params,
              RobotVars& vars)
{
  UpdatePlatformPose(position, orientation, params.platform, vars.platform);
  UpdatePlatformVel(velocity, orientation_dot, vars.platform);
  UpdatePlatformAcc(acceleration, orientation_ddot, vars.platform);
  for (uint8_t i = 0; i < vars.cables.size(); ++i)
  {
    UpdateCableZeroOrd(params.actuators[i], vars.platform, vars.cables[i]);
    UpdateCableFirstOrd(params.actuators[i].pulley, vars.platform, vars.cables[i]);
    UpdateCableSecondOrd(params.actuators[i].pulley, vars.platform, vars.cables[i]);
  }
}

void UpdateIK(const grabnum::Vector3d& position, const grabgeom::Quaternion& orientation,
              const grabnum::Vector3d& velocity,
              const grabgeom::Quaternion& orientation_dot,
              const grabnum::Vector3d& acceleration,
              const grabgeom::Quaternion& orientation_ddot, const RobotParams& params,
              RobotVarsQuat& vars)
{
  UpdatePlatformPose(position, orientation, params.platform, vars.platform);
  UpdatePlatformVel(velocity, orientation_dot, vars.platform);
  UpdatePlatformAcc(acceleration, orientation_ddot, vars.platform);
  for (uint8_t i = 0; i < vars.cables.size(); ++i)
  {
    UpdateCableZeroOrd(params.actuators[i], vars.platform, vars.cables[i]);
    UpdateCableFirstOrd(params.actuators[i].pulley, vars.platform, vars.cables[i]);
    UpdateCableSecondOrd(params.actuators[i].pulley, vars.platform, vars.cables[i]);
  }
}

} // end namespace grabcdpr
