/**
 * @file diff2kinematics.cpp
 * @author Edoardo Idà, Simone Comari
 * @date 02 Dec 2019
 * @brief File containing definitions of functions declared in diff2kinematics.h.
 */

#include "diff2kinematics.h"

namespace grabcdpr {

///////////////////////////////////////////////////////////////////////////////
//// Functions second-order kinematics
///////////////////////////////////////////////////////////////////////////////

void updatePlatformAcc(const Vector3d& acceleration, const Vector3d& orientation_ddot,
                       const Vector3d& pos_PG_glob, PlatformVars& platform)
{
  // Update platform velocities.
  platform.updateAcc(acceleration, orientation_ddot);
  // Calculate platform baricenter velocity expressed in global frame.
  Matrix3d Omega = Skew(platform.angular_vel);
  platform.acc_OG_glob =
    acceleration + (Skew(platform.angular_acc) + Omega * Omega) * pos_PG_glob;
}

void updatePlatformAcc(const Vector3d& acceleration, const grabgeom::Quaternion& quat_acc,
                       const Vector3d& pos_PG_glob, PlatformVarsQuat& platform)
{
  // Update platform velocities.
  platform.updateAcc(acceleration, quat_acc);
  // Calculate platform baricenter velocity expressed in global frame.
  Matrix3d Omega = Skew(platform.angular_vel);
  platform.acc_OG_glob =
    acceleration + (Skew(platform.angular_acc) + Omega * Omega) * pos_PG_glob;
}

void updatePlatformAcc(const Vector3d& acceleration, const Vector3d& orientation_ddot,
                       PlatformVars& platform)
{
  updatePlatformAcc(acceleration, orientation_ddot, platform.pos_PG_glob, platform);
}

void updatePlatformAcc(const Vector3d& acceleration, const grabgeom::Quaternion& quat_acc,
                       PlatformVarsQuat& platform)
{
  updatePlatformAcc(acceleration, quat_acc, platform.pos_PG_glob, platform);
}

Vector3d calcAccA(const Vector3d& pos_PA_glob, const PlatformVarsBase& platform)
{
  Matrix3d Omega = Skew(platform.angular_vel);
  Vector3d acc_OA_glob =
    platform.linear_acc + (Skew(platform.angular_acc) + Omega * Omega) * pos_PA_glob;
  return acc_OA_glob;
}

void updateAccA(const PlatformVarsBase& platform, CableVarsBase& cable)
{
  cable.acc_OA_glob = calcAccA(cable.pos_PA_glob, platform);
}

double calcSwivelAngAcc(const Vector3d& vers_u, const Vector3d& vers_w,
                        const Vector3d& vel_OA_glob, const Vector3d& pos_DA_glob,
                        const Vector3d& acc_OA_glob, const double swivel_ang_vel)
{
  return (Dot(vers_w, acc_OA_glob) - 2. * Dot(vers_u, vel_OA_glob) * swivel_ang_vel) /
         Dot(vers_u, pos_DA_glob);
}

void updateSwivelAngAcc(CableVarsBase& cable)
{
  cable.swivel_ang_acc =
    calcSwivelAngAcc(cable.vers_u, cable.vers_w, cable.vel_OA_glob, cable.pos_DA_glob,
                     cable.acc_OA_glob, cable.swivel_ang_vel);
}

double calcTangAngAcc(const double pulley_radius, const Vector3d& vers_u,
                      const Vector3d& vers_n, const Vector3d& pos_DA_glob,
                      const Vector3d& pos_BA_glob, const Vector3d& acc_OA_glob,
                      const double speed, const double tan_ang, const double tan_ang_vel,
                      const double swivel_ang_vel)
{
  return (Dot(vers_n, acc_OA_glob) +
          Dot(vers_u, pos_DA_glob) * cos(tan_ang) * swivel_ang_vel * swivel_ang_vel -
          (2. * speed + pulley_radius * tan_ang_vel) * tan_ang_vel) /
         Norm(pos_BA_glob);
}

void updateTangAngAcc(const double pulley_radius, CableVarsBase& cable)
{
  cable.tan_ang_acc =
    calcTangAngAcc(pulley_radius, cable.vers_u, cable.vers_n, cable.pos_DA_glob,
                   cable.pos_BA_glob, cable.acc_OA_glob, cable.speed, cable.tan_ang,
                   cable.tan_ang_vel, cable.swivel_ang_vel);
}

double calcCableAcc(const Vector3d& vers_u, const Vector3d& vers_t,
                    const Vector3d& pos_DA_glob, const Vector3d& pos_BA_glob,
                    const Vector3d& acc_OA_glob, const double tan_ang,
                    const double tan_ang_vel, const double swivel_ang_vel)
{
  return Dot(vers_u, pos_DA_glob) * sin(tan_ang) * SQUARE(swivel_ang_vel) +
         Norm(pos_BA_glob) * SQUARE(tan_ang_vel) + Dot(vers_t, acc_OA_glob);
}

void updateCableAcc(CableVarsBase& cable)
{
  cable.acceleration = calcCableAcc(cable.vers_u, cable.vers_t, cable.pos_DA_glob,
                                    cable.pos_BA_glob, cable.acc_OA_glob, cable.tan_ang,
                                    cable.tan_ang_vel, cable.swivel_ang_vel);
}

void updateCableSecondOrd(const PulleyParams& params, const PlatformVarsBase& platform,
                          CableVarsBase& cable)
{
  updateAccA(platform, cable);
  updateTangAngAcc(params.radius, cable);
  updateSwivelAngAcc(cable);
  updateCableAcc(cable);
}

void updateIK2(const Vector3d& acceleration, const Vector3d& orientation_ddot,
               const RobotParams& params, RobotVars& vars)
{
  updatePlatformAcc(acceleration, orientation_ddot, vars.platform);
  for (uint8_t i = 0; i < vars.cables.size(); ++i)
    updateCableSecondOrd(params.actuators[i].pulley, vars.platform, vars.cables[i]);
}

void updateIK2(const Vector3d& acceleration, const grabgeom::Quaternion& orientation_ddot,
               const RobotParams& params, RobotVarsQuat& vars)
{
  updatePlatformAcc(acceleration, orientation_ddot, vars.platform);
  for (uint8_t i = 0; i < vars.cables.size(); ++i)
    updateCableSecondOrd(params.actuators[i].pulley, vars.platform, vars.cables[i]);
}


///////////////////////////////////////////////////////////////////////////////
//// Functions full kinematics
///////////////////////////////////////////////////////////////////////////////

void updateIK(const Vector3d& position, const Vector3d& orientation,
              const Vector3d& velocity, const Vector3d& orientation_dot,
              const Vector3d& acceleration, const Vector3d& orientation_ddot,
              const RobotParams& params, RobotVars& vars)
{
  updatePlatformPose(position, orientation, params.platform, vars.platform);
  updatePlatformVel(velocity, orientation_dot, vars.platform);
  updatePlatformAcc(acceleration, orientation_ddot, vars.platform);
  for (uint8_t i = 0; i < vars.cables.size(); ++i)
  {
    updateCableZeroOrd(params.actuators[i], vars.platform, vars.cables[i]);
    updateCableFirstOrd(params.actuators[i].pulley, vars.platform, vars.cables[i]);
    updateCableSecondOrd(params.actuators[i].pulley, vars.platform, vars.cables[i]);
  }
}

void updateIK(const Vector3d& position, const grabgeom::Quaternion& orientation,
              const Vector3d& velocity, const grabgeom::Quaternion& orientation_dot,
              const Vector3d& acceleration, const grabgeom::Quaternion& orientation_ddot,
              const RobotParams& params, RobotVarsQuat& vars)
{
  updatePlatformPose(position, orientation, params.platform, vars.platform);
  updatePlatformVel(velocity, orientation_dot, vars.platform);
  updatePlatformAcc(acceleration, orientation_ddot, vars.platform);
  for (uint8_t i = 0; i < vars.cables.size(); ++i)
  {
    updateCableZeroOrd(params.actuators[i], vars.platform, vars.cables[i]);
    updateCableFirstOrd(params.actuators[i].pulley, vars.platform, vars.cables[i]);
    updateCableSecondOrd(params.actuators[i].pulley, vars.platform, vars.cables[i]);
  }
}

} // end namespace grabcdpr
