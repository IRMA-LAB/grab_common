/**
 * @file kinematics.cpp
 * @author Edoardo Idà, Simone Comari
 * @date 07 Feb 2020
 * @brief File containing definitions of functions declared in kinematics.h.
 */

#include "kinematics.h"

namespace grabcdpr {

void updatePlatformPose(const Vector3d& position,
                        const Vector3d& orientation,
                        const Vector3d& pos_PG_loc, PlatformVars& platform)
{
  // Update platform pose.
  platform.updatePose(position, orientation);
  // Calculate platform baricenter positions expressed in global frame.
  platform.pos_PG_glob = platform.rot_mat * pos_PG_loc;
  platform.pos_OG_glob = platform.position + platform.pos_PG_glob;
}

void updatePlatformPose(const Vector3d& position,
                        const grabgeom::Quaternion& orientation,
                        const Vector3d& pos_PG_loc, PlatformVarsQuat& platform)
{
  // Update platform pose.
  platform.updatePose(position, orientation);
  // Calculate platform baricenter positions expressed in global frame.
  platform.pos_PG_glob = platform.rot_mat * pos_PG_loc;
  platform.pos_OG_glob = platform.position + platform.pos_PG_glob;
}

void updatePlatformPose(const Vector3d& position,
                        const Vector3d& orientation,
                        const PlatformParams& params, PlatformVars& platform)
{
  updatePlatformPose(position, orientation, params.pos_PG_loc, platform);
}

void updatePlatformPose(const Vector3d& position,
                        const grabgeom::Quaternion& orientation,
                        const PlatformParams& params, PlatformVarsQuat& platform)
{
  updatePlatformPose(position, orientation, params.pos_PG_loc, platform);
}

void calcPosA(const ActuatorParams& params, const PlatformVarsBase& platform,
              Vector3d& pos_PA_glob, Vector3d& pos_OA_glob, Vector3d& pos_DA_glob)
{
  pos_PA_glob = platform.rot_mat * params.winch.pos_PA_loc;
  pos_OA_glob = platform.position + pos_PA_glob;
  pos_DA_glob = pos_OA_glob - params.pulley.pos_OD_glob;
}

void updatePosA(const ActuatorParams& params, const PlatformVarsBase& platform,
                CableVarsBase& cable)
{
  calcPosA(params, platform, cable.pos_PA_glob, cable.pos_OA_glob, cable.pos_DA_glob);
}

void calcPulleyVersors(const PulleyParams& params, const double swivel_ang,
                       Vector3d& vers_u, Vector3d& vers_w)
{
  double cos_sigma = cos(swivel_ang);
  double sin_sigma = sin(swivel_ang);
  vers_u           = params.vers_i * cos_sigma + params.vers_j * sin_sigma;
  vers_w           = -params.vers_i * sin_sigma + params.vers_j * cos_sigma;
}

void updatePulleyVersors(const PulleyParams& params, CableVarsBase& cable)
{
  calcPulleyVersors(params, cable.swivel_ang, cable.vers_u, cable.vers_w);
}

double calcSwivelAngle(const PulleyParams& params, const Vector3d& pos_DA_glob)
{
  return atan2(Dot(params.vers_j, pos_DA_glob),
               Dot(params.vers_i, pos_DA_glob));
}

void updateSwivelAngle(const PulleyParams& params, CableVarsBase& cable)
{
  cable.swivel_ang = calcSwivelAngle(params, cable.pos_DA_glob);
}

double calcTangentAngle(const PulleyParams& params, const Vector3d& vers_u,
                        const Vector3d& pos_DA_glob)
{
  double s       = Dot(vers_u, pos_DA_glob);
  double app_var = Dot(params.vers_k, pos_DA_glob) / s;
  double psi = 2. * atan(app_var + sqrt(1. - 2. * params.radius / s + SQUARE(app_var)));
  return psi;
}

void updateTangentAngle(const PulleyParams& params, CableVarsBase& cable)
{
  cable.tan_ang = calcTangentAngle(params, cable.vers_u, cable.pos_DA_glob);
}

void calcCableVectors(const PulleyParams& params, const Vector3d& vers_u,
                      const Vector3d& pos_DA_glob, const double tan_ang,
                      Vector3d& vers_n, Vector3d& vers_t, Vector3d& pos_BA_glob)
{
  // Versors describing cable exit direction from swivel pulley.
  double cos_psi = cos(tan_ang);
  double sin_psi = sin(tan_ang);
  vers_n         = vers_u * cos_psi + params.vers_k * sin_psi;
  vers_t         = vers_u * sin_psi - params.vers_k * cos_psi;
  // Vector from swivel pulley exit point to platform attaching point.
  pos_BA_glob = pos_DA_glob - params.radius * (vers_u + vers_n);
}

void updateCableVectors(const PulleyParams& params, CableVarsBase& cable)
{
  calcCableVectors(params, cable.vers_u, cable.pos_DA_glob, cable.tan_ang, cable.vers_n,
                   cable.vers_t, cable.pos_BA_glob);
}

double calcCableLen(const double pulley_radius, const Vector3d& pos_BA_glob,
                    const double tan_ang)
{
  return Norm(pos_BA_glob) + pulley_radius * (M_PI - tan_ang);
}

void updateCableLen(const PulleyParams& params, CableVarsBase& cable)
{
  cable.length = calcCableLen(params.radius, cable.pos_BA_glob, cable.tan_ang);
}

double calcMotorCounts(const double tau, const double cable_len,
                       const double pulley_radius, const double tan_ang)
{
  return (cable_len + pulley_radius * (M_PI - tan_ang)) / tau;
}

double calcMotorCounts(const ActuatorParams& params, const CableVarsBase& cable)
{
  return calcMotorCounts(params.winch.transmission_ratio, cable.length,
                         params.pulley.radius, cable.tan_ang);
}

void updateJacobiansRow(const Matrix3d H_mat, CableVars& cable)
{
  cable.geom_jacob_row.SetBlock<1, 3>(1, 1, cable.vers_t.Transpose());
  cable.geom_jacob_row.SetBlock<1, 3>(
    1, 4, -cable.vers_t.Transpose() * Skew(cable.pos_PA_glob));

  cable.anal_jacob_row = cable.geom_jacob_row;
  cable.anal_jacob_row.SetBlock<1, 3>(1, 4,
                                      cable.anal_jacob_row.GetBlock<1, 3>(1, 4) * H_mat);
}

void updateJacobiansRow(const MatrixXd<3, 4> H_mat, CableVarsQuat& cable)
{
  cable.geom_jacob_row.SetBlock<1, 3>(1, 1, cable.vers_t.Transpose());
  cable.geom_jacob_row.SetBlock<1, 3>(
    1, 4, -cable.vers_t.Transpose() * Skew(cable.pos_PA_glob));

  cable.anal_jacob_row.SetBlock<1, 3>(1, 1, cable.geom_jacob_row.GetBlock<1, 3>(1, 1));
  cable.anal_jacob_row.SetBlock<1, 4>(1, 4,
                                      cable.geom_jacob_row.GetBlock<1, 3>(1, 4) * H_mat);
}

void updateCableZeroOrd(const ActuatorParams& params, const PlatformVars& platform,
                        CableVars& cable)
{
  updatePosA(params, platform, cable);     // update segments ending with point A_i.
  updateSwivelAngle(params.pulley, cable); // from 1st kinematic constraint.
  updatePulleyVersors(params.pulley, cable);
  updateTangentAngle(params.pulley, cable); // from 2nd kinematic constraint.
  updateCableVectors(params.pulley, cable); // from 1st kinematic constraint.
  updateCableLen(params.pulley, cable);     // from 3rd kinematic constraint.
  updateJacobiansRow(platform.h_mat, cable);
}

void updateCableZeroOrd(const ActuatorParams& params, const PlatformVarsQuat& platform,
                        CableVarsQuat& cable)
{
  updatePosA(params, platform, cable);     // update segments ending with point A_i.
  updateSwivelAngle(params.pulley, cable); // from 1st kinematic constraint.
  updatePulleyVersors(params.pulley, cable);
  updateTangentAngle(params.pulley, cable); // from 2nd kinematic constraint.
  updateCableVectors(params.pulley, cable); // from 1st kinematic constraint.
  updateCableLen(params.pulley, cable);     // from 3rd kinematic constraint.
  updateJacobiansRow(platform.h_mat, cable);
}

void updateIK0(const Vector3d& position, const Vector3d& orientation,
               const RobotParams& params, RobotVars& vars)
{
  updatePlatformPose(position, orientation, params.platform, vars.platform);
  std::vector<id_t> active_actuators_id = params.activeActuatorsId();
  for (uint8_t i = 0; i < active_actuators_id.size(); ++i)
    updateCableZeroOrd(params.actuators[active_actuators_id[i]], vars.platform,
                       vars.cables[i]);
  vars.updateJacobians();
}

void updateIK0(const Vector6d& pose, const RobotParams& params, RobotVars& vars)
{
  updateIK0(pose.HeadRows<3>(), pose.TailRows<3>(), params, vars);
}

void updateIK0(const arma::vec6& _pose, const RobotParams& params, RobotVars& vars)
{
  Vector6d pose(_pose.begin(), _pose.end());
  updateIK0(pose, params, vars);
}

void updateIK0(const Vector3d& position, const grabgeom::Quaternion& orientation,
               const RobotParams& params, RobotVarsQuat& vars)
{
  updatePlatformPose(position, orientation, params.platform, vars.platform);
  std::vector<id_t> active_actuators_id = params.activeActuatorsId();
  for (uint8_t i = 0; i < active_actuators_id.size(); ++i)
    updateCableZeroOrd(params.actuators[active_actuators_id[i]], vars.platform,
                       vars.cables[i]);
  vars.updateJacobians();
}

arma::mat calcJacobianL(const RobotVars& vars) { return vars.anal_jacobian; }

arma::mat calcJacobianSw(const RobotVars& vars)
{
  arma::mat jacobian_sw(arma::size(vars.anal_jacobian), arma::fill::none);
  for (uint i = 0; i < jacobian_sw.n_rows; ++i)
  {
    arma::rowvec temp =
      arma::join_horiz(toArmaVec(vars.cables[i].vers_w).t(),
                       toArmaVec(-vars.cables[i].vers_w.Transpose() *
                                 Skew(vars.cables[i].pos_PA_glob) * vars.platform.h_mat));
    jacobian_sw.row(i) =
      temp / Dot(vars.cables[i].vers_u, vars.cables[i].pos_DA_glob);
  }
  return jacobian_sw;
}

void optFunDK0(const RobotParams& params, const arma::vec& cables_length,
               const arma::vec& swivel_angles, const arma::vec6& pose,
               arma::mat& fun_jacobian, arma::vec& fun_val)
{
  const ulong kNumCables = params.activeActuatorsNum();
  RobotVars vars(kNumCables, params.platform.rot_parametrization);
  updateIK0(pose, params, vars);

  arma::vec l_constraints(kNumCables, arma::fill::none);
  arma::vec sw_constraints(kNumCables, arma::fill::none);
  for (uint i = 0; i < kNumCables; ++i)
  {
    l_constraints(i)  = vars.cables[i].length - cables_length[i];
    sw_constraints(i) = vars.cables[i].swivel_ang - swivel_angles[i];
  }

  arma::mat l_jacobian  = calcJacobianL(vars);
  arma::mat sw_jacobian = calcJacobianSw(vars);

  fun_val      = arma::join_vert(l_constraints, sw_constraints);
  fun_jacobian = arma::join_vert(l_jacobian, sw_jacobian);
}

bool solveDK0(const std::vector<double>& cables_length,
              const std::vector<double>& swivel_angles,
              const VectorXd<POSE_DIM>& init_guess_pose,
              const RobotParams& params, VectorXd<POSE_DIM>& platform_pose,
              const uint8_t nmax /*= 100*/, uint8_t* iter_out /*= nullptr*/)
{
  static const double kFtol = 1e-6;
  static const double kXtol = 1e-6;

  // First round to init function value and jacobian
  arma::vec func_val;
  arma::mat func_jacob;
  arma::vec6 pose = toArmaVec(init_guess_pose);
  optFunDK0(params, cables_length, swivel_angles, pose, func_jacob, func_val);

  // Init iteration variables
  arma::vec s;
  uint8_t iter = 0;
  double err   = 1.0;
  double cond  = 0.0;
  // Start iterative process
  while (arma::norm(func_val) > kFtol && err > cond)
  {
    if (iter >= nmax)
      return false; // did not converge
    iter++;
    s = arma::solve(func_jacob, func_val);
    pose -= s;
    optFunDK0(params, cables_length, swivel_angles, pose, func_jacob, func_val);
    err  = arma::norm(s);
    cond = kXtol * (1 + arma::norm(pose));
  }

  if (iter_out != nullptr)
    *iter_out = iter;

  platform_pose.Fill(pose.begin(), pose.end());

  return true;
}

bool updateDK0(const RobotParams& params, RobotVars& vars)
{
  // Extract starting conditions from latest robot configuration
  // Cable's variables are expected to be up-to-date
  std::vector<double> cables_length(vars.cables.size(), 0);
  std::vector<double> swivel_angles(vars.cables.size(), 0);
  for (uint i = 0; i < vars.cables.size(); ++i)
  {
    cables_length[i] = vars.cables[i].length;
    swivel_angles[i] = vars.cables[i].swivel_ang;
  }
  // While platform pose is expected to be the latest known/computed value, so not updated
  VectorXd<POSE_DIM> init_guess_pose = vars.platform.pose;

  // Solve direct kinematics
  VectorXd<POSE_DIM> new_pose;
  if (solveDK0(cables_length, swivel_angles, init_guess_pose, params, new_pose))
  {
    // Update inverse kinematics
    updateIK0(new_pose, params, vars);
    return true;
  }
  // Could not solve optimization (failed direct kinematics)
  return false;
}

} // end namespace grabcdpr
