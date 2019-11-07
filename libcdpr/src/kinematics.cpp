/**
 * @file kinematics.cpp
 * @author Edoardo Idà, Simone Comari
 * @date 31 Oct 2019
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
    1, 4, -cable.vers_t.Transpose() * grabnum::Skew(cable.pos_PA_glob));

  cable.anal_jacob_row = cable.geom_jacob_row;
  cable.anal_jacob_row.SetBlock<1, 3>(1, 4,
                                      cable.anal_jacob_row.GetBlock<1, 3>(1, 4) * H_mat);
}

void UpdateJacobiansRow(const grabnum::MatrixXd<3, 4> H_mat, CableVarsQuat& cable)
{
  cable.geom_jacob_row.SetBlock<1, 3>(1, 1, cable.vers_t.Transpose());
  cable.geom_jacob_row.SetBlock<1, 3>(
    1, 4, -cable.vers_t.Transpose() * grabnum::Skew(cable.pos_PA_glob));

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
    vars.anal_jabobian.row(i) = arma::rowvec6(vars.cables[i].anal_jacob_row.Data());
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
    vars.anal_jabobian.row(i) = arma::rowvec7(vars.cables[i].anal_jacob_row.Data());
  }
}

void CalcDK0Jacobians(const RobotVars& vars, const arma::mat& Ja, arma::mat& J_sl)
{
  static constexpr ulong m = POSE_DIM;  // DOF
  const ulong n            = Ja.n_cols; // num cables

  J_sl = arma::mat(2 * n, m, arma::fill::zeros);
  for (ulong i = 0; i < static_cast<ulong>(n); i++)
  {
    grabnum::MatrixXd<3, m> dadq(1.0); // set diagonal to 1
    dadq.SetBlock<3, m - 3>(1, 4,
                            -Skew(vars.cables[i].pos_PA_glob) * vars.platform.h_mat);
    grabnum::MatrixXd<1, m> dsdq =
      vars.cables[i].vers_w.Transpose() * dadq /
      grabnum::Dot(vars.cables[i].pos_DA_glob, vars.cables[i].vers_u);
    grabnum::MatrixXd<1, m> dldq = vars.cables[i].vers_t.Transpose() * dadq;

    J_sl.row(i)     = arma::rowvec6(dsdq.Data());
    J_sl.row(n + i) = arma::rowvec6(dldq.Data());
  }
}

void CalcDK0Jacobians(const RobotVarsQuat& vars, const arma::mat& Ja, arma::mat& J_sl)
{
  static constexpr ulong m = POSE_QUAT_DIM; // DOF
  const ulong n            = Ja.n_cols;     // num cables

  J_sl = arma::mat(2 * n, m, arma::fill::zeros);
  for (ulong i = 0; i < static_cast<ulong>(n); i++)
  {
    grabnum::MatrixXd<3, m> dadq(1.0); // set diagonal to 1
    dadq.SetBlock<3, m - 3>(1, 4,
                            -Skew(vars.cables[i].pos_PA_glob) * vars.platform.h_mat);
    grabnum::MatrixXd<1, m> dsdq =
      vars.cables[i].vers_w.Transpose() * dadq /
      grabnum::Dot(vars.cables[i].pos_DA_glob, vars.cables[i].vers_u);
    grabnum::MatrixXd<1, m> dldq = vars.cables[i].vers_t.Transpose() * dadq;

    J_sl.row(i)     = arma::rowvec7(dsdq.Data());
    J_sl.row(n + i) = arma::rowvec7(dldq.Data());
  }
}

static void DK0OptimizationFunc(const RobotParams& params, const arma::vec& cables_length,
                                const arma::vec& swivel_angles, const arma::vec6& pose,
                                arma::mat& fun_jacobian, arma::vec& func_val)
{
  const ulong kNumCables = params.activeActuatorsNum();
  RobotVars vars(kNumCables, params.platform.rot_parametrization);
  UpdateIK0(fromArmaVec3(pose.head(3)), fromArmaVec3(pose.tail(3)), params, vars);

  arma::mat Ja = vars.geom_jabobian.cols(arma::span(0, kNumCables - 1));
  arma::mat J_sl;
  CalcDK0Jacobians(vars, Ja, J_sl); // only kinematics

  func_val = arma::zeros(2 * kNumCables);
  for (uint i = 0; i < kNumCables; ++i)
  {
    func_val(i)              = vars.cables[i].swivel_ang - swivel_angles(i);
    func_val(i + kNumCables) = vars.cables[i].length - cables_length(i);
  }

  fun_jacobian = J_sl; // [J_sigma; J_l] --> J_sl --> (2 * NumCables, POSE_DIM)
}

void CalcRobustDK0Jacobians(const RobotVars& vars, const arma::mat& Ja,
                            const arma::mat& Ju, const Vector3d& mg, arma::mat& J_q,
                            arma::mat& J_sl)
{
  static constexpr ulong m = POSE_DIM;  // DOF
  const ulong n            = Ja.n_cols; // num cables = controlled DOF

  J_sl = arma::mat(2 * n, m, arma::fill::zeros);
  grabnum::MatrixXd<m, m> dJT;     // init to zeros by default
  grabnum::MatrixXd<m, m> dJT_add; // init to zeros by default
  for (ulong i = 0; i < static_cast<ulong>(n); i++)
  {
    grabnum::MatrixXd<3, m> dadq(1.0); // set diagonal to 1
    dadq.SetBlock<3, m - 3>(1, 4,
                            -Skew(vars.cables[i].pos_PA_glob) * vars.platform.h_mat);
    grabnum::MatrixXd<1, m> dsdq =
      vars.cables[i].vers_w.Transpose() * dadq /
      grabnum::Dot(vars.cables[i].pos_DA_glob, vars.cables[i].vers_u);
    grabnum::MatrixXd<1, m> dphidq =
      vars.cables[i].vers_n.Transpose() * dadq / Norm(vars.cables[i].pos_BA_glob);
    grabnum::MatrixXd<1, m> dldq = vars.cables[i].vers_t.Transpose() * dadq;
    grabnum::MatrixXd<3, m> dtdq =
      sin(vars.cables[i].tan_ang) * vars.cables[i].vers_w * dsdq +
      vars.cables[i].vers_n * dphidq; // 3xm
    dadq.SetBlock<3, 3>(1, 1, grabnum::Matrix3d(0.0));

    dJT_add.SetBlock<3, m>(1, 1, dtdq);
    dJT_add.SetBlock<m - 3, m>(
      4, 1,
      (Skew(vars.cables[i].pos_PA_glob) * dtdq - Skew(vars.cables[i].vers_t) * dadq));
    dJT += (dJT_add * vars.tension_vector(i));

    J_sl.row(i)     = arma::rowvec6(dsdq.Data());
    J_sl.row(n + i) = arma::rowvec6(dldq.Data());
  }

  grabnum::MatrixXd<m, m> dfdq; // init to zeros by default
  dfdq.SetBlock<3, m - 3>(
    4, 4, Skew(mg) * Skew(vars.platform.pos_PG_glob) * vars.platform.h_mat);
  arma::mat dfdq_arma = arma::mat(dfdq.Data(), m, m).t();

  // J_q = (m-n)xm
  arma::mat dJT_arma = arma::mat(dJT.Data(), m, m).t();
  J_q                = dJT_arma.tail_rows(m - n) +
        Ju.t() * arma::solve(Ja.t(), dfdq_arma.head_rows(n) - dJT_arma.head_rows(n)) -
        dfdq_arma.tail_rows(m - n);
}

void CalcRobustDK0Jacobians(const RobotVarsQuat& vars, const arma::mat& Ja,
                            const arma::mat& Ju, const Vector3d& mg, arma::mat& J_q,
                            arma::mat& J_sl)
{
  static constexpr ulong m = POSE_QUAT_DIM; // DOF
  const ulong n            = Ja.n_cols;     // num cables

  J_sl = arma::mat(2 * n, m, arma::fill::zeros);
  grabnum::MatrixXd<m, m> dJT;     // init to zeros by default
  grabnum::MatrixXd<m, m> dJT_add; // init to zeros by default
  for (ulong i = 0; i < static_cast<ulong>(n); i++)
  {
    grabnum::MatrixXd<3, m> dadq(1.0); // set diagonal to 1
    dadq.SetBlock<3, m - 3>(1, 4,
                            -Skew(vars.cables[i].pos_PA_glob) * vars.platform.h_mat);
    grabnum::MatrixXd<1, m> dsdq =
      vars.cables[i].vers_w.Transpose() * dadq /
      grabnum::Dot(vars.cables[i].pos_DA_glob, vars.cables[i].vers_u);
    grabnum::MatrixXd<1, m> dphidq =
      vars.cables[i].vers_n.Transpose() * dadq / Norm(vars.cables[i].pos_BA_glob);
    grabnum::MatrixXd<1, m> dldq = vars.cables[i].vers_t.Transpose() * dadq;
    grabnum::MatrixXd<3, m> dtdq =
      sin(vars.cables[i].tan_ang) * vars.cables[i].vers_w * dsdq +
      vars.cables[i].vers_n * dphidq; // 3xm
    dadq.SetBlock<3, 3>(1, 1, grabnum::Matrix3d(0.0));

    dJT_add.SetBlock<3, m>(1, 1, dtdq);
    // TODO: solve this dims mismatch
    //    dJT_add.SetBlock<m - 3, m>(
    //      4, 1,
    //      (Skew(vars.cables[i].pos_PA_glob) * dtdq - Skew(vars.cables[i].vers_t) *
    //      dadq));
    dJT += (dJT_add * vars.tension_vector(i));

    J_sl.row(i)     = arma::rowvec7(dsdq.Data());
    J_sl.row(n + i) = arma::rowvec7(dldq.Data());
  }

  grabnum::MatrixXd<m, m> dfdq; // init to zeros by default
  dfdq.SetBlock<3, m - 3>(
    4, 4, Skew(mg) * Skew(vars.platform.pos_PG_glob) * vars.platform.h_mat);
  arma::mat dfdq_arma = arma::mat(dfdq.Data(), m, m).t();

  // J_q = (m-n)xm
  arma::mat dJT_arma = arma::mat(dJT.Data(), m, m).t();
  J_q                = dJT_arma.tail_rows(m - n) +
        Ju.t() * arma::solve(Ja.t(), dfdq_arma.head_rows(n) - dJT_arma.head_rows(n)) -
        dfdq_arma.tail_rows(m - n);
}

static void RobustDK0OptimizationFunc(const RobotParams& params,
                                      const arma::vec& cables_length,
                                      const arma::vec& swivel_angles,
                                      const arma::vec6& pose, arma::mat& fun_jacobian,
                                      arma::vec& func_val)
{
  const ulong kNumCables = params.activeActuatorsNum();  // = num. controlled DOF
  const ulong kNumUncontrolledDof = POSE_DIM - kNumCables;
  RobotVars vars(kNumCables, params.platform.rot_parametrization);
  UpdateIK0(fromArmaVec3(pose.head(3)), fromArmaVec3(pose.tail(3)), params, vars);
  UpdateExternalLoads(grabnum::Matrix3d(1.0), params.platform, vars.platform);

  arma::mat Ja       = vars.geom_jabobian.head_cols(kNumCables);
  arma::mat Ju       = vars.geom_jabobian.tail_cols(kNumUncontrolledDof);
  arma::vec ext_load = toArmaVec(vars.platform.ext_load);
  arma::vec Wa       = ext_load.head(kNumCables);
  arma::vec Wu       = ext_load.tail(kNumUncontrolledDof);
  vars.tension_vector = arma::solve(Ja.t(), Wa); // linsolve Ax = b
  arma::mat J_sl, J_q;
  CalcRobustDK0Jacobians(vars, Ja, Ju, params.platform.mass * params.platform.gravity_acc,
                         J_q, J_sl); // kinematics + statics

  func_val = arma::zeros(kNumCables + POSE_DIM); // = 2*NumCables+(NumUncontrolledDof)
  for (uint i = 0; i < kNumCables; ++i)
  {
    func_val(i)              = vars.cables[i].swivel_ang - swivel_angles(i); // f_sigma
    func_val(i + kNumCables) = vars.cables[i].length - cables_length(i);     // f_l
  }
  func_val.tail(kNumUncontrolledDof) = Ju.t() * vars.tension_vector - Wu;

  fun_jacobian                           = arma::zeros(kNumCables + POSE_DIM, POSE_DIM);
  fun_jacobian.head_rows(2 * kNumCables) = J_sl; // [J_sigma; J_l] --> J_sl (kinematics)
  fun_jacobian.tail_rows(kNumUncontrolledDof) = J_q; // (statics)
}

bool SolveDK0(const std::vector<double>& cables_length,
              const std::vector<double>& swivel_angles,
              const grabnum::VectorXd<POSE_DIM>& init_guess_pose,
              const RobotParams& params, VectorXd<POSE_DIM> &platform_pose,
              const bool use_gs_jacob /*=false*/, const uint8_t nmax /*= 100*/,
              uint8_t* iter_out /*= nullptr*/)
{
  static const double kFtol = 1e-6;
  static const double kXtol = 1e-6;

  // First round to init function value and jacobian
  arma::vec func_val;
  arma::mat func_jacob;
  arma::vec6 pose = toArmaVec(init_guess_pose);
  if (use_gs_jacob)
    grabcdpr::RobustDK0OptimizationFunc(params, cables_length, swivel_angles, pose,
                                        func_jacob, func_val);
  else
    grabcdpr::DK0OptimizationFunc(params, cables_length, swivel_angles, pose, func_jacob,
                                  func_val);

  // Init iteration variables
  arma::vec s;
  uint8_t iter = 0;
  double err   = 1.0;
  double cond  = 0.0;
  // Start iterative process
  while (arma::norm(func_val) > kFtol && err > cond)
  {
    if (iter >= nmax)
      return false;
    iter++;
    s = arma::solve(func_jacob, func_val);
    pose -= s;
    if (use_gs_jacob)
      grabcdpr::RobustDK0OptimizationFunc(params, cables_length, swivel_angles, pose,
                                          func_jacob, func_val);
    else
      grabcdpr::DK0OptimizationFunc(params, cables_length, swivel_angles, pose,
                                    func_jacob, func_val);
    err  = arma::norm(s);
    cond = kXtol * (1 + arma::norm(pose));
  }

  if (iter_out != nullptr)
    *iter_out = iter;

  platform_pose.Fill(pose.begin(), pose.end());

  return true;
}

void UpdateDK0(const RobotParams& params, RobotVars& vars,
               const bool use_gs_jacob /*=false*/)
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
  grabnum::VectorXd<POSE_DIM> init_guess_pose = vars.platform.pose;

  // Solve direct kinematics
  grabnum::VectorXd<POSE_DIM> new_pose =
    SolveDK0(cables_length, swivel_angles, init_guess_pose, params, use_gs_jacob);
  // Update inverse kinematics (?)
  grabcdpr::UpdateIK0(new_pose.GetBlock<3, 1>(1, 1), new_pose.GetBlock<3, 1>(4, 1),
                      params, vars);
}

} // end namespace grabcdpr
