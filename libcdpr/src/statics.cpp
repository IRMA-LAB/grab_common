#include "statics.h"

arma::vec toArmaMat(Vector3d vect, bool copy /*= true*/)
{
  return arma::vec(vect.Data(), 3, copy);
}

arma::vec toArmaMat(VectorXd<POSE_DIM> vect, bool copy /*= true*/)
{
  return arma::vec(vect.Data(), POSE_DIM, copy);
}

arma::vec toArmaMat(VectorXd<POSE_QUAT_DIM> vect, bool copy /*= true*/)
{
  return arma::vec(vect.Data(), POSE_QUAT_DIM, copy);
}

arma::mat toArmaMat(Matrix3d vect, bool copy /*= true*/)
{
  return arma::mat(vect.Data(), 3, 3, copy);
}

namespace grabcdpr {

void UpdateExternalLoads(const grabnum::Matrix3d& R, const PlatformParams& params,
                         PlatformVars& platform)
{
  platform.ext_load.SetBlock<3, 1>(
    1, 1,
    R * (params.mass * params.gravity_acc + platform.rot_mat * params.ext_force_loc));
  platform.ext_load.SetBlock<3, 1>(4, 1,
                                   grabnum::Skew(platform.pos_PG_glob) *
                                       platform.ext_load.GetBlock<3, 1>(1, 1) +
                                     platform.rot_mat * params.ext_torque_loc);

  platform.ext_load_ss = platform.ext_load_ss;
  platform.ext_load_ss.SetBlock<3, 1>(
    4, 1, platform.h_mat.Transpose() * platform.ext_load.GetBlock<3, 1>(4, 1));
}

void UpdateExternalLoads(const grabnum::Matrix3d& R, const PlatformParams& params,
                         PlatformQuatVars& platform)
{
  platform.ext_load.SetBlock<3, 1>(
    1, 1,
    R * (params.mass * params.gravity_acc + platform.rot_mat * params.ext_force_loc));
  platform.ext_load.SetBlock<3, 1>(4, 1,
                                   grabnum::Skew(platform.pos_PG_glob) *
                                       platform.ext_load.GetBlock<3, 1>(1, 1) +
                                     platform.rot_mat * params.ext_torque_loc);

  platform.ext_load_ss = platform.ext_load_ss;
  platform.ext_load_ss.SetBlock<4, 1>(
    4, 1, platform.h_mat.Transpose() * platform.ext_load.GetBlock<3, 1>(4, 1));
}

void CalCablesTensionStat(RobotVars& vars)
{
  arma::mat A = -vars.geom_jabobian * vars.geom_jabobian.t();
  arma::vec b = -vars.geom_jabobian * toArmaMat(vars.platform.ext_load, false);
  arma::solve(A, b, vars.tension_vector);
}

void CalCablesTensionStat(RobotVarsQuat& vars)
{
  arma::mat A = -vars.geom_jabobian * vars.geom_jabobian.t();
  arma::vec b = -vars.geom_jabobian * toArmaMat(vars.platform.ext_load, false);
  arma::solve(A, b, vars.tension_vector);
}

void calcGeometricStatic(const RobotParams& params, const arma::vec& fixed_coord,
                         const arma::vec& var_coord, const VectorXi<POSE_DIM>& mask,
                         arma::mat& fun_jacobian, arma::vec& func_val)
{
  assert(fixed_coord.n_elem + var_coord.n_elem == POSE_DIM);

  ulong params_idx = 0;
  ulong vars_idx   = 0;
  VectorXd<POSE_DIM> pose;
  for (uint i = 0; i < pose.Size(); i++)
    if (mask(i) == 1)
      pose(i) = fixed_coord(params_idx++);
    else
      pose(i) = var_coord(vars_idx++);

  RobotVars vars;
  UpdateIK0(pose.GetBlock<3, 1>(1, 1), pose.GetBlock<3, 1>(4, 1), params, vars);
  UpdateExternalLoads(grabnum::Matrix3d(0.0), params.platform, vars.platform);

  const ulong kNumCables = params.actuators.size();
  arma::mat Ja(kNumCables, kNumCables, arma::fill::zeros);
  arma::mat Ju(kNumCables, POSE_DIM - kNumCables, arma::fill::zeros);
  arma::vec Wa(kNumCables, arma::fill::zeros);
  arma::vec Wu(POSE_DIM - kNumCables, arma::fill::zeros);

  ulong act_idx   = 0;
  ulong unact_idx = 0;
  for (uint i = 0; i < POSE_DIM; i++)
    if (mask(i + 1) == 1)
    {
      Ja.col(act_idx) = vars.geom_jabobian.col(i);
      Wa(act_idx++) = vars.platform.ext_load(i + 1); // index of grabnum vect starts at 1
    }
    else
    {
      Ju.col(unact_idx) = vars.geom_jabobian.col(i);
      Wu(unact_idx++) =
        vars.platform.ext_load(i + 1); // index of grabnum vect starts at 1
    }

  arma::vec tension_vector = arma::solve(Ja.t(), Wa); // linsolve Ax = b
  func_val                 = Ju.t() * tension_vector - Wu;
  fun_jacobian =
    CalcGsJacobians(vars, Ja, Ju, params.platform.mass * params.platform.gravity_acc);
}

void calcGeometricStatic(const RobotParams& params, const arma::vec& fixed_coord,
                         const arma::vec& var_coord, const VectorXi<POSE_QUAT_DIM>& mask,
                         arma::mat& fun_jacobian, arma::vec& func_val)
{
  assert(fixed_coord.n_elem + var_coord.n_elem == POSE_QUAT_DIM);

  ulong params_idx = 0;
  ulong vars_idx   = 0;
  VectorXd<POSE_QUAT_DIM> pose;
  for (uint i = 0; i < pose.Size(); i++)
    if (mask(i) == 1)
      pose(i) = fixed_coord(params_idx++);
    else
      pose(i) = var_coord(vars_idx++);

  RobotVarsQuat vars;
  UpdateIK0(pose.GetBlock<3, 1>(1, 1), pose.GetBlock<4, 1>(4, 1), params, vars);
  UpdateExternalLoads(grabnum::Matrix3d(0.0), params.platform, vars.platform);

  const ulong kNumCables = params.actuators.size();
  arma::mat Ja(kNumCables, kNumCables, arma::fill::zeros);
  arma::mat Ju(kNumCables, POSE_QUAT_DIM - kNumCables, arma::fill::zeros);
  arma::vec Wa(kNumCables, arma::fill::zeros);
  arma::vec Wu(POSE_QUAT_DIM - kNumCables, arma::fill::zeros);

  ulong act_idx   = 0;
  ulong unact_idx = 0;
  for (uint i = 0; i < POSE_QUAT_DIM; i++)
    if (mask(i + 1) == 1)
    {
      Ja.col(act_idx) = vars.geom_jabobian.col(i);
      Wa(act_idx++) = vars.platform.ext_load(i + 1); // index of grabnum vect starts at 1
    }
    else
    {
      Ju.col(unact_idx) = vars.geom_jabobian.col(i);
      Wu(unact_idx++) =
        vars.platform.ext_load(i + 1); // index of grabnum vect starts at 1
    }

  arma::vec tension_vector = arma::solve(Ja.t(), Wa); // linsolve Ax = b
  func_val                 = Ju.t() * tension_vector - Wu;
  fun_jacobian =
    CalcGsJacobians(vars, Ja, Ju, params.platform.mass * params.platform.gravity_acc);
}

arma::mat CalcGsJacobians(const RobotVars& vars, const arma::mat& Ja, const arma::mat& Ju,
                          const Vector3d& mg)
{
  const ulong n = Ja.n_cols; // num cables
  const ulong m = POSE_DIM;  // DOF

  arma::mat J_sl(2 * n, m, arma::fill::zeros);
  arma::mat dsdq(n, m, arma::fill::zeros);
  arma::mat dphidq(n, m, arma::fill::zeros);
  arma::mat dldq(n, m, arma::fill::zeros);
  arma::cube dJdq(m, n, m, arma::fill::none);
  for (ulong i = 0; i < static_cast<ulong>(n); i++)
  {
    grabnum::MatrixXd<3, POSE_DIM> dadq(1.0); // set diagonal to 1
    dadq.SetBlock<3, POSE_DIM - 3>(
      1, 4, -Skew(vars.cables[i].pos_PA_glob) * vars.platform.h_mat);
    dsdq.row(i) =
      arma::rowvec6((vars.cables[i].vers_w.Transpose() * dadq /
                     grabnum::Dot(vars.cables[i].pos_DA_glob, vars.cables[i].vers_u))
                      .Data());
    dphidq.row(i) = arma::rowvec6(
      (vars.cables[i].vers_n.Transpose() * dadq / Norm(vars.cables[i].pos_BA_glob))
        .Data());
    dldq.row(i) = arma::rowvec6((vars.cables[i].vers_t.Transpose() * dadq).Data());
    arma::mat dtdq =
      toArmaMat(sin(vars.cables[i].tan_ang) * vars.cables[i].vers_w) * dsdq.row(i) +
      toArmaMat(vars.cables[i].vers_n, false) * dphidq.row(i); // 3xm
    // dJTp 3xnxm (p->pose), matlab equivalent = dJdq(1:3,i,:)
    dJdq.tube(arma::span(0, 2), arma::span(i)) = dtdq;
    MatrixXd<3, POSE_DIM> dadql(0.0); // set diagonal to 0
    dadql.SetBlock<3, 3>(1, 4, -Skew(vars.cables[i].pos_PA_glob) * vars.platform.h_mat);
    // dJTo m-3xnxm (o->orient), matlab equivalent = dJdq(3:m,i,:)
    dJdq.tube(arma::span(3, m - 1), arma::span(i)) =
      toArmaMat(Skew(vars.cables[i].pos_PA_glob)) * dtdq -
      arma::mat((Skew(vars.cables[i].vers_t) * dadql).Data(), 3, POSE_DIM);
  }

  J_sl.rows(arma::span(0, n - 1))     = dsdq;
  J_sl.rows(arma::span(n, 2 * n - 1)) = dldq;
  arma::mat dfdq(m, m, arma::fill::zeros);
  dfdq(arma::span(3, m - 1), arma::span(3, m - 1)) =
    toArmaMat(Skew(mg) * Skew(vars.platform.pos_PG_glob) * vars.platform.h_mat);
  arma::mat dWa = dfdq.rows(arma::span(0, n - 1));
  arma::mat dWu = dfdq.rows(arma::span(n, m - 1));

  arma::mat j0u(m - n, m, arma::fill::zeros);
  arma::mat j0a(n, m, arma::fill::zeros);
  for (ulong i = 0; i < m; i++)
  {
    // Matlab equivalent = dJdq(1:n,:,i)
    j0a.col(i) = dJdq.slice(i).rows(arma::span(0, n - 1)) * vars.tension_vector;
    // Matlab equivalent = dJdq(n+1:end,:,i)
    j0u.col(i) = dJdq.slice(i).rows(arma::span(n, m - 1)) * vars.tension_vector;
  }

  arma::mat J_q = j0u + Ju.t() * arma::solve(Ja.t(), dWa - j0a) - dWu;
  return J_q.cols(arma::span(3, m - 1));
}

arma::mat CalcGsJacobians(const RobotVarsQuat& vars, const arma::mat& Ja,
                          const arma::mat& Ju, const Vector3d& mg)
{
  const ulong n = Ja.n_cols;     // num cables
  const ulong m = POSE_QUAT_DIM; // DOF

  arma::mat J_sl(2 * n, m, arma::fill::zeros);
  arma::mat dsdq(n, m, arma::fill::zeros);
  arma::mat dphidq(n, m, arma::fill::zeros);
  arma::mat dldq(n, m, arma::fill::zeros);
  arma::cube dJdq(m, n, m, arma::fill::none);
  for (ulong i = 0; i < static_cast<ulong>(n); i++)
  {
    grabnum::MatrixXd<3, POSE_QUAT_DIM> dadq(1.0); // set diagonal to 1
    dadq.SetBlock<3, POSE_QUAT_DIM - 3>(
      1, 4, -Skew(vars.cables[i].pos_PA_glob) * vars.platform.h_mat);
    dsdq.row(i) =
      arma::rowvec6((vars.cables[i].vers_w.Transpose() * dadq /
                     grabnum::Dot(vars.cables[i].pos_DA_glob, vars.cables[i].vers_u))
                      .Data());
    dphidq.row(i) = arma::rowvec6(
      (vars.cables[i].vers_n.Transpose() * dadq / Norm(vars.cables[i].pos_BA_glob))
        .Data());
    dldq.row(i) = arma::rowvec6((vars.cables[i].vers_t.Transpose() * dadq).Data());
    arma::mat dtdq =
      toArmaMat(sin(vars.cables[i].tan_ang) * vars.cables[i].vers_w) * dsdq.row(i) +
      toArmaMat(vars.cables[i].vers_n, false) * dphidq.row(i); // 3xm
    // dJTp 3xnxm (p->pose), matlab equivalent = dJdq(1:3,i,:)
    dJdq.tube(arma::span(0, 2), arma::span(i)) = dtdq;
    MatrixXd<3, POSE_QUAT_DIM> dadql(0.0); // set diagonal to 0
    dadql.SetBlock<3, 4>(1, 4, -Skew(vars.cables[i].pos_PA_glob) * vars.platform.h_mat);
    // dJTo m-3xnxm (o->orient), matlab equivalent = dJdq(3:m,i,:)
    dJdq.tube(arma::span(3, m - 1), arma::span(i)) =
      toArmaMat(Skew(vars.cables[i].pos_PA_glob)) * dtdq -
      arma::mat((Skew(vars.cables[i].vers_t) * dadql).Data(), 3, POSE_QUAT_DIM);
  }

  J_sl.rows(arma::span(0, n - 1))     = dsdq;
  J_sl.rows(arma::span(n, 2 * n - 1)) = dldq;
  arma::mat dfdq(m, m, arma::fill::zeros);
  dfdq(arma::span(3, m - 1), arma::span(3, m - 1)) = arma::mat(
    (Skew(mg) * Skew(vars.platform.pos_PG_glob) * vars.platform.h_mat).Data(), 3, 4);
  arma::mat dWa = dfdq.rows(arma::span(0, n - 1));
  arma::mat dWu = dfdq.rows(arma::span(n, m - 1));

  arma::mat j0u(m - n, m, arma::fill::zeros);
  arma::mat j0a(n, m, arma::fill::zeros);
  for (ulong i = 0; i < m; i++)
  {
    // Matlab equivalent = dJdq(1:n,:,i)
    j0a.col(i) = dJdq.slice(i).rows(arma::span(0, n - 1)) * vars.tension_vector;
    // Matlab equivalent = dJdq(n+1:end,:,i)
    j0u.col(i) = dJdq.slice(i).rows(arma::span(n, m - 1)) * vars.tension_vector;
  }

  arma::mat J_q = j0u + Ju.t() * arma::solve(Ja.t(), dWa - j0a) - dWu;
  return J_q.cols(arma::span(3, m - 1));
}

} // namespace grabcdpr
