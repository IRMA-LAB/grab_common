/**
 * @file statics.cpp
 * @author Simone Comari
 * @date 06 Feb 2020
 * @brief File containing definitions of functions declared in statics.h.
 */

#include "statics.h"

namespace grabcdpr {

arma::vec calcCablesStaticTension(const arma::mat& geom_jacobian,
                                  const Vector6d& ext_load)
{
  arma::mat A              = geom_jacobian * geom_jacobian.t();
  arma::vec b              = geom_jacobian * toArmaVec(ext_load);
  arma::vec tension_vector = arma::solve(A, b);
  // Tensions cannot be negative
  if (tension_vector.min() < 0.0)
  {
    PrintColor('y', "WARNING: negative cables tension! Values will be clamped to 0.");
    tension_vector = arma::clamp(tension_vector, 0.0, tension_vector.max());
  }
  return tension_vector;
}

void updateCablesStaticTension(RobotVars& vars)
{
  vars.tension_vector =
    calcCablesStaticTension(vars.geom_jacobian, vars.platform.ext_load);
}

void updateCablesStaticTension(RobotVarsQuat& vars)
{
  vars.tension_vector =
    calcCablesStaticTension(vars.geom_jacobian, vars.platform.ext_load);
}

void calcGeometricStatic(const RobotParams& params, const arma::vec& fixed_coord,
                         const arma::vec& var_coord, const VectorXi<POSE_DIM>& mask,
                         arma::mat& fun_jacobian, arma::vec& func_val)
{
  assert(fixed_coord.n_elem + var_coord.n_elem == POSE_DIM);

  ulong params_idx = 0;
  ulong vars_idx   = 0;
  VectorXd<POSE_DIM> pose;
  for (uint i = 1; i <= pose.Size(); i++)
    if (mask(i) == 1)
      pose(i) = fixed_coord(params_idx++);
    else
      pose(i) = var_coord(vars_idx++);

  const ulong kNumCables = params.activeActuatorsNum();
  RobotVars vars(kNumCables, params.platform.rot_parametrization);
  updateIK0(pose.GetBlock<3, 1>(1, 1), pose.GetBlock<3, 1>(4, 1), params, vars);
  updateExternalLoads(params.platform, vars.platform);

  arma::mat Ja(kNumCables, kNumCables, arma::fill::zeros);
  arma::mat Ju(kNumCables, POSE_DIM - kNumCables, arma::fill::zeros);
  arma::vec Wa(kNumCables, arma::fill::zeros);
  arma::vec Wu(POSE_DIM - kNumCables, arma::fill::zeros);

  ulong act_idx   = 0;
  ulong unact_idx = 0;
  for (uint i = 0; i < POSE_DIM; i++)
    if (mask(i + 1) == 1)
    {
      Ja.col(act_idx) = vars.geom_jacobian.col(i);
      Wa(act_idx++) = vars.platform.ext_load(i + 1); // index of grabnum vect starts at 1
    }
    else
    {
      Ju.col(unact_idx) = vars.geom_jacobian.col(i);
      Wu(unact_idx++) =
        vars.platform.ext_load(i + 1); // index of grabnum vect starts at 1
    }

  vars.tension_vector = arma::solve(Ja.t(), Wa); // linsolve Ax = b
  func_val            = Ju.t() * vars.tension_vector - Wu;
  calcGsJacobians(vars, Ja, Ju, params.platform.mass * params.platform.gravity_acc,
                  fun_jacobian);
}

void calcGeometricStatic(const RobotParams& params, const arma::vec& fixed_coord,
                         const arma::vec& var_coord, const VectorXi<POSE_QUAT_DIM>& mask,
                         arma::mat& fun_jacobian, arma::vec& func_val)
{
  assert(fixed_coord.n_elem + var_coord.n_elem == POSE_QUAT_DIM);

  ulong params_idx = 0;
  ulong vars_idx   = 0;
  VectorXd<POSE_QUAT_DIM> pose;
  for (uint i = 1; i <= pose.Size(); i++)
    if (mask(i) == 1)
      pose(i) = fixed_coord(params_idx++);
    else
      pose(i) = var_coord(vars_idx++);

  RobotVarsQuat vars(params.activeActuatorsNum());
  updateIK0(pose.GetBlock<3, 1>(1, 1), pose.GetBlock<4, 1>(4, 1), params, vars);
  updateExternalLoads(params.platform, vars.platform);

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
      Ja.col(act_idx) = vars.geom_jacobian.col(i);
      Wa(act_idx++) = vars.platform.ext_load(i + 1); // index of grabnum vect starts at 1
    }
    else
    {
      Ju.col(unact_idx) = vars.geom_jacobian.col(i);
      Wu(unact_idx++) =
        vars.platform.ext_load(i + 1); // index of grabnum vect starts at 1
    }

  arma::vec tension_vector = arma::solve(Ja.t(), Wa); // linsolve Ax = b
  func_val                 = Ju.t() * tension_vector - Wu;
  calcGsJacobians(vars, Ja, Ju, params.platform.mass * params.platform.gravity_acc,
                  fun_jacobian);
}

void calcGsJacobians(const RobotVars& vars, const arma::mat& Ja, const arma::mat& Ju,
                     const Vector3d& mg, arma::mat& J_q)
{
  static constexpr ulong m = POSE_DIM;  // DOF
  const ulong n            = Ja.n_cols; // num cables

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
    grabnum::MatrixXd<3, m> dtdq =
      sin(vars.cables[i].tan_ang) * vars.cables[i].vers_w * dsdq +
      vars.cables[i].vers_n * dphidq; // 3xm
    dadq.SetBlock<3, 3>(1, 1, grabnum::Matrix3d(0.0));

    dJT_add.SetBlock<3, m>(1, 1, dtdq);
    dJT_add.SetBlock<m - 3, m>(
      4, 1,
      (Skew(vars.cables[i].pos_PA_glob) * dtdq - Skew(vars.cables[i].vers_t) * dadq));
    dJT += (dJT_add * vars.tension_vector(i));
  }

  grabnum::MatrixXd<m, m> dfdq; // init to zeros by default
  dfdq.SetBlock<3, m - 3>(
    4, 4, Skew(mg) * Skew(vars.platform.pos_PG_glob) * vars.platform.h_mat);
  arma::mat dfdq_arma = arma::mat(dfdq.Data(), m, m).t();

  // J_q = (m-n)x(m-n)
  arma::mat dJT_arma = arma::mat(dJT.Data(), m, m).t();
  J_q =
    dJT_arma(arma::span(n, m - 1), arma::span(n, m - 1)) +
    Ju.t() * arma::solve(Ja.t(), dfdq_arma(arma::span(0, n - 1), arma::span(n, m - 1)) -
                                   dJT_arma(arma::span(0, n - 1), arma::span(n, m - 1))) -
    dfdq_arma(arma::span(n, m - 1), arma::span(n, m - 1));
}

void calcGsJacobians(const RobotVars& vars, const arma::mat& Ja, const arma::mat& Ju,
                     const Vector3d& mg, arma::mat& J_q, arma::mat& J_sl)
{
  static constexpr ulong m = POSE_DIM;  // DOF
  const ulong n            = Ja.n_cols; // num cables

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

  // J_q = (m-n)x(m-n)
  arma::mat dJT_arma = arma::mat(dJT.Data(), m, m).t();
  J_q =
    dJT_arma(arma::span(n, m - 1), arma::span(n, m - 1)) +
    Ju.t() * arma::solve(Ja.t(), dfdq_arma(arma::span(0, n - 1), arma::span(n, m - 1)) -
                                   dJT_arma(arma::span(0, n - 1), arma::span(n, m - 1))) -
    dfdq_arma(arma::span(n, m - 1), arma::span(n, m - 1));
}

void calcGsJacobians(const RobotVarsQuat& vars, const arma::mat& Ja, const arma::mat& Ju,
                     const Vector3d& mg, arma::mat& J_q)
{
  static constexpr ulong m = POSE_QUAT_DIM; // DOF
  const ulong n            = Ja.n_cols;     // num cables

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
    grabnum::MatrixXd<3, m> dtdq =
      sin(vars.cables[i].tan_ang) * vars.cables[i].vers_w * dsdq +
      vars.cables[i].vers_n * dphidq; // 3xm
    dadq.SetBlock<3, 3>(1, 1, grabnum::Matrix3d(0.0));

    dJT_add.SetBlock<3, m>(1, 1, dtdq);
    dJT += (dJT_add * vars.tension_vector(i));
  }

  grabnum::MatrixXd<m, m> dfdq; // init to zeros by default
  dfdq.SetBlock<3, m - 3>(
    4, 4, Skew(mg) * Skew(vars.platform.pos_PG_glob) * vars.platform.h_mat);
  arma::mat dfdq_arma = arma::mat(dfdq.Data(), m, m).t();

  // J_q = (m-n)xm
  arma::mat dJT_arma = arma::mat(dJT.Data(), m, m).t();
  J_q =
    dJT_arma(arma::span(n, m - 1), arma::span(n, m - 1)) +
    Ju.t() * arma::solve(Ja.t(), dfdq_arma(arma::span(0, n - 1), arma::span(n, m - 1)) -
                                   dJT_arma(arma::span(0, n - 1), arma::span(n, m - 1))) -
    dfdq_arma(arma::span(n, m - 1), arma::span(n, m - 1));
}

void calcGsJacobians(const RobotVarsQuat& vars, const arma::mat& Ja, const arma::mat& Ju,
                     const Vector3d& mg, arma::mat& J_q, arma::mat& J_sl)
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
  J_q =
    dJT_arma(arma::span(n, m - 1), arma::span(n, m - 1)) +
    Ju.t() * arma::solve(Ja.t(), dfdq_arma(arma::span(0, n - 1), arma::span(n, m - 1)) -
                                   dJT_arma(arma::span(0, n - 1), arma::span(n, m - 1))) -
    dfdq_arma(arma::span(n, m - 1), arma::span(n, m - 1));
}

arma::mat calcGsJacobiansOld(const RobotVars& vars, const arma::mat& Ja,
                             const arma::mat& Ju, const Vector3d& mg)
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
      toArmaVec(sin(vars.cables[i].tan_ang) * vars.cables[i].vers_w) * dsdq.row(i) +
      toArmaVec(vars.cables[i].vers_n) * dphidq.row(i); // 3xm
    // dJTp 3xnxm (p->pose), matlab equivalent = dJdq(1:3,i,:)
    dJdq.tube(arma::span(0, 2), arma::span(i)) = dtdq;
    MatrixXd<3, POSE_DIM> dadql(0.0); // set diagonal to 0
    dadql.SetBlock<3, 3>(1, 4, -Skew(vars.cables[i].pos_PA_glob) * vars.platform.h_mat);
    // dJTo m-3xnxm (o->orient), matlab equivalent = dJdq(4:m,i,:)
    dJdq.tube(arma::span(3, m - 1), arma::span(i)) =
      toArmaMat(Skew(vars.cables[i].pos_PA_glob)) * dtdq -
      arma::mat((Skew(vars.cables[i].vers_t) * dadql).Transpose().Data(), 3,
                POSE_DIM); // note: transpose because data is copied column-by-column
  }

  J_sl.head_rows(n) = dsdq;
  J_sl.tail_rows(n) = dldq;
  arma::mat dfdq(m, m, arma::fill::zeros);
  dfdq(arma::span(3, m - 1), arma::span(3, m - 1)) =
    toArmaMat(Skew(mg) * Skew(vars.platform.pos_PG_glob) * vars.platform.h_mat);
  arma::mat dWa = dfdq.head_rows(n);
  arma::mat dWu = dfdq.tail_rows(m - n);

  arma::mat j0u(m - n, m, arma::fill::zeros);
  arma::mat j0a(n, m, arma::fill::zeros);
  for (ulong i = 0; i < m; i++)
  {
    // Matlab equivalent = dJdq(1:n,:,i)
    j0a.col(i) = dJdq.slice(i).head_rows(n) * vars.tension_vector;
    // Matlab equivalent = dJdq(n+1:end,:,i)
    j0u.col(i) = dJdq.slice(i).tail_rows(m - n) * vars.tension_vector;
  }

  arma::mat J_q = j0u + Ju.t() * arma::solve(Ja.t(), dWa - j0a) - dWu;
  return J_q.tail_cols(m - n);
}

arma::mat calcGsJacobiansOld(const RobotVarsQuat& vars, const arma::mat& Ja,
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
      toArmaVec(sin(vars.cables[i].tan_ang) * vars.cables[i].vers_w) * dsdq.row(i) +
      toArmaVec(vars.cables[i].vers_n, false) * dphidq.row(i); // 3xm
    // dJTp 3xnxm (p->pose), matlab equivalent = dJdq(1:3,i,:)
    dJdq.tube(arma::span(0, 2), arma::span(i)) = dtdq;
    MatrixXd<3, POSE_QUAT_DIM> dadql(0.0); // set diagonal to 0
    dadql.SetBlock<3, 4>(1, 4, -Skew(vars.cables[i].pos_PA_glob) * vars.platform.h_mat);
    // dJTo m-3xnxm (o->orient), matlab equivalent = dJdq(3:m,i,:)
    dJdq.tube(arma::span(3, m - 1), arma::span(i)) =
      toArmaMat(Skew(vars.cables[i].pos_PA_glob)) * dtdq -
      arma::mat((Skew(vars.cables[i].vers_t) * dadql).Data(), 3, POSE_QUAT_DIM);
  }

  J_sl.head_rows(n) = dsdq;
  J_sl.tail_rows(n) = dldq;
  arma::mat dfdq(m, m, arma::fill::zeros);
  dfdq(arma::span(3, m - 1), arma::span(3, m - 1)) = arma::mat(
    (Skew(mg) * Skew(vars.platform.pos_PG_glob) * vars.platform.h_mat).Transpose().Data(),
    3, 4); // note: transpose because data is copied column-by-column
  arma::mat dWa = dfdq.head_rows(n);
  arma::mat dWu = dfdq.tail_rows(m - n);

  arma::mat j0u(m - n, m, arma::fill::zeros);
  arma::mat j0a(n, m, arma::fill::zeros);
  for (ulong i = 0; i < m; i++)
  {
    // Matlab equivalent = dJdq(1:n,:,i)
    j0a.col(i) = dJdq.slice(i).head_rows(n) * vars.tension_vector;
    // Matlab equivalent = dJdq(n+1:end,:,i)
    j0u.col(i) = dJdq.slice(i).tail_rows(m - n) * vars.tension_vector;
  }

  arma::mat J_q = j0u + Ju.t() * arma::solve(Ja.t(), dWa - j0a) - dWu;
  return J_q.tail_cols(m - n);
}

} // namespace grabcdpr
