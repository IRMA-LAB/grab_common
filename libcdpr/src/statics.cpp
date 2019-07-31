#include "statics.h"

cv::Mat toCvMat(Vector3d vect) { return cv::Mat(3, 1, CV_64F, vect.Data()); }
cv::Mat toCvMat(Matrix3d vect) { return cv::Mat(3, 3, CV_64F, vect.Data()); }

namespace grabcdpr {

void calcGeometricStatic(const RobotParams& params,
                         const cv::Mat& fixed_coord,
                         const cv::Mat& var_coord,
                         const VectorXi<POSE_DIM>& mask, cv::Mat& mat, cv::Mat vector)
{
  assert(fixed_coord.rows + var_coord.rows == POSE_DIM);

  int params_idx = 0;
  int vars_idx   = 0;
  VectorXd<POSE_DIM> pose;
  for (uint i = 0; i < pose.Size(); i++)
    if (mask(i) == 1)
      pose(i) = fixed_coord.at<double>(params_idx++);
    else
      pose(i) = var_coord.at<double>(vars_idx++);

  RobotVars vars;
  UpdateIK0(pose.GetBlock<3, 1>(1, 1), pose.GetBlock<3, 1>(4, 1), params, vars);
  UpdateExternalLoads(grabnum::Matrix3d(0.0), params.platform, vars.platform);

  int num_cables = static_cast<int>(params.actuators.size());
  cv::Mat Ja(cv::Size(num_cables, num_cables), CV_64F, cv::Scalar(0));
  cv::Mat Ju(cv::Size(num_cables, POSE_DIM - num_cables), CV_64F, cv::Scalar(0));
  cv::Mat Wa(cv::Size(num_cables, 1), CV_64F, cv::Scalar(0));
  cv::Mat Wu(cv::Size(POSE_DIM - num_cables, 1), CV_64F, cv::Scalar(0));

  int act_idx   = 0;
  int unact_idx = 0;
  for (uint i = 0; i < POSE_DIM; i++)
    if (mask(i + 1) == 1)
    {
      vars.geom_jabobian.col(i).copyTo(Ja.col(act_idx));
      Wa.at<double>(act_idx++) =
        vars.platform.ext_load(i + 1); // index of grabnum vect starts at 1
    }
    else
    {
      vars.geom_jabobian.col(i).copyTo(Ju.col(unact_idx));
      Wu.at<double>(unact_idx++) =
        vars.platform.ext_load(i + 1); // index of grabnum vect starts at 1
    }

  cv::Mat tension_vector;
  cv::solve(Ja.t(), Wa, tension_vector); // linsolve Ax = b
  vector = Ju.t() * tension_vector - Wu;
  mat = CalcGsJacobians(vars, Ja, Ju, params.platform.mass * params.platform.gravity_acc);
}

cv::Mat CalcGsJacobians(const RobotVars& vars, const cv::Mat& Ja, const cv::Mat& Ju,
                        const Vector3d& mg)
{
  // TODO: check implementation of these (missing parts)
  int n = Ja.rows;
  int m = Ja.cols + Ju.cols;
  cv::Mat J_sl(cv::Size(2 * n, m), CV_64F, cv::Scalar(0));
  cv::Mat dsdq(cv::Size(n, m), CV_64F, cv::Scalar(0));
  cv::Mat dphidq(cv::Size(n, m), CV_64F, cv::Scalar(0));
  cv::Mat dldq(cv::Size(n, m), CV_64F, cv::Scalar(0));
  cv::Mat drhodq(cv::Size(n, m), CV_64F, cv::Scalar(0));
  for (ulong i = 0; i < static_cast<ulong>(n); i++)
  {
    grabnum::MatrixXd<3, 6> dadq(1.0); // set diagonal to 1
    dadq.SetBlock<3, 3>(1, 4, -Skew(vars.cables[i].pos_PA_glob) * vars.platform.h_mat);
    cv::Mat(1, m, CV_64F,
            (vars.cables[i].vers_w.Transpose() * dadq /
             grabnum::Dot(vars.cables[i].pos_DA_glob, vars.cables[i].vers_u))
              .Data())
      .copyTo(dsdq.row(static_cast<int>(i)));
    cv::Mat(1, m, CV_64F,
            (vars.cables[i].vers_n.Transpose() * dadq / Norm(vars.cables[i].pos_BA_glob))
              .Data())
      .copyTo(dphidq.row(static_cast<int>(i)));
    cv::Mat(1, m, CV_64F, (vars.cables[i].vers_t.Transpose() * dadq).Data())
      .copyTo(dldq.row(static_cast<int>(i)));
    cv::Mat drhodq =
      toCvMat(sin(vars.cables[i].tan_ang) * vars.cables[i].vers_w) * dsdq.row(i) +
      toCvMat(vars.cables[i].vers_n) * dphidq.row(i);
    MatrixXd<3, 6> dadq1(0.0); // set diagonal to 0
    dadq.SetBlock<3, 3>(1, 4, -Skew(vars.cables[i].pos_PA_glob) * vars.platform.h_mat);
  }

  J_sl.rowRange(0, n)     = dsdq;
  J_sl.rowRange(n, 2 * n) = dldq;
  cv::Mat dfdq(cv::Size(6, 6), CV_64F, cv::Scalar(0));
  dfdq(cv::Range(3, 6), cv::Range(3, 6)) =
    toCvMat(Skew(mg) * Skew(vars.platform.pos_PG_glob) * vars.platform.h_mat);
  cv::Mat dWa = dfdq.rowRange(0, n);
  cv::Mat dWu = dfdq.rowRange(n, 6);

  cv::Mat j0u(cv::Size(m - n, m), CV_64F, cv::Scalar(0));
  cv::Mat j0a(cv::Size(n, m), CV_64F, cv::Scalar(0));
  for (int i = 0; i < m; i++) {}

  cv::Mat sol;
  cv::solve(Ja.t(), dWa - j0a, sol);
  cv::Mat J_q = j0u + Ju.t() * sol - dWu;
  return J_q;
}

} // namespace grabcdpr
