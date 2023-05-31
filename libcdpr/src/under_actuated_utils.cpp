/**
 * @file under_actuated_utils.cpp
 * @author Simone Comari
 * @date 21 Feb 2020
 * @brief This file includes definitions of structures and functions declared in
 * under_actuated_utils.h.
 */

#include "under_actuated_utils.h"

namespace grabcdpr {

UnderActuatedPlatformVars::UnderActuatedPlatformVars(
  const arma::uvec6& _mask /*=arma::uvec6(arma::fill::ones)*/)
{
  setMask(_mask);
}

UnderActuatedPlatformVars::UnderActuatedPlatformVars(
  const PlatformVars vars, const arma::uvec6& _mask /*=arma::uvec6(arma::fill::ones)*/)
  : PlatformVars(vars)
{
  setMask(_mask);
}

UnderActuatedPlatformVars::UnderActuatedPlatformVars(
  const RotParametrization _angles_type /*=TILT_TORSION*/,
  const arma::uvec6& _mask /*=arma::uvec6(arma::fill::ones)*/)
  : PlatformVars(_angles_type)
{
  setMask(_mask);
}

UnderActuatedPlatformVars::UnderActuatedPlatformVars(
  const grabnum::Vector3d& _position, const grabnum::Vector3d& _orientation,
  const RotParametrization _angles_type /*=TILT_TORSION*/,
  const arma::uvec6& _mask /*=arma::uvec6(arma::fill::ones)*/)
  : PlatformVars(_angles_type)
{
  setMask(_mask);
  updatePose(_position, _orientation);
}

UnderActuatedPlatformVars::UnderActuatedPlatformVars(
  const grabnum::Vector3d& _position, const grabnum::Vector3d& _velocity,
  const grabnum::Vector3d& _orientation, const grabnum::Vector3d& _orientation_dot,
  const RotParametrization _angles_type /*=TILT_TORSION*/,
  const arma::uvec6& _mask /*=arma::uvec6(arma::fill::ones)*/)
  : PlatformVars(_angles_type)
{
  setMask(_mask);
  updatePose(_position, _orientation);
  updateVel(_velocity, _orientation_dot);
}

UnderActuatedPlatformVars::UnderActuatedPlatformVars(
  const grabnum::Vector3d& _position, const grabnum::Vector3d& _velocity,
  const grabnum::Vector3d& _acceleration, const grabnum::Vector3d& _orientation,
  const grabnum::Vector3d& _orientation_dot, const grabnum::Vector3d& _orientation_ddot,
  const RotParametrization _angles_type /*=TILT_TORSION*/,
  const arma::uvec6& _mask /*=arma::uvec6(arma::fill::ones)*/)
  : PlatformVars(_angles_type)
{
  setMask(_mask);
  update(_position, _velocity, _acceleration, _orientation, _orientation_dot,
         _orientation_ddot);
}

void UnderActuatedPlatformVars::updatePose(const grabnum::VectorXd<POSE_DIM>& _pose)
{
  PlatformVars::updatePose(_pose);
  uint a = 0, u = 0;
  for (uint i = 1; i <= POSE_DIM; i++)
    if (mask(i - 1) == 1)
      actuated_vars(a++) = pose(i);
    else
      unactuated_vars(u++) = pose(i);
}

void UnderActuatedPlatformVars::updatePose(const grabnum::Vector3d& _position,
                                           const grabnum::Vector3d& _orientation)
{
  PlatformVars::updatePose(_position, _orientation);
  uint a = 0, u = 0;
  for (uint i = 1; i <= POSE_DIM; i++)
    if (mask(i - 1) == 1)
      actuated_vars(a++) = pose(i);
    else
      unactuated_vars(u++) = pose(i);
}

void UnderActuatedPlatformVars::updatePose(const arma::vec& _actuated_vars,
                                           const arma::vec& _unactuated_vars)
{
  assert(_actuated_vars.n_elem + _unactuated_vars.n_elem == POSE_DIM);

  actuated_vars   = _actuated_vars;
  unactuated_vars = _unactuated_vars;
  uint a = 0, u = 0;
  for (uint i = 1; i <= POSE_DIM; i++)
  {
    if (mask(i - 1) == 1)
      pose(i) = actuated_vars(a++);
    else
      pose(i) = unactuated_vars(u++);
  }
  PlatformVars::updatePose(pose);
}

void UnderActuatedPlatformVars::updateVel(const grabnum::Vector3d& _velocity,
                                          const grabnum::Vector3d& _orientation_dot,
                                          const grabnum::Vector3d& _orientation)
{
  PlatformVars::updateVel(_velocity, _orientation_dot, _orientation);
  uint a = 0, u = 0;
  for (uint i = 1; i <= POSE_DIM; i++)
    if (mask(i - 1) == 1)
    {
      actuated_vars(a)    = pose(i);
      actuated_deriv(a++) = velocity(i);
    }
    else
    {
      unactuated_vars(u)    = pose(i);
      unactuated_deriv(u++) = velocity(i);
    }
}

void UnderActuatedPlatformVars::updateVel(const grabnum::Vector3d& _velocity,
                                          const grabnum::Vector3d& _orientation_dot)
{
  PlatformVars::updateVel(_velocity, _orientation_dot);
  uint a = 0, u = 0;
  for (uint i = 1; i <= POSE_DIM; i++)
    if (mask(i - 1) == 1)
    {
      actuated_vars(a)    = pose(i);
      actuated_deriv(a++) = velocity(i);
    }
    else
    {
      unactuated_vars(u)    = pose(i);
      unactuated_deriv(u++) = velocity(i);
    }
}

void UnderActuatedPlatformVars::updateAcc(const grabnum::Vector3d& _acceleration,
                                          const grabnum::Vector3d& _orientation_ddot,
                                          const grabnum::Vector3d& _orientation_dot,
                                          const grabnum::Vector3d& _orientation,
                                          const grabnum::Matrix3d& _h_mat)
{
  PlatformVars::updateAcc(_acceleration, _orientation_ddot, _orientation_dot,
                          _orientation, _h_mat);
  uint a = 0, u = 0;
  for (uint i = 1; i <= POSE_DIM; i++)
    if (mask(i - 1) == 1)
    {
      actuated_vars(a)      = pose(i);
      actuated_deriv(a)     = velocity(i);
      actuated_deriv_2(a++) = acceleration(i);
    }
    else
    {
      unactuated_vars(u)      = pose(i);
      unactuated_deriv(u)     = velocity(i);
      unactuated_deriv_2(u++) = acceleration(i);
    }
}

void UnderActuatedPlatformVars::updateAcc(const grabnum::Vector3d& _acceleration,
                                          const grabnum::Vector3d& _orientation_ddot)
{
  PlatformVars::updateAcc(_acceleration, _orientation_ddot, orientation_dot, orientation,
                          h_mat);
  uint a = 0, u = 0;
  for (uint i = 1; i <= POSE_DIM; i++)
    if (mask(i - 1) == 1)
    {
      actuated_vars(a)      = pose(i);
      actuated_deriv(a)     = velocity(i);
      actuated_deriv_2(a++) = acceleration(i);
    }
    else
    {
      unactuated_vars(u)      = pose(i);
      unactuated_deriv(u)     = velocity(i);
      unactuated_deriv_2(u++) = acceleration(i);
    }
}

void UnderActuatedPlatformVars::update(const grabnum::Vector3d& _position,
                                       const grabnum::Vector3d& _velocity,
                                       const grabnum::Vector3d& _acceleration,
                                       const grabnum::Vector3d& _orientation,
                                       const grabnum::Vector3d& _orientation_dot,
                                       const grabnum::Vector3d& _orientation_ddot)
{
  PlatformVars::update(_position, _velocity, _acceleration, _orientation,
                       _orientation_dot, _orientation_ddot);
  uint a = 0, u = 0;
  for (uint i = 1; i <= POSE_DIM; i++)
    if (mask(i - 1) == 1)
    {
      actuated_vars(a)      = pose(i);
      actuated_deriv(a)     = velocity(i);
      actuated_deriv_2(a++) = acceleration(i);
    }
    else
    {
      unactuated_vars(u)      = pose(i);
      unactuated_deriv(u)     = velocity(i);
      unactuated_deriv_2(u++) = acceleration(i);
    }
}

void UnderActuatedPlatformVars::setMask(const arma::uvec6& _mask)
{
  mask                   = _mask;
  const arma::uvec a_idx = arma::find(mask == 1);
  // Reset and resize vectors
  actuated_vars.clear();
  actuated_deriv.clear();
  actuated_deriv_2.clear();
  actuated_vars.resize(a_idx.n_elem);
  actuated_deriv.resize(actuated_vars.size());
  actuated_deriv_2.resize(actuated_vars.size());
  unactuated_vars.clear();
  unactuated_deriv.clear();
  unactuated_deriv_2.clear();
  unactuated_vars.resize(POSE_DIM - a_idx.n_elem);
  unactuated_deriv.resize(unactuated_vars.size());
  unactuated_deriv_2.resize(unactuated_vars.size());
  // Distribute variables
  uint a = 0, u = 0;
  for (uint i = 1; i <= POSE_DIM; i++)
    if (mask(i - 1) == 1)
    {
      actuated_vars(a)      = pose(i);
      actuated_deriv(a)     = velocity(i);
      actuated_deriv_2(a++) = acceleration(i);
    }
    else
    {
      unactuated_vars(u)      = pose(i);
      unactuated_deriv(u)     = velocity(i);
      unactuated_deriv_2(u++) = acceleration(i);
    }
}


UnderActuatedRobotVars::UnderActuatedRobotVars(
  const size_t num_cables, const arma::uvec6& _mask /*=arma::uvec6(arma::fill::ones)*/)
  : platform(_mask), cables(std::vector<CableVars>(num_cables))
{
  resize();
}

UnderActuatedRobotVars::UnderActuatedRobotVars(
  const size_t num_cables, const RotParametrization _angles_type,
  const arma::uvec6& _mask /*=arma::uvec6(arma::fill::ones)*/)
  : platform(_angles_type, _mask), cables(std::vector<CableVars>(num_cables))
{
  resize();
}

void UnderActuatedRobotVars::setMask(const arma::uvec6& mask)
{
  platform.setMask(mask);
  resize();
}

void UnderActuatedRobotVars::resize()
{
  const size_t n         = cables.size();
  const size_t m         = POSE_DIM;
  const arma::uvec a_idx = arma::find(platform.mask == 0);
  const size_t a         = a_idx.n_elem;
  const size_t u         = m - a;
  // Geometric jacobians
  geom_jacobian.clear();
  geom_jacobian_d.clear();
  geom_jacobian_a.clear();
  geom_jacobian_u.clear();
  geom_orthogonal.clear();
  geom_jacobian.resize(n, m);
  geom_jacobian_d.resize(n, m);
  geom_jacobian_a.resize(n, a);
  geom_jacobian_u.resize(n, u);
  geom_orthogonal.resize(m, m - n);
  // Analitic jacobians
  anal_jacobian_d.clear();
  anal_jacobian_a.clear();
  anal_jacobian_u.clear();
  anal_orthogonal.clear();
  anal_jacobian.resize(n, m);
  anal_jacobian_d.resize(n, m);
  anal_jacobian_a.resize(n, a);
  anal_jacobian_u.resize(n, u);
  anal_orthogonal.resize(m, m - n);
  // Tension vector
  tension_vector.clear();
  tension_vector.resize(n);
}

void UnderActuatedRobotVars::updateGeometricJacobians()
{
  // Safety check
  if (geom_jacobian.n_rows != cables.size())
    resize();
  // Standard jacobian update
  for (uint8_t i = 0; i < cables.size(); ++i)
    geom_jacobian.row(i) = arma::rowvec6(cables[i].geom_jacob_row.Data());
  // Under-actuated-related update
  arma::uvec act_indeces   = find(platform.mask == 1);
  arma::uvec unact_indeces = find(platform.mask == 0);
  geom_jacobian_a          = geom_jacobian.cols(act_indeces);
  geom_jacobian_u          = geom_jacobian.cols(unact_indeces);
  const size_t n           = cables.size();
  const size_t m           = POSE_DIM;
  arma::mat temp(n, m - n, arma::fill::none);
  for (uint8_t i = 0; i < temp.n_cols; ++i)
    temp.col(i) = arma::solve(-geom_jacobian_a, geom_jacobian_u.col(i));
  geom_orthogonal.cols(act_indeces)   = temp;
  geom_orthogonal.cols(unact_indeces) = arma::eye(m - n, m - n);
}

void UnderActuatedRobotVars::updateAnaliticJacobians()
{
  // Safety check
  if (anal_jacobian.n_rows != cables.size())
    resize();
  // Standard jacobian update
  for (uint8_t i = 0; i < cables.size(); ++i)
    anal_jacobian.row(i) = arma::rowvec6(cables[i].anal_jacob_row.Data());
  // Under-actuated-related update
  arma::uvec act_indeces   = find(platform.mask == 1);
  arma::uvec unact_indeces = find(platform.mask == 0);
  anal_jacobian_a          = anal_jacobian.cols(act_indeces);
  anal_jacobian_u          = anal_jacobian.cols(unact_indeces);
  const size_t n           = cables.size();
  const size_t m           = POSE_DIM;
  arma::mat temp(n, m - n, arma::fill::none);
  for (uint8_t i = 0; i < temp.n_cols; ++i)
    temp.col(i) = arma::solve(-anal_jacobian_a, anal_jacobian_u.col(i));
  anal_orthogonal.cols(act_indeces)   = temp;
  anal_orthogonal.cols(unact_indeces) = arma::eye(m - n, m - n);
}

void UnderActuatedRobotVars::updateJacobians()
{
  // Safety check
  if (geom_jacobian.n_rows != cables.size() || anal_jacobian.n_rows != cables.size())
    resize();
  // Standard jacobian update
  for (uint8_t i = 0; i < cables.size(); ++i)
  {
    geom_jacobian.row(i) = arma::rowvec6(cables[i].geom_jacob_row.Data());
    anal_jacobian.row(i) = arma::rowvec6(cables[i].anal_jacob_row.Data());
  }
  // Under-actuated-related update
  arma::uvec act_indeces   = find(platform.mask == 1);
  arma::uvec unact_indeces = find(platform.mask == 0);
  geom_jacobian_a          = geom_jacobian.cols(act_indeces);
  geom_jacobian_u          = geom_jacobian.cols(unact_indeces);
  anal_jacobian_a          = anal_jacobian.cols(act_indeces);
  anal_jacobian_u          = anal_jacobian.cols(unact_indeces);
  const size_t n           = cables.size();
  const size_t m           = POSE_DIM;
  arma::mat geom_temp(n, m - n, arma::fill::none);
  arma::mat anal_temp(geom_temp);
  for (uint8_t i = 0; i < geom_temp.n_cols; ++i)
  {
    geom_temp.col(i) = arma::solve(-geom_jacobian_a, geom_jacobian_u.col(i));
    anal_temp.col(i) = arma::solve(-anal_jacobian_a, anal_jacobian_u.col(i));
  }
  const arma::mat temp_eye(m - n, m - n, arma::fill::eye);
  geom_orthogonal.rows(act_indeces)   = geom_temp;
  geom_orthogonal.rows(unact_indeces) = temp_eye;
  anal_orthogonal.rows(act_indeces)   = anal_temp;
  anal_orthogonal.rows(unact_indeces) = temp_eye;
}

void updateIK0(const Vector3d& position, const Vector3d& orientation,
               const RobotParams& params, UnderActuatedRobotVars& vars)
{
  updatePlatformPose(position, orientation, params.platform, vars.platform);
  for (uint8_t i = 0; i < vars.cables.size(); ++i)
    updateCableZeroOrd(params.actuators[i], vars.platform, vars.cables[i]);
  vars.updateJacobians();
}

void updateIK0(const Vector6d& pose, const RobotParams& params,
               UnderActuatedRobotVars& vars)
{
  updateIK0(pose.HeadRows<3>(), pose.TailRows<3>(), params, vars);
}

void updateIK0(const arma::vec6& _pose, const RobotParams& params,
               UnderActuatedRobotVars& vars)
{
  grabnum::Vector6d pose(_pose.begin(), _pose.end());
  updateIK0(pose, params, vars);
}

void updateCablesStaticTension(UnderActuatedRobotVars& vars)
{
  vars.tension_vector =
    calcCablesStaticTension(vars.geom_jacobian, vars.platform.ext_load);
}

grabnum::Matrix3d calcMatrixT(const CableVarsBase& cable)
{
  grabnum::Matrix3d T = cable.vers_w * cable.vers_w.Transpose() * sin(cable.tan_ang) /
                        (cable.vers_u.Transpose() * cable.pos_DA_glob)(1, 1);
  T += cable.vers_n * cable.vers_n.Transpose() / grabnum::Norm(cable.pos_BA_glob);
  return T;
}

arma::vec calcStaticConstraint(const UnderActuatedRobotVars& vars)
{
  return vars.geom_orthogonal.t() * toArmaVec(vars.platform.ext_load);
}

arma::mat calcJacobianL(const UnderActuatedRobotVars& vars) { return vars.anal_jacobian; }

arma::mat calcJacobianSw(const UnderActuatedRobotVars& vars)
{
  arma::mat jacobian_sw(arma::size(vars.anal_jacobian), arma::fill::none);
  for (uint i = 0; i < jacobian_sw.n_rows; ++i)
  {
    arma::rowvec temp =
      arma::join_horiz(toArmaVec(vars.cables[i].vers_w).t(),
                       toArmaVec(-vars.cables[i].vers_w.Transpose() *
                                 Skew(vars.cables[i].pos_PA_glob) * vars.platform.h_mat));
    jacobian_sw.row(i) =
      temp / grabnum::Dot(vars.cables[i].vers_u, vars.cables[i].pos_DA_glob);
  }
  return jacobian_sw;
}

arma::mat calcJacobianGS(const UnderActuatedRobotVars& vars)
{
  grabnum::Matrix6d K;
  for (uint8_t i = 0; i < vars.cables.size(); ++i)
  {
    grabnum::Matrix3d T       = calcMatrixT(vars.cables[i]);
    grabnum::Matrix3d a_tilde = grabnum::Skew(vars.cables[i].pos_PA_glob);
    grabnum::Matrix6d K_i;
    K_i.SetBlock<3, 3>(1, 1, -T);
    K_i.SetBlock<3, 3>(1, 4, T * a_tilde);
    K_i.SetBlock<3, 3>(4, 1, -a_tilde * T);
    K_i.SetBlock<3, 3>(4, 4,
                       (a_tilde * T - grabnum::Skew(vars.cables[i].vers_t)) * a_tilde);
    K += (vars.tension_vector(i) * K_i);
  }
  grabnum::Matrix3d K_blk(K.GetBlock<3, 3>(4, 4));
  K.SetBlock<3, 3>(4, 4,
                   K_blk + grabnum::Skew(vars.platform.ext_load.HeadRows<3>()) *
                             grabnum::Skew(vars.platform.pos_PG_glob));
  arma::mat jacobian_gs    = vars.geom_orthogonal.t() * toArmaMat(K);
  jacobian_gs.tail_cols(3) = jacobian_gs.tail_cols(3) * toArmaMat(vars.platform.h_mat);
  return jacobian_gs;
}

void optFunGS(const RobotParams& params, const arma::vec& act_vars,
              const arma::vec& unact_vars, arma::mat& fun_jacobian, arma::vec& fun_val)
{
  const ulong kNumCables = params.actuators.size();
  UnderActuatedRobotVars vars(kNumCables, params.platform.rot_parametrization,
                              params.controlled_vars_mask);
  vars.platform.updatePose(act_vars, unact_vars);
  updateIK0(vars.platform.position, vars.platform.orientation, params, vars);
  updateExternalLoads(params.platform, vars.platform);
  updateCablesStaticTension(vars);

  fun_val      = calcStaticConstraint(vars);
  fun_jacobian = calcJacobianGS(vars).cols(arma::find(params.controlled_vars_mask == 0));
}

arma::mat ComputeJacobian_GSonlypos(
  const UnderActuatedRobotVars& vars, const Vector3d tau_r, const ulong kNumCables,
  const Matrix<double, 4, 3> Xi_T, const Matrix<double, 4, 3> Xi_r,
  const Matrix<double, 4, 1> Xi_T_ort, const Vector3d f_T, const double lambda,
  const uint i_a, const double a, const uint i_b, const double b)
{

  Matrix<double, 3, 6> K3;
  Matrix<double, 6, 6> K;
  Matrix<double, 4, 6> K2;
  Matrix<double, 6, 6> K_mod;
  K3.SetZero();
  K.SetZero();
  K2.SetZero();
  K_mod.SetZero();
  Vector4d temp;
  for (uint i = 0; i < kNumCables; i++)
  {
    temp(i + 1) = Xi_T_ort(i) * vars.cables[i].length;
  }
  Vector4d tau_0 = fromArmaVec4(vars.tension_vector) - temp;
  for (uint i = 0; i < kNumCables; i++)
  {
    Matrix3d T               = calcMatrixT(vars.cables[i]);
    Matrix3d a_tilde         = Skew(vars.cables[i].pos_PA_glob);
    Matrix<double, 3, 6> dJ2 = HorzCat(-1. * T, T * a_tilde);
    Matrix<double, 3, 6> temp2 =
      HorzCat(-1. * a_tilde * T, (a_tilde * T - Skew(vars.cables[i].vers_t)) * a_tilde);
    Matrix<double, 6, 6> dJ   = VertCat(dJ2, temp2);

    K3 = K3 - Xi_T_ort(i + 1) * dJ2;
    K2.SetBlock<1, 6>(i + 1, 1, (dJ2.Transpose() * tau_r).Transpose());
    K     = K + vars.tension_vector(i, 0) * dJ;
    K_mod = K_mod + tau_0(i + 1) * dJ;
  }
  K.SetBlock<3, 3>(4, 4,
                   K.GetBlock<3, 3>(4, 4) + Skew(f_T) * Skew(vars.platform.pos_PG_glob));


  Matrix<double, 1, 6> temp4;
  temp4.SetZero();
  Matrix<double, 4, 6> dXi_T_ort = VertCat(
   Linsolve3x3_3x6((Xi_T.GetBlock<3, 3>(1, 1)).Transpose(), K3), temp4);
  Matrix<double, 4, 4> eye4;
  eye4.SetIdentity();
  Matrix<double, 4, 6> dtau0 =
    (eye4 -
     Xi_T * (Linsolve3x3_3x4(Xi_T.Transpose() * Xi_T, Xi_T.Transpose()))) *
      K2 -
    Xi_T *
      Linsolve3x3_3x6(Xi_T.Transpose() * Xi_T, K_mod.GetBlock<3, 6>(1, 1));

  Vector6d da;
  Vector6d db;
  for (uint i = 0; i < 6; i++)
  {
    da(i + 1) = (dtau0(i_a, i + 1) * Xi_T_ort(i_a) - a * dXi_T_ort(i_a, i + 1)) /
                pow(Xi_T_ort(i_a), 2);
    db(i + 1) = (dtau0(i_b, i + 1) * Xi_T_ort(i_b) - b * dXi_T_ort(i_b, i + 1)) /
                pow(Xi_T_ort(i_b), 2);
  }
  Matrix<double, 4, 6> dtau =
    dtau0 + dXi_T_ort * lambda - 0.5 * grabnum::ExtProduct(Xi_T_ort, (da + db));
  Matrix<double, 3, 6> Jac = -1. * (K.GetBlock<3, 6>(1, 1) + Xi_r.Transpose() * dtau);

  return toArmaMat_3x6(Jac);
}

void optFunGS_onlypos(const RobotParams& params, const arma::vec& act_vars,
                      const arma::vec& unact_vars, arma::mat& fun_jacobian,
                      arma::vec& fun_val)
{
  const ulong kNumCables = params.actuators.size();
  UnderActuatedRobotVars vars(kNumCables, params.platform.rot_parametrization,
                              params.controlled_vars_mask);
  vars.platform.updatePose(act_vars, unact_vars);
  updateIK0(vars.platform.position, vars.platform.orientation, params, vars);
  updateExternalLoads(params.platform, vars.platform);
  updateCablesStaticTension(vars);

  Matrix<double, 6, 4> geomjac_grab = fromArmaMat6x4(vars.geom_jacobian);
  Matrix<double, 3, 4> Xi_T         = geomjac_grab.GetBlock<3, 4>(1, 1);
  Matrix<double, 3, 4> Xi_r         = geomjac_grab.GetBlock<3, 4>(4, 1);
  Matrix<double, 4, 1> Xi_T_ort;
  Xi_T_ort(4, 1) = 1;
  Xi_T_ort.SetBlock<3, 1>(1, 1,
                          Linsolve3x3_3x1(geomjac_grab.GetBlock<3, 3>(1, 1),
                                                        -1. * Xi_T.GetBlock<3, 1>(1, 4)));
  // Xi_T =  cdpr_v.geometric_jacobian(1:3,:);
  // f_T = cdpr_v.platform.ext_load(1:3);
  // Xi_R = cdpr_v.geometric_jacobian(4:6,:);
  // f_R = cdpr_v.platform.ext_load(4:6);
  Vector3d f_T = vars.platform.ext_load.GetBlock<3, 1>(1, 1);
  Vector3d f_r = vars.platform.ext_load.GetBlock<3, 1>(4, 1);

  Vector4d tau_0 =
    Xi_T.Transpose() * Linsolve3x3_3x1(Xi_T * Xi_T.Transpose(), f_T);

  Vector4d tau_min(40);
  Vector4d tau_max(400);

  Vector4d a = -tau_max + tau_0;
  Vector4d b = -tau_min + tau_0;
  for (uint i = 0; i < 4; i++)
  {
    if (Xi_T_ort(i + 1) >= 0)
    {
      a(i + 1) = a(i + 1) / Xi_T_ort(i + 1);
      b(i + 1) = b(i + 1) / Xi_T_ort(i + 1);
    }
    else
    {
      double temp = a(i + 1);
      a(i + 1)    = b(i) / Xi_T_ort(i + 1);
      b(i + 1)    = temp / Xi_T_ort(i + 1);
    }
  }
  uint ind_b              = b.MinIdx();
  uint ind_a              = a.MaxIdx();
  double lambda           = -(b.Min() + a.Max()) / 2;
  Vector4d tension_vector = tau_0 + lambda * Xi_T_ort;

  fun_val = toArmaVec(Xi_r * tension_vector - f_r);
  // linsolve(Xi_T*Xi_T',f_T)
  Vector3d tau_r = Linsolve3x3_3x1(Xi_T * Xi_T.Transpose(), f_T);
  ;

  fun_jacobian =
    ComputeJacobian_GSonlypos(vars, tau_r, kNumCables, Xi_T.Transpose(), Xi_r.Transpose(),
                              Xi_T_ort, f_T, lambda, ind_a, a.Max(), ind_b, b.Max());
}

void optFunDK0GS(const RobotParams& params, const arma::vec& cables_length,
                 const arma::vec& swivel_angles, const arma::vec6& pose,
                 arma::mat& fun_jacobian, arma::vec& fun_val)
{
  const ulong kNumCables = params.actuators.size();
  UnderActuatedRobotVars vars(kNumCables, params.platform.rot_parametrization,
                              params.controlled_vars_mask);
  updateIK0(pose, params, vars);
  updateExternalLoads(params.platform, vars.platform);
  updateCablesStaticTension(vars);

  arma::vec l_constraints(kNumCables, arma::fill::none);
  arma::vec sw_constraints(kNumCables, arma::fill::none);
  for (uint i = 0; i < kNumCables; ++i)
  {
    l_constraints(i)  = vars.cables[i].length - cables_length[i];
    sw_constraints(i) = vars.cables[i].swivel_ang - swivel_angles[i];
  }
  arma::vec gs_contraints = calcStaticConstraint(vars);

  arma::mat l_jacobian  = calcJacobianL(vars);
  arma::mat sw_jacobian = calcJacobianSw(vars);
  arma::mat gs_jacobian = calcJacobianGS(vars);

  fun_val      = arma::join_vert(l_constraints, sw_constraints, gs_contraints);
  fun_jacobian = arma::join_vert(l_jacobian, sw_jacobian, gs_jacobian);
}

bool solveDK0GS(const std::vector<double>& cables_length,
                const std::vector<double>& swivel_angles,
                const grabnum::VectorXd<POSE_DIM>& init_guess_pose,
                const RobotParams& params, VectorXd<POSE_DIM>& platform_pose,
                const uint8_t nmax /*= 100*/, uint8_t* iter_out /*= nullptr*/)
{
  static const double kFtol = 1e-6;
  static const double kXtol = 1e-6;

  // First round to init function value and jacobian
  arma::vec func_val;
  arma::mat func_jacob;
  arma::vec6 pose = toArmaVec(init_guess_pose);
  optFunDK0GS(params, cables_length, swivel_angles, pose, func_jacob, func_val);

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
    optFunDK0GS(params, cables_length, swivel_angles, pose, func_jacob, func_val);
    err  = arma::norm(s);
    cond = kXtol * (1 + arma::norm(pose));
  }

  if (iter_out != nullptr)
    *iter_out = iter;

  platform_pose.Fill(pose.begin(), pose.end());

  return true;
}

bool updateDK0(const RobotParams& params, UnderActuatedRobotVars& vars)
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
  grabnum::VectorXd<POSE_DIM> new_pose;
  if (solveDK0GS(cables_length, swivel_angles, init_guess_pose, params, new_pose))
  {
    // Update inverse kinematics
    updateIK0(new_pose.GetBlock<3, 1>(1, 1), new_pose.GetBlock<3, 1>(4, 1), params, vars);
    return true;
  }
  // Could not solve optimization (failed direct kinematics)
  return false;
}

arma::vec nonLinsolveJacGeomStatic(const grabnum::VectorXd<POSE_DIM>& init_guess,
                                   const arma::uvec6& mask, const RobotParams& params,
                                   const uint8_t nmax /*= 100*/,
                                   uint8_t* iter_out /*= nullptr*/)
{
  static const double kFtol = 1e-4;
  static const double kXtol = 1e-3;

  // Distribute initial guess between fixed and variable coordinates (i.e. the solution of
  // the iterative process)
  arma::vec init_guess_arma = toArmaVec(init_guess);
  arma::vec fixed_coord(init_guess_arma.elem(arma::find(mask == 1)));
  arma::vec var_coord(init_guess_arma.elem(arma::find(mask == 0)));

  // First round to init function value and jacobian
  arma::vec func_val;
  arma::mat func_jacob;
  grabcdpr::optFunGS(params, fixed_coord, var_coord, func_jacob, func_val);

  // Init iteration variables
  arma::vec s;
  uint8_t iter = 0;
  double err   = 1.0;
  double cond  = 0.0;
  // Start iterative process
  while (iter < nmax && arma::norm(func_val) > kFtol && err > cond)
  {
    iter++;
    s = arma::solve(func_jacob, func_val);
    var_coord -= s;
    grabcdpr::optFunGS(params, fixed_coord, var_coord, func_jacob, func_val);
    err  = arma::norm(s);
    cond = kXtol * (1 + arma::norm(var_coord));
  }

  if (iter_out != nullptr)
    *iter_out = iter;

  return var_coord;
}
arma::vec nonLinsolveJacGeomStatic_onlypos(const grabnum::VectorXd<POSE_DIM>& init_guess,
                                           const arma::uvec6& mask,
                                           const RobotParams& params,
                                           const uint8_t nmax /*= 100*/,
                                           uint8_t* iter_out /*= nullptr*/)
{
  static const double kFtol = 1e-4;
  static const double kXtol = 1e-3;

  // Distribute initial guess between fixed and variable coordinates (i.e. the solution of
  // the iterative process)
  arma::vec init_guess_arma = toArmaVec(init_guess);
  arma::vec fixed_coord(init_guess_arma.elem(arma::find(mask == 1)));
  arma::vec var_coord(init_guess_arma.elem(arma::find(mask == 0)));

  // First round to init function value and jacobian
  arma::vec func_val;
  arma::mat func_jacob;
  grabcdpr::optFunGS_onlypos(params, fixed_coord, var_coord, func_jacob, func_val);

  // Init iteration variables
  arma::vec s;
  uint8_t iter = 0;
  double err   = 1.0;
  double cond  = 0.0;
  // Start iterative process
  while (iter < nmax && arma::norm(func_val) > kFtol && err > cond)
  {
    iter++;
    s = arma::solve(func_jacob, func_val);
    var_coord -= s;
    grabcdpr::optFunGS(params, fixed_coord, var_coord, func_jacob, func_val);
    err  = arma::norm(s);
    cond = kXtol * (1 + arma::norm(var_coord));
  }

  if (iter_out != nullptr)
    *iter_out = iter;

  return var_coord;
}
Vector3d Linsolve3x3_3x1(const Matrix3d& _mat, const Vector3d& _vect)
{

  Matrix3d mat(_mat);
  Vector3d vect(_vect);
  uint8_t dim = 3;
  for (uint8_t i = 1; i < dim; ++i)
  {
    uint8_t max_idx = i;
    double max_val  = mat(i, i);
    for (uint8_t j = i + 1; j <= dim; ++j)
    {
      if (fabs(mat(j, i)) > fabs(max_val))
      {
        max_val = mat(j, i);
        max_idx = j;
      }
    }
    mat.SwapRow(i, max_idx);
    vect.SwapRow(i, max_idx);
    for (uint8_t j = i + 1; j <= dim; ++j)
    {
      double m = mat(j, i) / mat(i, i);
      vect(j) -= vect(i) * m;
      for (uint8_t k = i; k <= dim; ++k)
      {
        mat(j, k) -= mat(i, k) * m;
      }
    }
  }
  return LinsolveUp3x3_3x1(mat, vect);
}


Vector3d LinsolveUp3x3_3x1(const Matrix3d& mat, const Vector3d& vect)
{

  Vector3d result;
  result = vect;
  result(3) /= mat(3, 3);
  for (uint8_t j = 3 - 1; j > 0; j--)
  {
    for (uint8_t k = j + 1; k <= 3; ++k)
    {
      result(j) -= mat(j, k) * result(k);
    }
    result(j) /= mat(j, j);
  }

  return result;
}

grabnum::Matrix<double, 3, 4> Linsolve3x3_3x4(const Matrix<double, 3, 3>& mat,
                                              const grabnum::Matrix<double, 3, 4>& mat2)
{

  grabnum::Matrix<double, 3, 4> result;
  grabnum::Vector3d column;
  for(uint i=0;i<4;i++){
    column = Linsolve3x3_3x1(mat,mat2.GetBlock<3,1>(1,i+1));
    result.SetBlock(1,i+1,column);
  }

  return result;
}
grabnum::Matrix<double, 3, 6> Linsolve3x3_3x6(const Matrix<double, 3, 3>& mat,
                                              const grabnum::Matrix<double, 3, 6>& mat2)
{

  grabnum::Matrix<double, 3, 6> result;
  grabnum::Vector3d column;
  for(uint i=0;i<6;i++){
    column = Linsolve3x3_3x1(mat,mat2.GetBlock<3,1>(1,i+1));
    result.SetBlock(1,i+1,column);
  }

  return result;
}
grabnum::Matrix<double, 3, 3> Linsolve3x3_3x3(const Matrix<double, 3, 3>& mat,
                                              const grabnum::Matrix<double, 3, 3>& mat2)
{

  grabnum::Matrix<double, 3, 3> result;
  grabnum::Vector3d column;
  for(uint i=0;i<3;i++){
    column = Linsolve3x3_3x1(mat,mat2.GetBlock<3,1>(1,i+1));
    result.SetBlock(1,i+1,column);
  }

  return result;
}
arma::mat toArmaMat_3x6(Matrix<double,3,6> mat, bool copy /*= true*/)
{
  // Data is filled column-by-column, that's why we need the transpose
  return arma::mat(mat.Data(), 6, 3, copy).t();
}
} // namespace grabcdpr
