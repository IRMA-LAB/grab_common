#include "under_actuated_utils.h"

namespace grabcdpr {

UnderActuatedPlatformVars::UnderActuatedPlatformVars(
  const arma::uvec6& _mask /*=arma::uvec6(arma::fill::ones)*/)
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

grabnum::Matrix3d calcMatrixT(const CableVarsBase& cable)
{
  grabnum::Matrix3d T = cable.vers_w * cable.vers_w.Transpose() * sin(cable.tan_ang) /
                        (cable.vers_u.Transpose() * cable.pos_DA_glob)(1, 1);
  T += cable.vers_n * cable.vers_n.Transpose() / grabnum::Norm(cable.pos_BA_glob);
  return T;
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
    K_i.SetBlock<3, 3>(4, 1, T * a_tilde);
    K_i.SetBlock<3, 3>(1, 4, -T * a_tilde);
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

void updateIK0(const Vector3d& position, const Vector3d& orientation,
               const RobotParams& params, UnderActuatedRobotVars& vars)
{
  updatePlatformPose(position, orientation, params.platform, vars.platform);
  std::vector<id_t> active_actuators_id = params.activeActuatorsId();
  for (uint8_t i = 0; i < active_actuators_id.size(); ++i)
    updateCableZeroOrd(params.actuators[active_actuators_id[i]], vars.platform,
                       vars.cables[i]);
  vars.updateJacobians();
}

void updateCablesStaticTension(UnderActuatedRobotVars& vars)
{
  vars.tension_vector =
    calcCablesStaticTension(vars.geom_jacobian, vars.platform.ext_load);
}

} // namespace grabcdpr
