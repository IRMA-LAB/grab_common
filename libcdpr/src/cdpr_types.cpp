/**
 * @file cdpr_types.cpp
 * @author Simone Comari
 * @date 02 Mar 2020
 * @brief File containing definitions of struct functions declared in cdpr_types.h.
 */

#include "cdpr_types.h"

arma::rowvec3 toArmaVec(RowVectorXd<3> vect, bool copy /*= true*/)
{
  return arma::rowvec(vect.Data(), 3, copy);
}

arma::vec toArmaVec(Vector3d vect, bool copy /*= true*/)
{
  return arma::vec(vect.Data(), 3, copy);
}

arma::vec toArmaVec(VectorXd<POSE_DIM> vect, bool copy /*= true*/)
{
  return arma::vec(vect.Data(), POSE_DIM, copy);
}

arma::vec toArmaVec(VectorXd<POSE_QUAT_DIM> vect, bool copy /*= true*/)
{
  return arma::vec(vect.Data(), POSE_QUAT_DIM, copy);
}

arma::mat toArmaMat(Matrix3d mat, bool copy /*= true*/)
{
  // Data is filled column-by-column, that's why we need the transpose
  return arma::mat(mat.Data(), 3, 3, copy).t();
}

arma::mat toArmaMat(Matrix6d mat, bool copy /*= true*/)
{
  // Data is filled column-by-column, that's why we need the transpose
  return arma::mat(mat.Data(), 6, 6, copy).t();
}

grabnum::Vector3d fromArmaVec3(const arma::vec3& vect)
{
  return grabnum::Vector3d(vect.begin(), vect.end());
}

grabnum::Vector4d fromArmaVec4(const arma::vec4& vect)
{
  return grabnum::Vector4d(vect.begin(), vect.end());
}


namespace grabcdpr {

//------ Parameters Structs ----------------------------------------------------------//

std::vector<id_t> RobotParams::activeActuatorsId() const
{
  std::vector<id_t> active_actuators_id;
  for (uint i = 0; i < actuators.size(); i++)
    if (actuators[i].active)
      active_actuators_id.push_back(i);
  return active_actuators_id;
}

size_t RobotParams::activeActuatorsNum() const
{
  size_t active_actuators_counter = 0;
  for (uint i = 0; i < actuators.size(); i++)
    if (actuators[i].active)
      active_actuators_counter++;
  return active_actuators_counter;
}

RobotParams RobotParams::getOnlyActiveComponents() const
{
  RobotParams params_active;
  params_active.platform = platform;
  for (const auto& actuator_params : actuators)
    if (actuator_params.active)
      params_active.actuators.push_back(actuator_params);
  return params_active;
}

void RobotParams::removeInactiveComponents()
{
  for (long i = actuators.size() - 1; i >= 0; i--)
    if (!actuators[i].active)
      actuators.erase(actuators.begin() + i);
}

//------ Variables Structs -----------------------------------------------------------//

PlatformVars::PlatformVars(const RotParametrization _angles_type /*=TILT_TORSION*/)
  : angles_type(_angles_type)
{}

PlatformVars::PlatformVars(const grabnum::Vector3d& _position,
                           const grabnum::Vector3d& _orientation,
                           const RotParametrization _angles_type /*=TILT_TORSION*/)
  : angles_type(_angles_type)
{
  updatePose(_position, _orientation);
}

PlatformVars::PlatformVars(const grabnum::Vector3d& _position,
                           const grabnum::Vector3d& _velocity,
                           const grabnum::Vector3d& _orientation,
                           const grabnum::Vector3d& _orientation_dot,
                           const RotParametrization _angles_type /*=TILT_TORSION*/)
  : angles_type(_angles_type)
{
  updatePose(_position, _orientation);
  updateVel(_velocity, _orientation_dot);
}

PlatformVars::PlatformVars(const grabnum::Vector3d& _position,
                           const grabnum::Vector3d& _velocity,
                           const grabnum::Vector3d& _acceleration,
                           const grabnum::Vector3d& _orientation,
                           const grabnum::Vector3d& _orientation_dot,
                           const grabnum::Vector3d& _orientation_ddot,
                           const RotParametrization _angles_type /*=TILT_TORSION*/)
  : angles_type(_angles_type)
{
  update(_position, _velocity, _acceleration, _orientation, _orientation_dot,
         _orientation_ddot);
}

void PlatformVars::updatePose(const grabnum::VectorXd<POSE_DIM>& _pose)
{
  updatePose(_pose.GetBlock<3, 1>(1, 1), _pose.GetBlock<3, 1>(4, 1));
}

void PlatformVars::updatePose(const grabnum::Vector3d& _position,
                              const grabnum::Vector3d& _orientation)
{
  position    = _position;
  orientation = _orientation;
  for (uint8_t i = 1; i <= 3; ++i)
  {
    pose(i)     = position(i);
    pose(3 + i) = orientation(i);
  }
  switch (angles_type)
  {
    case EULER_ZYZ:
      rot_mat = grabgeom::EulerZYZ2Rot(orientation);
      h_mat   = grabgeom::HtfZYZ(_orientation);
      break;
    case TAIT_BRYAN:
      rot_mat = grabgeom::EulerXYZ2Rot(orientation);
      h_mat   = grabgeom::HtfXYZ(_orientation);
      break;
    case RPY:
      rot_mat = grabgeom::RPY2Rot(orientation);
      h_mat   = grabgeom::HtfRPY(_orientation);
      break;
    case TILT_TORSION:
      rot_mat = grabgeom::TiltTorsion2Rot(orientation);
      h_mat   = grabgeom::HtfTiltTorsion(_orientation);
      break;
    default:
      // This should never happen
      break;
  }
}

void PlatformVars::updateVel(const grabnum::Vector3d& _velocity,
                             const grabnum::Vector3d& _orientation_dot,
                             const grabnum::Vector3d& _orientation)
{
  updatePose(position, _orientation); // update h_mat
  updateVel(_velocity, _orientation_dot);
}

void PlatformVars::updateVel(const grabnum::Vector3d& _velocity,
                             const grabnum::Vector3d& _orientation_dot)
{
  linear_vel      = _velocity;
  orientation_dot = _orientation_dot;
  angular_vel     = h_mat * orientation_dot;
  velocity.SetBlock<3, 1>(1, 1, linear_vel);
  velocity.SetBlock<3, 1>(4, 1, angular_vel);
}

void PlatformVars::updateAcc(const grabnum::Vector3d& _acceleration,
                             const grabnum::Vector3d& _orientation_ddot,
                             const grabnum::Vector3d& _orientation_dot,
                             const grabnum::Vector3d& _orientation,
                             const grabnum::Matrix3d& _h_mat)
{
  linear_acc       = _acceleration;
  orientation_ddot = _orientation_ddot;
  switch (angles_type)
  {
    case TAIT_BRYAN:
      dh_mat = grabgeom::DHtfXYZ(_orientation, _orientation_dot);
      break;
    case TILT_TORSION:
      dh_mat = grabgeom::DHtfTiltTorsion(_orientation, _orientation_dot);
      break;
    case RPY:
      dh_mat = grabgeom::DHtfRPY(_orientation, _orientation_dot);
      break;
    case EULER_ZYZ:
      dh_mat = grabgeom::DHtfZYZ(_orientation, _orientation_dot);
      break;
    default:
      // This should never happen
      break;
  }
  angular_acc = dh_mat * _orientation_dot + _h_mat * orientation_ddot;
  acceleration.SetBlock<3, 1>(1, 1, linear_acc);
  acceleration.SetBlock<3, 1>(4, 1, angular_acc);
}

void PlatformVars::updateAcc(const grabnum::Vector3d& _acceleration,
                             const grabnum::Vector3d& _orientation_ddot)
{
  updateAcc(_acceleration, _orientation_ddot, orientation_dot, orientation, h_mat);
}

void PlatformVars::update(const grabnum::Vector3d& _position,
                          const grabnum::Vector3d& _velocity,
                          const grabnum::Vector3d& _acceleration,
                          const grabnum::Vector3d& _orientation,
                          const grabnum::Vector3d& _orientation_dot,
                          const grabnum::Vector3d& _orientation_ddot)
{
  updatePose(_position, _orientation);
  updateVel(_velocity, _orientation_dot);
  updateAcc(_acceleration, _orientation_ddot);
}

PlatformVarsQuat::PlatformVarsQuat(const grabnum::Vector3d& _position,
                                   const grabnum::Vector3d& _velocity,
                                   const grabnum::Vector3d& _acceleration,
                                   const grabgeom::Quaternion& _orientation,
                                   const grabgeom::Quaternion& _orientation_dot,
                                   const grabgeom::Quaternion& _orientation_ddot)
{
  update(_position, _velocity, _acceleration, _orientation, _orientation_dot,
         _orientation_ddot);
}

void PlatformVarsQuat::updatePose(const grabnum::VectorXd<POSE_QUAT_DIM> pose)
{
  grabgeom::Quaternion quaternion(pose.GetBlock<4, 1>(4, 1));
  updatePose(pose.GetBlock<3, 1>(1, 1), quaternion);
}

void PlatformVarsQuat::updatePose(const grabnum::Vector3d& _position,
                                  const grabgeom::Quaternion& _orientation)
{
  position    = _position;
  orientation = _orientation.q();
  for (uint8_t i = 1; i <= 3; ++i)
  {
    pose(i)     = position(i);
    pose(2 * i) = orientation(i);
  }
  pose(7) = orientation(4);
  rot_mat = grabgeom::Quat2Rot(orientation);
}

void PlatformVarsQuat::updateVel(const grabnum::Vector3d& _velocity,
                                 const grabgeom::Quaternion& _orientation_dot,
                                 const grabgeom::Quaternion& _orientation)
{
  linear_vel      = _velocity;
  orientation_dot = _orientation_dot.q();
  h_mat           = grabgeom::HtfQuat(_orientation);
  angular_vel     = h_mat * orientation_dot;
}

void PlatformVarsQuat::updateVel(const grabnum::Vector3d& _velocity,
                                 const grabgeom::Quaternion& _orientation_dot)
{
  updateVel(_velocity, _orientation_dot, grabgeom::Quaternion(orientation));
}

void PlatformVarsQuat::updateAcc(const grabnum::Vector3d& _acceleration,
                                 const grabgeom::Quaternion& _orientation_ddot,
                                 const grabgeom::Quaternion& _orientation_dot,
                                 const grabnum::MatrixXd<3, 4>& _h_mat)
{
  linear_acc       = _acceleration;
  orientation_ddot = _orientation_ddot.q();
  dh_mat           = grabgeom::DHtfQuat(_orientation_dot);
  angular_acc      = dh_mat * _orientation_dot.q() + _h_mat * orientation_ddot;
}

void PlatformVarsQuat::updateAcc(const grabnum::Vector3d& _acceleration,
                                 const grabgeom::Quaternion& _orientation_ddot)
{
  updateAcc(_acceleration, _orientation_ddot, grabgeom::Quaternion(orientation_dot),
            h_mat);
}

void PlatformVarsQuat::update(const grabnum::Vector3d& _position,
                              const grabnum::Vector3d& _velocity,
                              const grabnum::Vector3d& _acceleration,
                              const grabgeom::Quaternion& _orientation,
                              const grabgeom::Quaternion& _orientation_dot,
                              const grabgeom::Quaternion& _orientation_ddot)
{
  updatePose(_position, _orientation);
  updateVel(_velocity, _orientation_dot);
  updateAcc(_acceleration, _orientation_ddot);
}

RobotVars::RobotVars(const size_t num_cables) : cables(std::vector<CableVars>(num_cables))
{
  /*resize();*/
}

RobotVars::RobotVars(const size_t num_cables, const RotParametrization _angles_type)
  : platform(_angles_type), cables(std::vector<CableVars>(num_cables))
{
  /*resize();*/
}

void RobotVars::resize()
{
  // Geometric jacobians
  geom_jacobian.clear();
  geom_jacobian_d.clear();
  //geom_jacobian.resize(cables.size(), POSE_DIM);
  //geom_jacobian_d.resize(cables.size(), POSE_DIM);
  // Analitic jacobians
  anal_jacobian.clear();
  anal_jacobian_d.clear();
  //anal_jacobian.resize(cables.size(), POSE_DIM);
  //anal_jacobian_d.resize(cables.size(), POSE_DIM);
  // Tension vector
  tension_vector.clear();
  //tension_vector.resize(cables.size());
}

void RobotVars::updateJacobians()
{
  // Safety check

 // if (geom_jacobian.n_rows != cables.size() || anal_jacobian.n_rows != cables.size())
    ///*resize();*/

  for (uint8_t i = 0; i < cables.size(); ++i)
  {

    geom_jacobian.row(i) = arma::rowvec6(cables[i].geom_jacob_row.Data());

    anal_jacobian.row(i) = arma::rowvec6(cables[i].anal_jacob_row.Data());

  }

}

RobotVarsQuat::RobotVarsQuat(const size_t num_cables)
  : cables(std::vector<CableVarsQuat>(num_cables))
{
  /*resize();*/
}

void RobotVarsQuat::resize()
{
  // Geometric jacobians
  geom_jacobian.clear();
  geom_jacobian_d.clear();
  //geom_jacobian.resize(cables.size(), POSE_DIM);
  //geom_jacobian_d.resize(cables.size(), POSE_DIM);
  // Analitic jacobians
  anal_jacobian.clear();
  anal_jacobian_d.clear();
  //anal_jacobian.resize(cables.size(), POSE_QUAT_DIM);
  //anal_jacobian_d.resize(cables.size(), POSE_QUAT_DIM);
  // Tension vector
  tension_vector.clear();
  //tension_vector.resize(cables.size());
}

void RobotVarsQuat::updateJacobians()
{
  // Safety check
  if (geom_jacobian.n_rows != cables.size() || anal_jacobian.n_rows != cables.size())
    /*resize();*/
  for (uint8_t i = 0; i < cables.size(); ++i)
  {
    geom_jacobian.row(i) = arma::rowvec6(cables[i].geom_jacob_row.Data());
    anal_jacobian.row(i) = arma::rowvec7(cables[i].anal_jacob_row.Data());
  }
}

} // namespace grabcdpr
