/**
 * @file cdpr_types.h
 * @author Edoardo Idà, Simone Comari
 * @date 30 Jul 2019
 * @brief File containing kinematics-related types to be included in the GRAB CDPR
 * library.
 *
 * @note
 *
 * Below we present a legend with all relevant symbols and nomenclature used throughout
 * the code.
 *
 * <table>
 * <caption id="legend">Systems of reference and geometric points</caption>
 * <tr><th>Name     <th>Symbol     <th>Description
 * <tr><td> _Global frame_  <td>@f$\mathcal{O}@f$ <td> World frame, centered on
 * point
 * @f$O@f$ and vertically aligned with gravitational axis.
 * <tr><td> _Local frame_  <td>@f$\mathcal{P}@f$ <td> Body-fixed frame centered
 * on an arbitrary point @f$P@f$ belonging to the platform and arbitrarly oriented.
 * <tr><td> _Swivel pulley frame_  <td>@f$\mathcal{D}_i@f$ <td> Fixed frame
 * centered on point @f$D@f$. Its @f$z@f$-axis represents the swiveling axis of the _i-th_
 * pulley.
 * <tr><td> _World origin_  <td>@f$O@f$ <td> Origin of global frame
 *@f$\mathcal{O}@f$.
 * <tr><td> _Local origin_  <td>@f$P@f$ <td> An arbitrary point rigidly fixed to the
 * platform, taken as the origin of _local frame_ @f$\mathcal{P}@f$.
 * <tr><td> _CoG_  <td>@f$G@f$ <td> The center of gravity (or baricenter) of the
 * platform.
 * <tr><td> _Swivel pulley frame origin_  <td> @f$D_i@f$ <td> A fixed point around
 * which the _i-th_ swivel pulley swivels. It also coincides with the entry point on the
 * pulley
 * of the cable unwinding from the _i-th_ winch. It is taken as origin of a fixed frame
 * used to parametrize the position of the pulley wrt global frame.
 * <tr><td> _Swivel pulley exit point_  <td>@f$B_i@f$ <td> The exit point of the
 * cable from the _i-th_ swivel pulley, going to the platform. At this point the cable is
 * assumed
 * to be tangent to the pulley and belonging to the pulley plane (always true in static
 * conditions).
 * <tr><td> _Platform attach point_  <td>@f$A_i@f$ <td> The attaching point of the
 * _i-th_ cable to the platform. This point is fixed wrt the local frame.
 * <tr><td> _Pulley center_  <td>@f$C_i@f$ <td> The revolving center of the _i-th_
 * pulley.
 * </table>
 *
 * <table>
 * <caption id="platform_vars">CDPR time-varying platform variables</caption>
 * <tr><th>Name     <th>Symbol     <th>Expression  <th>Description
 * <tr><td> _Position_  <td>@f$\mathbf{p}_P@f$ <td> @f$^\mathcal{O}(P - O)@f$ <td> Global
 * position of the platform expressed in global frame coordinates, i.e. of point @f$P@f$.
 * <tr><td> _Orientation_ <td> @f$\boldsymbol{\varepsilon}@f$ <td> - <td> Global
 * orientation of the platform expressed by 3 angles.
 * <tr><td> _Quaternion_ <td> @f$\boldsymbol{\varepsilon}_q@f$ <td> - <td> Global
 * orientation of the platform expressed by a quaternion @f$(q_w, q_x, q_y, q_z)^T@f$.
 * <tr><td> _Pose_ <td> @f$\mathbf{x}@f$ <td>
 * @f$[\mathbf{p}_P^T, \boldsymbol{\varepsilon}^T]^T@f$ <td> Platform pose or generalized
 * variables using angles.
 * <tr><td> _Pose variant_ <td> @f$\mathbf{x}_q@f$ <td>
 * @f$[\mathbf{p}_P^T, \boldsymbol{\varepsilon}_q^T]^T@f$ <td> Platform pose or
 * generalized variables using quaternion.
 * <tr><td> _Velocity_ <td> @f$\mathbf{v}@f$ <td>
 * @f$[\dot{\mathbf{p}}_P^T, \boldsymbol{\omega}^T]^T@f$ <td> Platform linear and angular
 * velocity.
 * </table>
 *
 * <table>
 * <caption id="cable_vars">CDPR time-varying cable variables</caption>
 * <tr><th>Name     <th>Symbol     <th>Expression  <th>Description
 * <tr><td> _Number of cables_  <td> @f$N@f$ <td> - <td> Number of cables attached to the
 * platform.
 * <tr><td> _Cable length_  <td> @f$l_i@f$ <td> @f$\|A_i - B_i\|@f$ <td> Length of _i-th_
 * cable from point @f$B_i@f$ to point @f$A_i@f$, including winding around swivel pulley.
 * <tr><td> _Motor counts_ <td>@f$q_i@f$ <td> - <td> Motor counts (i.e. position) of
 * _i-th_ motor.
 * <tr><td> _Swivel angle_ <td>@f$\sigma_i@f$ <td> - <td> _i-th_ pulley swivel angle, i.e.
 * the angle between @f$\hat{\mathbf{x}}_i@f$ and @f$\hat{\mathbf{u}}_i@f$.
 * <tr><td> _Tangent angle_ <td>@f$\psi_i@f$ <td> - <td> _i-th_ pulley tangent angle, i.e.
 * the angle between @f$\hat{\mathbf{u}}_i@f$ and @f$\hat{\mathbf{n}}_i@f$.
 * </table>
 *
 * <table>
 * <caption id="params">CDPR static parameters</caption>
 * <tr><th>Name     <th>Symbol     <th>Expression  <th>Description
 * <tr><td> - <td> @f$^\mathcal{P}\mathbf{p}'_G@f$ <td> @f$^\mathcal{P}(G - P)@f$
 * <td> Local position of the platform's CoG @f$G@f$ expressed in local frame coordinates.
 * <tr><td> - <td> @f$^\mathcal{P}\mathbf{a}'_i@f$ <td> @f$^\mathcal{P}(A_i - P)@f$
 * <td> Local position of the attaching point of the _i-th_ cable to the platform
 * expressed in local frame coordinates.
 * <tr><td> _Platform mass_ <td> @f$m@f$ <td> - <td> Platform mass in [Kg].
 * <tr><td> _Platform inertia_ <td> @f$\mathcal{P}\mathbf{I}_G@f$ <td> - <td> Platform
 * inertia expressed in local frame P.
 * <tr><td> _Motor inertia_ <td> @f$\mathbf{I}_q@f$ <td> - <td> Motor inertia.
 * <tr><td> _Transmittion ratio_ <td> @f$\tau_i@f$ <td> - <td> _i-th_ motor counts-to-
 * cable-length factor.
 * <tr><td> _Pulley radius_ <td>@f$r_i@f$ <td> - <td> _i-th_ swivel pulley radius length.
 * <tr><td> -  <td>@f$\mathbf{d}_i@f$ <td> @f$^\mathcal{O}(D_i - O)@f$ <td> Global
 * position of point @f$D_i@f$ expressed in global frame coordinates. This is a fixed
 * vector.
 * <tr><td> - <td>@f$\hat{\mathbf{i}}_i@f$ <td> - <td> @f$x@f$-axis versor of _i-th_
 * swivel pulley frame @f$\mathcal{D}_i@f$.
 * <tr><td> - <td>@f$\hat{\mathbf{j}}_i@f$ <td> - <td> @f$y@f$-axis versor of _i-th_
 * swivel pulley frame @f$\mathcal{D}_i@f$.
 * <tr><td> - <td>@f$\hat{\mathbf{k}}_i@f$ <td> - <td> @f$z@f$-axis versor of _i-th_
 * swivel pulley frame @f$\mathcal{D}_i@f$. This is also the swiveling axis of the pulley.
 * </table>
 *
 * <table>
 * <caption id="ancillary">CDPR ancillary variables</caption>
 * <tr><th>Name     <th>Symbol     <th>Expression  <th>Description
 * <tr><td> -  <td>@f$\mathbf{p}_G@f$ <td> @f$^\mathcal{O}(G - O)@f$ <td> Global
 * position of the platform's CoG @f$G@f$ expressed in global frame coordinates.
 * <tr><td> -  <td>@f$\mathbf{p}'_G@f$ <td> @f$^\mathcal{O}(G - P)@f$ <td> Local
 * position of the platform's CoG @f$G@f$ expressed in global frame coordinates.
 * <tr><td> -  <td>@f$\mathbf{a}_i@f$ <td> @f$^\mathcal{O}(A_i - O)@f$ <td> Global
 * position of the attaching point of the _i-th_ cable to the platform expressed in global
 * frame coordinates.
 * <tr><td> -  <td>@f$\mathbf{a}'_i@f$ <td> @f$^\mathcal{O}(A_i - P)@f$ <td> Local
 * position of the attaching point of the _i-th_ cable to the platform expressed in global
 * frame
 * coordinates.
 * <tr><td> -  <td>@f$\boldsymbol{\rho}^*_i@f$ <td> @f$^\mathcal{O}(A_i - D_i)@f$ <td>
 * Time-variant vector expressed in global frame coordinates.
 * <tr><td> -  <td>@f$\mathbf{n}_i@f$ <td> @f$^\mathcal{O}(B_i - C_i)@f$ <td>
 * Time-variant vector expressed in global frame coordinates. Used to define tangent
 * angle.
 * <tr><td> _Cable vector_  <td>@f$\boldsymbol{\rho}_i@f$ <td>
 * @f$^\mathcal{O}(A_i - B_i)@f$ <td> Cable vector expressed in global frame coordinates.
 * <tr><td> - <td>@f$\hat{\mathbf{u}}_i@f$ <td>
 * @f$\hat{\mathbf{u}}_i \perp \hat{\mathbf{w}}_i \perp \hat{\mathbf{k}}_i@f$ <td>
 * @f$x@f$-axis versor of a time-variant body-fixed frame to the _i-th_ swivel pulley. In
 * particular, @f$\hat{\mathbf{u}}_i@f$ belongs to the pulley plane and it is
 * perpendicular to @f$\hat{\mathbf{k}}_i@f$.
 * <tr><td> - <td>@f$\hat{\mathbf{w}}_i@f$ <td>
 * @f$\hat{\mathbf{u}}_i \perp \hat{\mathbf{w}}_i \perp \hat{\mathbf{k}}_i@f$ <td>
 * @f$y@f$-axis versor of a time-variant body-fixed frame to the _i-th_ swivel pulley. In
 * particular, @f$\hat{\mathbf{w}}_i@f$ is perpendicular to the pulley plane and to
 * @f$\hat{\mathbf{k}}_i@f$.
 * <tr><td> - <td>@f$\hat{\mathbf{n}}_i@f$ <td> - <td> Time-variant versor denoting the
 * direction of @f$\mathbf{n}_i@f$ and used to define the tangent angle @f$\psi_i@f$. Note
 * that @f$\mathbf{n}_i = r_i \hat{\mathbf{n}}_i@f$.
 * <tr><td> - <td>@f$\hat{\mathbf{t}}_i@f$ <td> - <td> Time-variant versor
 * denoting the direction of @f$\boldsymbol{\rho}_i@f$. Note that @f$\hat{\mathbf{t}}_i
 * \perp \hat{\mathbf{n}}_i@f$.
 * </table>
 *
 * <table>
 * <caption id="other_notation">CDPR ancillary variables</caption>
 * <tr><th>Name     <th>Symbol     <th>Expression  <th>Description
 * <tr><td> _Rotation matrix_ <td> @f$\mathbf{R}@f$ <td>
 * @f$\mathbf{R}(\boldsymbol{\varepsilon})@f$ <td> Rotation matrix from local to global
 * frame, function of the selected parametrization for the platform orientation.
 * <tr><td> _Transformation matrix_ <td> @f$\mathbf{H}@f$ <td>
 * @f$\mathbf{H}(\boldsymbol{\varepsilon})@f$ <td> Transformation matrix from angles
 * velocities @f$\dot{\boldsymbol{\varepsilon}}@f$ to angular speed vector
 * @f$\boldsymbol{\omega}@f$, function of the selected parametrization for the platform
 * orientation.
 * <tr><td> _Time-derivative_ <td> @f$\dot{(\cdot)}@f$ <td> @f$\frac{d(\cdot)}{dt}@f$
 * <td> -
 * <tr><td> _Second time-derivative_ <td> @f$\ddot{(\cdot)}@f$ <td>
 * @f$\frac{d^2(\cdot)}{dt^2}@f$ <td> -
 * <tr><td> _Skew-symmetric_ <td> @f$\tilde{(\cdot)}@f$ <td> @f$(\cdot) \times@f$ <td>
 * Skew-symmetric matrix of a vector, equivalent to its cross product.
 * <tr><td> _Angular speed_ <td> @f$\boldsymbol{\omega}@f$ <td>
 * @f$\mathbf{H}(\boldsymbol{\varepsilon})\dot{\boldsymbol{\varepsilon}}@f$ <td> Angular
 * speed vector of the platform expressed in global coordinates.
 * <tr><td> _Angular acceleration_ <td> @f$\boldsymbol{\alpha}@f$ <td>
 * @f$\dot{\mathbf{H}}(\dot{\boldsymbol{\varepsilon}}, \boldsymbol{\varepsilon})
 * \dot{\boldsymbol{\varepsilon}} + \mathbf{H}(\boldsymbol{\varepsilon})
 * \ddot{\boldsymbol{\varepsilon}} @f$ <td> Angular acceleration vector of the platform
 * expressed in global coordinates.
 * </table>
 */

#ifndef GRABCOMMON_LIBCDPR_CDPR_TYPES_H
#define GRABCOMMON_LIBCDPR_CDPR_TYPES_H

#include <armadillo>

#include "matrix_utilities.h"
#include "quaternions.h"
#include "rotations.h"

#define POSE_DIM 6
#define POSE_QUAT_DIM 7

/**
 * @brief Namespace for CDPR-related utilities, such as kinematics and dynamics.
 */
namespace grabcdpr {

//------ Enums -----------------------------------------------------------------------//

/**
 * @brief Rotation parametrization enum.
 *
 * Defines the way a rotation matrix is determined according to the convention used.
 */
enum RotParametrization
{
  EULER_ZYZ,    /**< _Euler_ angles convention with @f$Z_1Y_2Z_3@f$ order. */
  TAIT_BRYAN,   /**< _Tait-Bryan_ angles convention and @f$X_1Y_2Z_3@f$. */
  RPY,          /**< _Roll, Pitch, Yaw_ angles convention (from aviation). */
  TILT_TORSION, /**< _Tilt-and-torsion_ angles, a variation of _Euler_ angles convention.
                 */
  QUATERNION    /**< _Quaternions_ convention (not angles). */
};

//------ Structs ---------------------------------------------------------------------//

/**
 * @brief Structure collecting all common variables related to a generic 6DoF platform.
 * @see PlatformVarsStruct PlatformQuatVarsStruct
 * @note See @ref legend for symbols reference.
 */
struct PlatformVarsBase
{
  /** @addtogroup ZeroOrderKinematics
   * @{
   */
  grabnum::Vector3d position; /**< [_m_] vector @f$\mathbf{p}_P@f$. */

  grabnum::Matrix3d rot_mat; /**< matrix @f$\mathbf{R}@f$. */

  grabnum::Vector3d pos_PG_glob; /**< [_m_] vector @f$\mathbf{p}'_G@f$.*/
  grabnum::Vector3d pos_OG_glob; /**< [_m_] vector @f$\mathbf{p}_G@f$.*/
  /** @} */                      // end of ZeroOrderKinematics group

  /** @addtogroup FirstOrderKinematics
   * @{
   */
  grabnum::Vector3d velocity; /**< [_m/s_] vector @f$\dot{\mathbf{p}}_P@f$. */

  grabnum::Vector3d angular_vel; /**< vector @f$\boldsymbol\omega@f$. */

  grabnum::Vector3d vel_OG_glob; /**< [_m/s_] vector @f$\dot{\mathbf{p}}_G@f$. */
  /** @} */                      // end of FirstOrderKinematics group

  /** @addtogroup SecondOrderKinematics
   * @{
   */
  grabnum::Vector3d
    acceleration; /**< [_m/s<sup>2</sup>_] vector @f$\ddot{\mathbf{p}}_P@f$. */

  grabnum::Vector3d angular_acc; /**< vector @f$\boldsymbol\alpha@f$.*/

  grabnum::Vector3d
    acc_OG_glob; /**< [_m/s<sup>2</sup>_] vector @f$\ddot{\mathbf{p}}_G@f$.*/
  /** @} */      // end of SecondOrderKinematics group

  grabnum::VectorXd<POSE_DIM> ext_load; /**< vector containing component of external
                                    forces and moments, expressed in the global frame. */
};

/**
 * @brief Structure collecting all variables related to minimal orientation
 * parametrization of a generic 6DoF platform, i.e. with 3 angles.
 * @see PlatformQuatVarsStruct
 * @note See @ref legend for symbols reference.
 */
struct PlatformVars: PlatformVarsBase
{
  RotParametrization angles_type; /**< rotation parametrization used. */

  /** @addtogroup ZeroOrderKinematics
   * @{
   */
  grabnum::Vector3d orientation; /**< [_rad_] vector @f$\boldsymbol{\varepsilon}@f$. */

  grabnum::Matrix3d h_mat;  /**< matrix @f$\mathbf{H}@f$. */
  grabnum::Matrix3d dh_mat; /**< matrix @f$\dot{\mathbf{H}}@f$. */

  grabnum::VectorXd<POSE_DIM> pose; /**< vector @f$\mathbf{q}@f$.  */
  /** @} */                         // end of ZeroOrderKinematics group

  /** @addtogroup FirstOrderKinematics
   * @{
   */
  grabnum::Vector3d
    orientation_dot; /**< [_rad/s_] vector @f$\dot{\boldsymbol{\varepsilon}}@f$. */
  /** @} */          // end of FirstOrderKinematics group

  /** @addtogroup SecondOrderKinematics
   * @{
   */
  grabnum::Vector3d orientation_ddot; /**< [_rad/s<sup>2</sup>_] vector
                                   @f$\ddot{\boldsymbol{\varepsilon}}@f$. */
  /** @} */                           // end of SecondOrderKinematics group

  grabnum::VectorXd<POSE_DIM> ext_load_ss; /**< vector containing component of external
                                    forces and moments, projected onto the state-space. */

  /**
   * @brief Constructor to explicitly declare rotation parametrization desired only.
   * @param[in] _angles_type Desired rotation parametrization.
   */
  PlatformVars(const RotParametrization _angles_type = TILT_TORSION)
  {
    angles_type = _angles_type;
  }

  /**
   * @brief Constructor to initialize platform vars with position and angles.
   * @param[in] _position [m] Platform global position @f$\mathbf{p}_P@f$.
   * @param[in] _orientation [rad] Platform global orientation expressed by angles
   * @f$\boldsymbol{\varepsilon}@f$.
   * @param[in] _angles_type Desired rotation parametrization. Default is @a TILT_TORSION.
   * @note See @ref legend for more details.
   * @see UpdatePose()
   */
  PlatformVars(const grabnum::Vector3d& _position, const grabnum::Vector3d& _orientation,
               const RotParametrization _angles_type = TILT_TORSION)
  {
    angles_type = _angles_type;
    UpdatePose(_position, _orientation);
  }
  /**
   * @brief Constructor to initialize platform vars with position and angles and their
   * first derivatives.
   * @param[in] _position [m] Platform global position @f$\mathbf{p}_P@f$.
   * @param[in] _velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}_P@f$.
   * @param[in] _orientation [rad] Platform global orientation expressed by angles
   * @f$\boldsymbol{\varepsilon}@f$.
   * @param[in] _orientation_dot [rad/s] Platform orientation time-derivative
   * @f$\dot{\boldsymbol{\varepsilon}}@f$.
   * @param[in] _angles_type Desired rotation parametrization. Default is @a TILT_TORSION.
   * @note See @ref legend for more details.
   * @see Update()
   */
  PlatformVars(const grabnum::Vector3d& _position, const grabnum::Vector3d& _velocity,
               const grabnum::Vector3d& _orientation,
               const grabnum::Vector3d& _orientation_dot,
               const RotParametrization _angles_type = TILT_TORSION)
  {
    angles_type = _angles_type;
    UpdatePose(_position, _orientation);
    UpdateVel(_velocity, _orientation_dot);
  }
  /**
   * @brief Constructor to initialize platform vars with position and angles and their
   * first and second derivatives.
   * @param[in] _position [m] Platform global position @f$\mathbf{p}_P@f$.
   * @param[in] _velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}_P@f$.
   * @param[in] _acceleration [m/s<sup>2</sup>] Platform global linear acceleration
   * @f$\ddot{\mathbf{p}}_P@f$.
   * @param[in] _orientation [rad] Platform global orientation expressed by angles
   * @f$\boldsymbol{\varepsilon}@f$.
   * @param[in] _orientation_dot [rad/s] Platform orientation time-derivative
   * @f$\dot{\boldsymbol{\varepsilon}}@f$.
   * @param[in] _orientation_ddot [rad/s<sup>2</sup>] Platform orientation 2nd
   * time-derivative @f$\ddot{\boldsymbol{\varepsilon}}@f$.
   * @param[in] _angles_type Desired rotation parametrization. Default is @a TILT_TORSION.
   * @note See @ref legend for more details.
   * @see Update()
   */
  PlatformVars(const grabnum::Vector3d& _position, const grabnum::Vector3d& _velocity,
               const grabnum::Vector3d& _acceleration,
               const grabnum::Vector3d& _orientation,
               const grabnum::Vector3d& _orientation_dot,
               const grabnum::Vector3d& _orientation_ddot,
               const RotParametrization _angles_type = TILT_TORSION)
  {
    angles_type = _angles_type;
    Update(_position, _velocity, _acceleration, _orientation, _orientation_dot,
           _orientation_ddot);
  }

  /**
   * @brief Update platform pose with position and angles.
   * @param[in] _position [m] Platform global position @f$\mathbf{p}_P@f$.
   * @param[in] _orientation [rad] Platform global orientation expressed by angles
   * @f$\boldsymbol{\varepsilon}@f$.
   * @todo handle default case better
   * @ingroup ZeroOrderKinematics
   * @see UpdateVel() UpdateAcc() Update()
   * @note See @ref legend for more details.
   */
  void UpdatePose(const grabnum::Vector3d& _position,
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
        h_mat = grabgeom::HtfZYZ(_orientation);
        break;
      case TAIT_BRYAN:
        rot_mat = grabgeom::EulerXYZ2Rot(orientation);
        h_mat = grabgeom::HtfXYZ(_orientation);
        break;
      case RPY:
        rot_mat = grabgeom::RPY2Rot(orientation);
        h_mat = grabgeom::HtfRPY(_orientation);
        break;
      case TILT_TORSION:
        rot_mat = grabgeom::TiltTorsion2Rot(orientation);
        h_mat = grabgeom::HtfTiltTorsion(_orientation);
        break;
      default:
        // This should never happen
        break;
    }
  }

  /**
   * @brief Update platform velocities with linear velocity and angles speed.
   * @param[in] _velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}_P@f$.
   * @param[in] _orientation_dot [rad/s] Vector @f$\dot{\boldsymbol{\varepsilon}}@f$.
   * @param[in] _orientation [rad] Platform global orientation expressed by angles
   * @f$\boldsymbol{\varepsilon}@f$.
   * @ingroup FirstOrderKinematics
   * @see UpdatePose() UpdateAcc() Update()
   * @note See @ref legend for more details.
   */
  void UpdateVel(const grabnum::Vector3d& _velocity,
                 const grabnum::Vector3d& _orientation_dot,
                 const grabnum::Vector3d& _orientation)
  {
    UpdatePose(position, _orientation); // update h_mat
    UpdateVel(_velocity, _orientation_dot);
  }

  /**
   * @brief Update platform velocities with linear velocity and angles speed.
   *
   * Platform orientation needed for the update are infered from current values of
   * structure members, so make sure it is up-to-date by using UpdatePose() first.
   * @param[in] _velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}_P@f$.
   * @param[in] _orientation_dot [rad/s] Vector @f$\dot{\boldsymbol{\varepsilon}}@f$.
   * @ingroup FirstOrderKinematics
   * @see UpdateVel()
   * @note See @ref legend for more details.
   */
  void UpdateVel(const grabnum::Vector3d& _velocity,
                 const grabnum::Vector3d& _orientation_dot)
  {
    velocity        = _velocity;
    orientation_dot = _orientation_dot;
    angular_vel = h_mat * orientation_dot;
  }

  /**
   * @brief Update platform accelerations with linear and angles acceleration.
   * @param[in] _acceleration [m/s<sup>2</sup>] Vector @f$\ddot{\mathbf{p}}_P@f$.
   * @param[in] _orientation_ddot [rad/s<sup>2</sup>] Vector
   * @f$\ddot{\boldsymbol{\varepsilon}}@f$.
   * @param[in] _orientation_dot [rad/s] Vector @f$\dot{\boldsymbol{\varepsilon}}@f$.
   * @param[in] _orientation [rad] Platform global orientation expressed by angles
   * @f$\boldsymbol{\varepsilon}@f$.
   * @param[in] _h_mat Transformation matrix @f$\mathbf{H}@f$.
   * @ingroup SecondOrderKinematics
   * @see UpdateVel() UpdatePose() Update()
   * @note See @ref legend for more details.
   */
  void UpdateAcc(const grabnum::Vector3d& _acceleration,
                 const grabnum::Vector3d& _orientation_ddot,
                 const grabnum::Vector3d& _orientation_dot,
                 const grabnum::Vector3d& _orientation, const grabnum::Matrix3d& _h_mat)
  {
    acceleration     = _acceleration;
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
  }
  /**
   * @brief Update platform accelerations with linear and quaternion acceleration.
   *
   * Platform orientation and angular velocity needed for the update are infered from
   * current values of structure members, so make sure they are up-to-date by using
   * UpdateVel() first.
   * @param[in] _acceleration [m/s<sup>2</sup>] Vector @f$\ddot{\mathbf{p}}_P@f$.
   * @param[in] _orientation_ddot [rad/s<sup>2</sup>] Vector
   * @f$\ddot{\boldsymbol{\varepsilon}}@f$.
   * @ingroup SecondOrderKinematics
   * @see UpdateAcc()
   * @note See @ref legend for more details.
   */
  void UpdateAcc(const grabnum::Vector3d& _acceleration,
                 const grabnum::Vector3d& _orientation_ddot)
  {
    UpdateAcc(_acceleration, _orientation_ddot, orientation_dot, orientation, h_mat);
  }

  /**
   * @brief Update platform vars with position and angles and their first and second
   * derivatives.
   * @param[in] _position [m] Platform global position @f$\mathbf{p}_P@f$.
   * @param[in] _velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}_P@f$.
   * @param[in] _acceleration [m/s<sup>2</sup>] Platform global linear acceleration
   * @f$\ddot{\mathbf{p}}_P@f$.
   * @param[in] _orientation [rad] Platform global orientation expressed by angles
   * @f$\boldsymbol{\varepsilon}@f$.
   * @param[in] _orientation_dot [rad/s] Platform orientation time-derivative
   * @f$\dot{\boldsymbol{\varepsilon}}@f$.
   * @param[in] _orientation_ddot [rad/s<sup>2</sup>] Platform orientation 2nd
   * time-derivative
   * @f$\ddot{\boldsymbol{\varepsilon}}@f$.
   * @note See @ref legend for more details.
   * @see UpdatePose() UpdateVel() UpdateAcc()
   */
  void Update(const grabnum::Vector3d& _position, const grabnum::Vector3d& _velocity,
              const grabnum::Vector3d& _acceleration,
              const grabnum::Vector3d& _orientation,
              const grabnum::Vector3d& _orientation_dot,
              const grabnum::Vector3d& _orientation_ddot)
  {
    UpdatePose(_position, _orientation);
    UpdateVel(_velocity, _orientation_dot);
    UpdateAcc(_acceleration, _orientation_ddot);
  }
};

/**
 * @brief Structure collecting all variables related to non-minimal orientation
 * parametrization of a generic 6DoF platform, i.e. with quaternions.
 * @see PlatformVarsQuatStruct
 * @note See @ref legend for symbols reference.
 */
struct PlatformQuatVars: PlatformVarsBase
{
  /** @addtogroup ZeroOrderKinematics
   * @{
   */
  grabnum::VectorXd<4> orientation; /**< vector @f$\boldsymbol{\varepsilon}_q@f$. */

  grabnum::MatrixXd<3, 4> h_mat;  /**< matrix @f$\mathbf{H}_q@f$. */
  grabnum::MatrixXd<3, 4> dh_mat; /**< matrix @f$\dot{\mathbf{H}}_q@f$. */

  grabnum::VectorXd<POSE_QUAT_DIM> pose; /**< vector @f$\mathbf{x}_q@f$. */
  /** @} */                              // end of ZeroOrderKinematics group

  /** @addtogroup FirstOrderKinematics
   * @{
   */
  grabnum::VectorXd<4>
    orientation_dot; /**< quaternion @f$\dot{\boldsymbol{\varepsilon}}_q@f$. */
  /** @} */          // end of FirstOrderKinematics group

  /** @addtogroup SecondOrderKinematics
   * @{
   */
  grabnum::VectorXd<4>
    orientation_ddot; /**< quaternion @f$\ddot{\boldsymbol{\varepsilon}}_q@f$. */
  /** @} */           // end of SecondOrderKinematics group

  grabnum::VectorXd<POSE_QUAT_DIM>
    ext_load_ss; /**< vector containing component of external
     forces and moments, projected onto the state-space. */

  /**
   * @brief PlatformQuatVars default constructor.
   */
  PlatformQuatVars() {}
  /**
   * @brief Constructor to initialize platform vars with position and orientation and
   * their first and second derivatives.
   * @param[in] _position [m] Platform global position @f$\mathbf{p}_P@f$.
   * @param[in] _velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}_P@f$.
   * @param[in] _acceleration [m/s<sup>2</sup>] Platform global linear acceleration
   * @f$\ddot{\mathbf{p}}_P@f$.
   * @param[in] _orientation Platform global orientation expressed by quaternion
   * @f$\boldsymbol{\varepsilon}_q@f$.
   * @param[in] _orientation_dot Platform orientation time-derivative
   * @f$\dot{\boldsymbol{\varepsilon}}_q@f$.
   * @param[in] _orientation_ddot Platform orientation 2<sup>nd</sup> time-derivative
   * @f$\ddot{\boldsymbol{\varepsilon}}_q@f$.
   * @note See @ref legend for more details.
   * @see Update()
   */
  PlatformQuatVars(const grabnum::Vector3d& _position, const grabnum::Vector3d& _velocity,
                   const grabnum::Vector3d& _acceleration,
                   const grabgeom::Quaternion& _orientation,
                   const grabgeom::Quaternion& _orientation_dot,
                   const grabgeom::Quaternion& _orientation_ddot)
  {
    Update(_position, _velocity, _acceleration, _orientation, _orientation_dot,
           _orientation_ddot);
  }

  /**
   * @brief Update platform pose with position and quaternion.
   * @param[in] _position [m] Platform global position @f$\mathbf{p}_P@f$.
   * @param[in] _orientation Platform global orientation expressed by quaternion
   * @f$\boldsymbol{\varepsilon}_q = (q_w, q_x, q_y, q_z)@f$.
   * @todo automatically update orientation from quaternion.
   * @ingroup ZeroOrderKinematics
   * @see UpdateVel() UpdateAcc()
   * @note See @ref legend for more details.
   */
  void UpdatePose(const grabnum::Vector3d& _position,
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

  /**
   * @brief Update platform velocities with linear velocity and angles speed.
   * @param[in] _velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}_P@f$.
   * @param[in] _orientation_dot Quaternion velocity
   * @f$\dot{\boldsymbol{\varepsilon}}_q@f$.
   * @param[in] _orientation Platform global orientation expressed by quaternion
   * @f$\boldsymbol{\varepsilon}_q = (q_w, q_x, q_y, q_z)@f$.
   * @ingroup FirstOrderKinematics
   * @see UpdatePose() UpdateAcc()
   * @note See @ref legend for more details.
   */
  void UpdateVel(const grabnum::Vector3d& _velocity,
                 const grabgeom::Quaternion& _orientation_dot,
                 const grabgeom::Quaternion& _orientation)
  {
    velocity        = _velocity;
    orientation_dot = _orientation_dot.q();
    h_mat           = grabgeom::HtfQuat(_orientation);
    angular_vel     = h_mat * orientation_dot;
  }
  /**
   * @brief Update platform velocities with linear velocity and angles speed.
   *
   * Platform orientation needed for the update are infered from current values of
   * structure members, so make sure they are up-to-date by using UpdatePose() first.
   * @param[in] _velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}_P@f$.
   * @param[in] _orientation_dot Quaternion velocity
   * @f$\dot{\boldsymbol{\varepsilon}}_q@f$.
   * @ingroup FirstOrderKinematics
   * @see UpdateVel()
   * @note See @ref legend for more details.
   */
  void UpdateVel(const grabnum::Vector3d& _velocity,
                 const grabgeom::Quaternion& _orientation_dot)
  {
    UpdateVel(_velocity, _orientation_dot, grabgeom::Quaternion(orientation));
  }

  /**
   * @brief Update platform accelerations with linear and quaternion acceleration.
   *
   * @param[in] _acceleration [m/s<sup>2</sup>] Vector @f$\ddot{\mathbf{p}}_P@f$.
   * @param[in] _orientation_ddot Quaternion acceleration
   * @f$\ddot{\boldsymbol{\varepsilon}}_q@f$.
   * @param[in] _orientation_dot Quaternion speed @f$\dot{\boldsymbol{\varepsilon}}_q@f$.
   * @param[in] _h_mat Transformation matrix @f$\mathbf{H}_q@f$.
   * @ingroup SecondOrderKinematics
   * @note See @ref legend for more details.
   */
  void UpdateAcc(const grabnum::Vector3d& _acceleration,
                 const grabgeom::Quaternion& _orientation_ddot,
                 const grabgeom::Quaternion& _orientation_dot,
                 const grabnum::MatrixXd<3, 4>& _h_mat)
  {
    acceleration     = _acceleration;
    orientation_ddot = _orientation_ddot.q();
    dh_mat           = grabgeom::DHtfQuat(_orientation_dot);
    angular_acc      = dh_mat * _orientation_dot.q() + _h_mat * orientation_ddot;
  }
  /**
   * @brief Update platform accelerations with linear and quaternion acceleration.
   *
   * Platform angular velocity needed for the update are infered from current values of
   * structure members, so make sure they are up-to-date by using UpdateVel() first.
   * @param[in] _acceleration [m/s<sup>2</sup>] Vector @f$\ddot{\mathbf{p}}_P@f$.
   * @param[in] _orientation_ddot Quaternion acceleration
   * @f$\ddot{\boldsymbol{\varepsilon}}_q@f$.
   * @ingroup SecondOrderKinematics
   * @note See @ref legend for more details.
   */
  void UpdateAcc(const grabnum::Vector3d& _acceleration,
                 const grabgeom::Quaternion& _orientation_ddot)
  {
    UpdateAcc(_acceleration, _orientation_ddot, grabgeom::Quaternion(orientation_dot),
              h_mat);
  }

  /**
   * @brief Update platform vars with position and orientation and their first and second
   * derivatives.
   * @param[in] _position [m] Platform global position @f$\mathbf{p}_P@f$.
   * @param[in] _velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}_P@f$.
   * @param[in] _acceleration [m/s<sup>2</sup>] Platform global linear acceleration
   * @f$\ddot{\mathbf{p}}_P@f$.
   * @param[in] _orientation Platform global orientation expressed by quaternion
   * @f$\boldsymbol{\varepsilon}_q@f$.
   * @param[in] _orientation_dot Platform orientation time-derivative
   * @f$\dot{\boldsymbol{\varepsilon}}_q@f$.
   * @param[in] _orientation_ddot Platform orientation 2<sup>nd</sup> time-derivative
   * @f$\ddot{\boldsymbol{\varepsilon}}_q@f$.
   * @note See @ref legend for more details.
   * @see UpdatePose() UpdateVel() UpdateAcc()
   */
  void Update(const grabnum::Vector3d& _position, const grabnum::Vector3d& _velocity,
              const grabnum::Vector3d& _acceleration,
              const grabgeom::Quaternion& _orientation,
              const grabgeom::Quaternion& _orientation_dot,
              const grabgeom::Quaternion& _orientation_ddot)
  {
    UpdatePose(_position, _orientation);
    UpdateVel(_velocity, _orientation_dot);
    UpdateAcc(_acceleration, _orientation_ddot);
  }
};

/**
 * @brief Structure collecting variable related to a single generic cable of a CDPR.
 * @note See @ref legend for symbols reference.
 */
struct CableVarsBase
{
  /** @addtogroup ZeroOrderKinematics
   * @{
   */
  double length; /**< [_m_] cable length @f$l_i@f$. */

  double swivel_ang; /**< [_rad_] _i-th_ pulley swivel angle @f$\sigma_i@f$. */
  double tan_ang;    /**< [_rad_] _i-th_ pulley tangent angle @f$\psi_i@f$. */

  grabnum::Vector3d pos_PA_glob; /**< [_m_] vector @f$\mathbf{a}'_i@f$. */
  grabnum::Vector3d pos_OA_glob; /**< [_m_] vector @f$\mathbf{a}_i@f$. */
  grabnum::Vector3d pos_DA_glob; /**< [_m_] vector @f$\boldsymbol{\rho}^*_i@f$. */
  grabnum::Vector3d pos_BA_glob; /**< [_m_] vector @f$\boldsymbol{\rho}_i@f$. */

  grabnum::Vector3d vers_u; /**< _i-th_ swivel pulley versor @f$\hat{\mathbf{u}}_i@f$. */
  grabnum::Vector3d vers_w; /**< _i-th_ swivel pulley versor @f$\hat{\mathbf{w}}_i@f$. */
  grabnum::Vector3d vers_n; /**< _i-th_ swivel pulley versor @f$\hat{\mathbf{n}}_i@f$. */
  grabnum::Vector3d vers_t; /**< _i-th_ cable versor @f$\hat{\mathbf{t}}_i@f$. */

  grabnum::MatrixXd<1, POSE_DIM> geom_jacob_row; /**< _i-th_ row of geometric jacobian. */
  /** @} */                                      // end of ZeroOrderKinematics group

  /** @addtogroup FirstOrderKinematics
   * @{
   */
  double speed; /**< [_m/s_] _i-th_ cable speed @f$\dot{l}_i@f$. */

  double swivel_ang_vel; /**< [_rad/s_] _i-th_ pulley swivel angle speed
                            @f$\dot{\sigma}_i@f$. */
  double
    tan_ang_vel; /**< [_rad/s_] _i-th_ pulley tangent angle speed @f$\dot{\psi}_i@f$. */

  grabnum::Vector3d vel_OA_glob; /**< [_m/s_] vector @f$\dot{\mathbf{a}}_i@f$. */
  grabnum::Vector3d vel_BA_glob; /**< [_m/s_] vector @f$\dot{\boldsymbol{\rho}}_i@f$. */

  grabnum::Vector3d vers_u_dot; /**< versor @f$\dot{\hat{\mathbf{u}}}_i@f$. */
  grabnum::Vector3d vers_w_dot; /**< versor @f$\dot{\hat{\mathbf{w}}}_i@f$. */
  grabnum::Vector3d vers_n_dot; /**< versor @f$\dot{\hat{\mathbf{n}}}_i@f$. */
  grabnum::Vector3d vers_t_dot; /**< versor @f$\dot{\hat{\mathbf{t}}}_i@f$. */
  /** @} */                     // end of FirstOrderKinematics group

  /** @addtogroup SecondOrderKinematics
   * @{
   */
  double acceleration; /**< [_m/s<sup>2</sup>_] cable acceleration @f$\ddot{l}_i@f$. */

  double
    swivel_ang_acc;   /**< [_rad/s<sup>2</sup>_] _i-th_ pulley @f$\ddot{\sigma}_i@f$. */
  double tan_ang_acc; /**< [_rad/s<sup>2</sup>_] _i-th_ pulley @f$\ddot{\psi}_i@f$. */

  grabnum::Vector3d
    acc_OA_glob; /**< [_m/s<sup>2</sup>_] vector @f$\ddot{\mathbf{a}}_i@f$. */
  /** @} */      // end of SecondOrderKinematics group
};

struct CableVars: CableVarsBase
{
  /** @addtogroup ZeroOrderKinematics
   * @{
   */
  grabnum::MatrixXd<1, POSE_DIM> anal_jacob_row; /**< _i-th_ row of analitic jacobian. */
  /** @} */                                      // end of ZeroOrderKinematics group
};

struct CableVarsQuat: CableVarsBase
{
  /** @addtogroup ZeroOrderKinematics
   * @{
   */
  grabnum::MatrixXd<1, POSE_QUAT_DIM>
    anal_jacob_row; /**< _i-th_ row of analitic jacobian. */
  /** @} */         // end of ZeroOrderKinematics group
};

/**
 * @brief Structure collecting all variables related to a generic 6DoF CDPR.
 *
 * This structure employs 3-angle parametrization for the orientation of the platform.
 * @see VarsQuatStruct
 */
struct RobotVars
{
  PlatformVars platform;         /**< variables of a generic 6DoF platform with angles. */
  std::vector<CableVars> cables; /**< vector of variables of a single cables in a CDPR. */

  arma::mat geom_jabobian;
  arma::mat anal_jabobian;
  arma::vec tension_vector;

  RobotVars() {}
  RobotVars(const size_t num_cables) : cables(std::vector<CableVars>(num_cables))
  {
    geom_jabobian.resize(num_cables, POSE_DIM);
    anal_jabobian.resize(num_cables, POSE_DIM);
    tension_vector.resize(num_cables);
  }
  RobotVars(const RotParametrization _angles_type) : platform(_angles_type) {}
  RobotVars(const size_t num_cables, const RotParametrization _angles_type)
    : platform(_angles_type), cables(std::vector<CableVars>(num_cables))
  {
    geom_jabobian.resize(num_cables, POSE_DIM);
    anal_jabobian.resize(num_cables, POSE_DIM);
    tension_vector.resize(num_cables);
  }
};

/**
 * @brief Structure collecting all variables related to a generic 6DoF CDPR.
 *
 * This structure employs quaternion parametrization for the orientation of the platform.
 * @see VarsStruct
 */
struct RobotVarsQuat
{
  PlatformQuatVars
    platform; /**< variables of a generic 6DoF platform with quaternions. */
  std::vector<CableVarsQuat>
    cables; /**< vector of variables of a single cables in a CDPR. */

  arma::mat geom_jabobian;
  arma::mat anal_jabobian;
  arma::vec tension_vector;

  RobotVarsQuat() {}
  RobotVarsQuat(const size_t num_cables) : cables(std::vector<CableVarsQuat>(num_cables))
  {
    geom_jabobian.resize(num_cables, POSE_DIM);
    anal_jabobian.resize(num_cables, POSE_QUAT_DIM);
    tension_vector.resize(num_cables);
  }
};

/**
 * @brief Structure collecting parameters related to a generic 6DoF platform.
 */
struct PlatformParams
{
  RotParametrization rot_parametrization; /**< rotation parametrization used. */
  grabnum::Matrix3d inertia_mat_G_loc;    /**< inertia matrix. */
  grabnum::Vector3d
    ext_torque_loc; /**< [Nm] external torque vector expressed in the local frame. */
  grabnum::Vector3d
    ext_force_loc; /**< [N] external force vector expressed in the local frame. */
  grabnum::Vector3d pos_PG_loc;  /**< [m] vector @f$^\mathcal{P}\mathbf{p}'_G@f$. */
  double mass = 0.0;             /**< [Kg] platform mass (@f$m@f$). */
  grabnum::Vector3d gravity_acc; /**< [m/s^2] gravity acceleration wrt global frame. */
};

/**
 * @brief Structure collecting parameters related to a single swivel pulley of a CDPR.
 */
struct PulleyParams
{
  grabnum::Vector3d pos_OD_glob; /**< [m] vector @f$\mathbf{d}_i@f$. */
  grabnum::Vector3d vers_i; /**< versor @f$\hat{\mathbf{i}}_i@f$ of _i-th_ swivel pulley
                               expressed in global frame. */
  grabnum::Vector3d vers_j; /**< versor @f$\hat{\mathbf{j}}_i@f$ of _i-th_ swivel pulley
                               expressed in global frame. */
  grabnum::Vector3d vers_k; /**< versor @f$\hat{\mathbf{k}}_i@f$ of _i-th_ swivel pulley
                               expressed in global frame. */
  double radius = 0.0;      /**< [m] _i-th_ swivel pulley radius length @f$r_i@f$ */
  uint32_t encoder_res =
    0; /**< _i-th_ pulley encoder resolution in counts per revolution. */

  /**
   * @brief PulleyAngleFactorRad
   * @return
   */
  inline double PulleyAngleFactorRad() const { return 2.0 * M_PI / encoder_res; }
  /**
   * @brief PulleyAngleFactorDeg
   * @return
   */
  inline double PulleyAngleFactorDeg() const { return 360.0 / encoder_res; }
};

/**
 * @brief Structure collecting parameters related to a single winch of a CDPR.
 */
struct WinchParams
{
  grabnum::Vector3d pos_PA_loc; /**< vector @f$\mathbf{a}_i'@f$. */
  double l0 = 0.0; /**< [m] length between @f$D_i@f$ and the exit point of the _i-th_
                      cable from the corresponding winch. */
  double drum_pitch    = 0.007; /**< [m] todo..*/
  double drum_diameter = 0.1;   /**< [m] todo..*/
  double gear_ratio    = 5.0;   /**< [m] todo..*/
  uint32_t motor_encoder_res =
    1048576; /**< motor encoder resolution in counts per revolution. */

  /**
   * @brief CountsToLengthFactor
   * @return
   */
  double CountsToLengthFactor() const
  {
    static double tau =
      sqrt(pow(M_PI * drum_diameter, 2.0) + pow(drum_pitch, 2.0) - drum_diameter * 0.1) /
      (motor_encoder_res * gear_ratio);
    return tau;
  }
};

/**
 * @brief Structure collecting parameters related to a single generic actuator of a CDPR.
 */
struct ActuatorParams
{
  bool active = false; /**< configuration of actuators. */
  PulleyParams pulley; /**< swivel pulley parameters set */
  WinchParams winch;   /**< winch parameters set */
};

/**
 * @brief Structure collecting all parameters related to a generic 6DoF CDPR.
 */
struct RobotParams
{
  PlatformParams platform; /**< parameters of a generic 6DoF platform. */
  std::vector<ActuatorParams>
    actuators; /**< vector of parameters of a single actuator in a CDPR. */

  std::vector<id_t> activeActuatorsId() const
  {
    std::vector<id_t> active_actuators_id;
    for (uint i = 0; i < actuators.size(); i++)
      if (actuators[i].active)
        active_actuators_id.push_back(i);
    return active_actuators_id;
  }

  size_t activeActuatorsNum() const
  {
    size_t active_actuators_counter = 0;
    for (uint i = 0; i < actuators.size(); i++)
      if (actuators[i].active)
        active_actuators_counter++;
    return active_actuators_counter;
  }
};

} // end namespace grabcdpr

#endif // GRABCOMMON_LIBCDPR_CDPR_TYPES_H
