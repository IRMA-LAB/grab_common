/**
 * @file cdpr_types.h
 * @author Edoardo Idà, Simone Comari
 * @date 02 Mar 2020
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
 * <tr><td> _World origin_  <td>@f$O@f$ <td> Origin of global frame @f$\mathcal{O}@f$.
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
 * <tr><td> _Number of cables_ <td> @f$N@f$ <td> - <td> Number of cables attached to the
 * platform.
 * <tr><td> _Cable length_ <td> @f$l_i@f$ <td> @f$\|A_i - B_i\| + \overparen{D_iB_i}@f$
 * <td> Length of _i-th_ cable from point @f$D_i@f$ to point @f$A_i@f$, including winding
 * around swivel pulley. <tr><td> _Motor counts_ <td>@f$q_i@f$ <td> - <td> Motor counts
 * (i.e. position) of _i-th_ motor. <tr><td> _Swivel angle_ <td>@f$\sigma_i@f$ <td> - <td>
 * _i-th_ pulley swivel angle, i.e. the angle between @f$\hat{\mathbf{x}}_i@f$ and
 * @f$\hat{\mathbf{u}}_i@f$. <tr><td> _Tangent angle_ <td>@f$\psi_i@f$ <td> - <td> _i-th_
 * pulley tangent angle, i.e. the angle between @f$\hat{\mathbf{u}}_i@f$ and
 * @f$\hat{\mathbf{n}}_i@f$.
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

#define POSE_DIM 6 /**< pose dim. of a body in space with 3-angle parametrization */
#define POSE_QUAT_DIM                                                                    \
  7 /**< pose dim. of a body in space with quaternion parametrization */

using namespace grabnum;

/**
 * @brief Convert a 3D row vector from GRAB format to armadillo format.
 * @param[in] vect A 3D row vector of double in GRAB format.
 * @param[in] copy If _True_ values are copied, otherwise the same piece of memory is
 * used. Be careful when doing so.
 * @return A 3D row vector of double in armadillo format.
 */
arma::rowvec3 toArmaVec(RowVectorXd<3> vect, bool copy = true);
/**
 * @brief Convert a 3D column vector from GRAB format to armadillo format.
 * @param[in] vect A 3D column vector of double in GRAB format.
 * @param[in] copy If _True_ values are copied, otherwise the same piece of memory is
 * used. Be careful when doing so.
 * @return A 3D column vector of double in armadillo format.
 */
arma::vec toArmaVec(Vector3d vect, bool copy = true);
/**
 * @brief Convert a 6D column vector from GRAB format to armadillo format.
 * @param[in] vect A 6D column vector of double in GRAB format.
 * @param[in] copy If _True_ values are copied, otherwise the same piece of memory is
 * used. Be careful when doing so.
 * @return A 6D column vector of double in armadillo format.
 */
arma::vec toArmaVec(
  VectorXd<POSE_DIM> vect,
  bool copy =
    true); /**
            * @brief Convert a 7D column vector from GRAB format to armadillo format.
            * @param[in] vect A 7D column vector of double in GRAB format.
            * @param[in] copy If _True_ values are copied, otherwise the same piece of
            * memory is used. Be careful when doing so.
            * @return A 7D column vector of double in armadillo format.
            */
arma::vec toArmaVec(VectorXd<POSE_QUAT_DIM> vect, bool copy = true);

/**
 * @brief Convert a 3D matrix from GRAB format to armadillo format.
 * @param[in] mat A 3D matrix of double in GRAB format.
 * @param[in] copy If _True_ values are copied, otherwise the same piece of memory is
 * used. Be careful when doing so.
 * @return A 3D matrix of double in armadillo format.
 */
arma::mat toArmaMat(Matrix3d mat, bool copy = true);
/**
 * @brief Convert a 6D matrix from GRAB format to armadillo format.
 * @param[in] mat A 6D matrix of double in GRAB format.
 * @param[in] copy If _True_ values are copied, otherwise the same piece of memory is
 * used. Be careful when doing so.
 * @return A 6D matrix of double in armadillo format.
 */
arma::mat toArmaMat(Matrix6d mat, bool copy = true);

/**
 * @brief Convert a 3D column vector from armadillo format to GRAB format.
 * @param[in] vect A 3D column vector of double in armadillo format.
 * @return A 3D column vector of double in GRAB format.
 */
grabnum::Vector3d fromArmaVec3(const arma::vec3& vect);

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

//------ Parameters Structs ----------------------------------------------------------//

/**
 * @brief Structure collecting parameters related to a generic 6DoF platform.
 */
struct PlatformParams
{
  RotParametrization rot_parametrization; /**< rotation parametrization used. */
  grabnum::Matrix3d inertia_mat_G_loc;    /**< inertia matrix. */
  grabnum::Vector3d
    ext_force_loc; /**< [N] external force vector expressed in the local frame. */
  grabnum::Vector3d
    ext_torque_loc; /**< [Nm] external torque vector expressed in the local frame. */
  grabnum::Vector3d
    ext_force_glob; /**< [N] external force vector expressed in the global frame. */
  grabnum::Vector3d
    ext_torque_glob; /**< [Nm] external torque vector expressed in the global frame. */
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
  grabnum::Vector3d vers_i;  /**< versor @f$\hat{\mathbf{i}}_i@f$ of _i-th_ swivel pulley
                                expressed in global frame. */
  grabnum::Vector3d vers_j;  /**< versor @f$\hat{\mathbf{j}}_i@f$ of _i-th_ swivel pulley
                                expressed in global frame. */
  grabnum::Vector3d vers_k;  /**< versor @f$\hat{\mathbf{k}}_i@f$ of _i-th_ swivel pulley
                                expressed in global frame. */
  double radius = 0.0;       /**< [m] _i-th_ swivel pulley radius length @f$r_i@f$ */
  double transmission_ratio; /**< _i-th_ pulley counts-to-radians transmition ratio. */

  /**
   * @brief Returns the swivel pulley encoder counts-to-radians factor.
   * @return Swivel pulley encoder counts-to-radians factor.
   * @see pulleyAngleFactorDeg()
   */
  inline double pulleyAngleFactorRad() const { return transmission_ratio; }
  /**
   * @brief Returns the swivel pulley encoder counts-to-degrees factor.
   * @return Swivel pulley encoder counts-to-degrees factor.
   * @see pulleyAngleFactorRad()
   */
  inline double pulleyAngleFactorDeg() const { return transmission_ratio * 180. / M_PI; }

  /**
   * @brief Fix versors to ensure orthogonality and being unit vectors.
   */
  void orthogonalizeVersors()
  {
    vers_k = vers_k / grabnum::Norm(vers_k);
    vers_i = grabnum::Cross(vers_j, vers_k);
    vers_i = vers_i / grabnum::Norm(vers_i);
    vers_j = grabnum::Cross(vers_k, vers_i);
    vers_j = vers_j / grabnum::Norm(vers_j);
  }
};

/**
 * @brief Structure collecting parameters related to a single winch of a CDPR.
 */
struct WinchParams
{
  grabnum::Vector3d pos_PA_loc; /**< vector @f$\mathbf{a}_i'@f$. */
  double l0 = 0.0; /**< [m] length between @f$D_i@f$ and the exit point of the _i-th_
                      cable from the corresponding winch. */
  double transmission_ratio; /**< counts-to-meters transmission ratio. */
  double tension_bias =
    0.0; /**< [N] tension bias in linear transformation loadcell value --> tension. */
  double tension_gain =
    1.; /**< [N] tension gain in linear transformation loadcell value --> tension. */
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
  arma::uvec6
    controlled_vars_mask; /**< actuation binary mask (1=actuated, 0=unactuated).*/

  /**
   * @brief Returns IDs of active actuators in the daisy chain.
   * @return IDs of active actuators in the daisy chain.
   * @see activeActuatorsNum()
   */
  std::vector<id_t> activeActuatorsId() const;

  /**
   * @brief Returns the number of active actuators in the daisy chain.
   * @return The number of active actuators in the daisy chain.
   * @see activeActuatorsId()
   */
  size_t activeActuatorsNum() const;

  /**
   * @brief Get a subset of original parameters without inactive components.
   * @return A subset of original CDPR parameters.
   */
  RobotParams getOnlyActiveComponents() const;

  /**
   * @brief Remove parameters of inactive actuators components.
   */
  void removeInactiveComponents();
};

//------ Variables Structs -----------------------------------------------------------//

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
  grabnum::Vector3d linear_vel; /**< [_m/s_] vector @f$\dot{\mathbf{p}}_P@f$. */

  grabnum::Vector3d angular_vel; /**< vector @f$\boldsymbol\omega@f$. */

  grabnum::Vector6d velocity; /**< vector @f$\mathbf{v}@f$. */

  grabnum::Vector3d vel_OG_glob; /**< [_m/s_] vector @f$\dot{\mathbf{p}}_G@f$. */
  /** @} */                      // end of FirstOrderKinematics group

  /** @addtogroup SecondOrderKinematics
   * @{
   */
  grabnum::Vector3d
    linear_acc; /**< [_m/s<sup>2</sup>_] vector @f$\ddot{\mathbf{p}}_P@f$. */

  grabnum::Vector3d angular_acc; /**< vector @f$\boldsymbol\alpha@f$.*/

  grabnum::Vector6d acceleration; /**< vector @f$\mathbf{v}@f$. */

  grabnum::Vector3d
    acc_OG_glob; /**< [_m/s<sup>2</sup>_] vector @f$\ddot{\mathbf{p}}_G@f$.*/
  /** @} */      // end of SecondOrderKinematics group

  /** @addtogroup Dynamics
   * @{
   */
  grabnum::Matrix3d
    inertia_mat_glob; /**< [kg m^2] inertia matrix expressed in the global frame. */
  grabnum::Matrix6d mass_mat_glob; /**< mass matrix expressed in the global frame. */
  grabnum::Vector6d ext_load; /**< vector containing components of external forces and
                                 moments, expressed in the global frame. */
  grabnum::Vector6d dyn_load; /**< vector containing components of dynamic forces and
                                 moments, expressed in the global frame. */
  grabnum::Vector6d
    total_load; /**< vector containing components of total external and dynamic forces and
                 moments, expressed in the global frame. */
  /** @} */     // end of Dynamics group
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

  /** @addtogroup Dynamics
   * @{
   */
  grabnum::MatrixXd<POSE_DIM, POSE_DIM>
    mass_mat_glob_ss; /**< mass matrix projected onto the state-space. */
  grabnum::VectorXd<POSE_DIM> ext_load_ss; /**< vector containing component of external
                                    forces and moments, projected onto the state-space. */
  grabnum::VectorXd<POSE_DIM> dyn_load_ss; /**< vector containing components of dynamic
                                 forces and moments, projected onto the state-space. */
  grabnum::VectorXd<POSE_DIM>
    total_load_ss; /**< vector containing components of total external and dynamic forces
                 and moments, projected onto the state-space. */
  /** @} */        // end of Dynamics group

  /**
   * @brief Constructor to explicitly declare rotation parametrization desired only.
   * @param[in] _angles_type Desired rotation parametrization.
   */
  PlatformVars(const RotParametrization _angles_type = TILT_TORSION);
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
               const RotParametrization _angles_type = TILT_TORSION);
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
               const RotParametrization _angles_type = TILT_TORSION);
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
               const RotParametrization _angles_type = TILT_TORSION);

  /**
   * @brief Set platform pose.
   * @param[in] _pose Platform pose, including both position and orientation.
   * @ingroup ZeroOrderKinematics
   * @see UpdatePose()
   */
  void updatePose(const grabnum::VectorXd<POSE_DIM>& _pose);
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
  void updatePose(const grabnum::Vector3d& _position,
                  const grabnum::Vector3d& _orientation);

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
  void updateVel(const grabnum::Vector3d& _velocity,
                 const grabnum::Vector3d& _orientation_dot,
                 const grabnum::Vector3d& _orientation);
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
  void updateVel(const grabnum::Vector3d& _velocity,
                 const grabnum::Vector3d& _orientation_dot);

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
  void updateAcc(const grabnum::Vector3d& _acceleration,
                 const grabnum::Vector3d& _orientation_ddot,
                 const grabnum::Vector3d& _orientation_dot,
                 const grabnum::Vector3d& _orientation, const grabnum::Matrix3d& _h_mat);
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
  void updateAcc(const grabnum::Vector3d& _acceleration,
                 const grabnum::Vector3d& _orientation_ddot);

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
  void update(const grabnum::Vector3d& _position, const grabnum::Vector3d& _velocity,
              const grabnum::Vector3d& _acceleration,
              const grabnum::Vector3d& _orientation,
              const grabnum::Vector3d& _orientation_dot,
              const grabnum::Vector3d& _orientation_ddot);
};

/**
 * @brief Structure collecting all variables related to non-minimal orientation
 * parametrization of a generic 6DoF platform, i.e. with quaternions.
 * @see PlatformVarsQuatStruct
 * @note See @ref legend for symbols reference.
 */
struct PlatformVarsQuat: PlatformVarsBase
{
  /** @addtogroup ZeroOrderKinematics
   * @{
   */
  grabnum::Vector4d orientation; /**< vector @f$\boldsymbol{\varepsilon}_q@f$. */

  grabnum::MatrixXd<3, 4> h_mat;  /**< matrix @f$\mathbf{H}_q@f$. */
  grabnum::MatrixXd<3, 4> dh_mat; /**< matrix @f$\dot{\mathbf{H}}_q@f$. */

  grabnum::VectorXd<POSE_QUAT_DIM> pose; /**< vector @f$\mathbf{x}_q@f$. */
  /** @} */                              // end of ZeroOrderKinematics group

  /** @addtogroup FirstOrderKinematics
   * @{
   */
  grabnum::Vector4d
    orientation_dot; /**< quaternion @f$\dot{\boldsymbol{\varepsilon}}_q@f$. */
  /** @} */          // end of FirstOrderKinematics group

  /** @addtogroup SecondOrderKinematics
   * @{
   */
  grabnum::Vector4d
    orientation_ddot; /**< quaternion @f$\ddot{\boldsymbol{\varepsilon}}_q@f$. */
  /** @} */           // end of SecondOrderKinematics group

  /** @addtogroup Dynamics
   * @{
   */
  grabnum::MatrixXd<POSE_QUAT_DIM, POSE_QUAT_DIM>
    mass_mat_glob_ss; /**< mass matrix projected onto the state-space. */
  grabnum::VectorXd<POSE_QUAT_DIM>
    ext_load_ss; /**< vector containing component of external
     forces and moments, projected onto the state-space. */
  grabnum::VectorXd<POSE_QUAT_DIM>
    dyn_load_ss; /**< vector containing components of dynamic
  forces and moments, projected onto the state-space. */
  grabnum::VectorXd<POSE_QUAT_DIM>
    total_load_ss; /**< vector containing components of total external and dynamic forces
                 and moments, projected onto the state-space. */
  /** @} */        // end of Dynamics group

  /**
   * @brief PlatformQuatVars default constructor.
   */
  PlatformVarsQuat() {}
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
  PlatformVarsQuat(const grabnum::Vector3d& _position, const grabnum::Vector3d& _velocity,
                   const grabnum::Vector3d& _acceleration,
                   const grabgeom::Quaternion& _orientation,
                   const grabgeom::Quaternion& _orientation_dot,
                   const grabgeom::Quaternion& _orientation_ddot);

  /**
   * @brief Set platform pose.
   * @param[in] pose Platform pose, including both position and orientation expressed in
   * quaternion.
   * @see UpdatePose()
   */
  void updatePose(const grabnum::VectorXd<POSE_QUAT_DIM> pose);

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
  void updatePose(const grabnum::Vector3d& _position,
                  const grabgeom::Quaternion& _orientation);

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
  void updateVel(const grabnum::Vector3d& _velocity,
                 const grabgeom::Quaternion& _orientation_dot,
                 const grabgeom::Quaternion& _orientation);
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
  void updateVel(const grabnum::Vector3d& _velocity,
                 const grabgeom::Quaternion& _orientation_dot);

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
  void updateAcc(const grabnum::Vector3d& _acceleration,
                 const grabgeom::Quaternion& _orientation_ddot,
                 const grabgeom::Quaternion& _orientation_dot,
                 const grabnum::MatrixXd<3, 4>& _h_mat);
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
  void updateAcc(const grabnum::Vector3d& _acceleration,
                 const grabgeom::Quaternion& _orientation_ddot);

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
  void update(const grabnum::Vector3d& _position, const grabnum::Vector3d& _velocity,
              const grabnum::Vector3d& _acceleration,
              const grabgeom::Quaternion& _orientation,
              const grabgeom::Quaternion& _orientation_dot,
              const grabgeom::Quaternion& _orientation_ddot);
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

  grabnum::RowVectorXd<POSE_DIM>
    geom_jacob_d_row; /**< _i-th_ row of geometric jacobian derivatoves. */
  /** @} */           // end of FirstOrderKinematics group

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

/**
 * @brief A specialized version of CableVarsBase when using a minimal orientation
 * parametrization, i.e. with 3 angles.
 */
struct CableVars: CableVarsBase
{
  /** @addtogroup ZeroOrderKinematics
   * @{
   */
  grabnum::RowVectorXd<POSE_DIM> anal_jacob_row; /**< _i-th_ row of analitic jacobian. */
  /** @} */                                      // end of ZeroOrderKinematics group

  /** @addtogroup FirstOrderKinematics
   * @{
   */
  grabnum::RowVectorXd<POSE_DIM>
    anal_jacob_d_row; /**< _i-th_ row of analitic jacobian derivatives. */
  /** @} */           // end of FirstOrderKinematics group
};

/**
 * @brief A specialized version of CableVarsBase when using a quaternions for orientation
 * parametrization.
 */
struct CableVarsQuat: CableVarsBase
{
  /** @addtogroup ZeroOrderKinematics
   * @{
   */
  grabnum::RowVectorXd<POSE_QUAT_DIM>
    anal_jacob_row; /**< _i-th_ row of analitic jacobian. */
  /** @} */         // end of ZeroOrderKinematics group

  /** @addtogroup FirstOrderKinematics
   * @{
   */
  grabnum::RowVectorXd<POSE_QUAT_DIM>
    anal_jacob_d_row; /**< _i-th_ row of analitic jacobian derivatives. */
  /** @} */           // end of FirstOrderKinematics group
};

/**
 * @brief Structure collecting variables related to a generic 6DoF CDPR that are
 * independent on its orientation parametrization.
 */
struct RobotVarsBase
{
  /** @addtogroup ZeroOrderKinematics
   * @{
   */
  arma::mat geom_jacobian; /**< geometric jacobian. */
  arma::mat anal_jacobian; /**< analytical jacobian. */
  /** @} */                // end of ZeroOrderKinematics group

  /** @addtogroup FirstOrderKinematics
   * @{
   */
  arma::mat geom_jacobian_d; /**< geometric jacobian first derivative. */
  arma::mat anal_jacobian_d; /**< analytical jacobian first derivative. */
  /** @} */                  // end of FirstOrderKinematics group

  /** @addtogroup Dynamics
   * @{
   */
  arma::vec tension_vector; /**< [N] tensions vector, collecting tension on each cable.*/
  /** @} */                 // end of Dynamics group
};

/**
 * @brief Structure collecting all variables related to a generic 6DoF CDPR.
 *
 * This structure employs 3-angle parametrization for the orientation of the platform.
 * @see RobotVarsQuat
 */
struct RobotVars: RobotVarsBase
{
  PlatformVars platform;         /**< variables of a generic 6DoF platform with angles. */
  std::vector<CableVars> cables; /**< vector of variables of a single cables in a CDPR. */

  /**
   * @brief Default empty contructor.
   */
  RobotVars() {}
  /**
   * @brief Constructor to define the rotation parametrization.
   * @param[in] _angles_type Desired rotation parametrization.
   */
  RobotVars(const RotParametrization _angles_type) : platform(_angles_type) {}
  /**
   * @brief Constructor to predefine number of cables attached to the platform.
   * @param[in] num_cables Number of cables attached to the platform.
   */
  RobotVars(const size_t num_cables);
  /**
   * @brief Constructor to predefine number of cables attached to the platform and
   * rotation parametrization.
   * @param[in] num_cables Number of cables attached to the platform.
   * @param[in] _angles_type Desired rotation parametrization.
   */
  RobotVars(const size_t num_cables, const RotParametrization _angles_type);

  /**
   * @brief Clear and resize jacobians in case number of cables is changed.
   */
  void resize();

  /**
   * @brief Update all jacobians according to current CDPR status.
   * @note If a dimension mismatch is detected, resize() is invoked before updating
   * values.
   * @see resize()
   */
  void updateJacobians();
};

/**
 * @brief Structure collecting all variables related to a generic 6DoF CDPR.
 *
 * This structure employs quaternion parametrization for the orientation of the platform.
 * @see RobotVars
 */
struct RobotVarsQuat: RobotVarsBase
{
  PlatformVarsQuat
    platform; /**< variables of a generic 6DoF platform with quaternions. */
  std::vector<CableVarsQuat>
    cables; /**< vector of variables of a single cables in a CDPR. */

  /**
   * @brief Default empty contructor.
   */
  RobotVarsQuat() {}
  /**
   * @brief Constructor to predefine number of cables attached to the platform.
   * @param[in] num_cables Number of cables attached to the platform.
   */
  RobotVarsQuat(const size_t num_cables);

  /**
   * @brief Clear and resize jacobians in case number of cables is changed.
   */
  void resize();

  /**
   * @brief Update all jacobians according to current CDPR status.
   * @note If a dimension mismatch is detected, resize() is invoked before updating
   * values.
   * @see resize()
   */
  void updateJacobians();
};

} // end namespace grabcdpr

#endif // GRABCOMMON_LIBCDPR_CDPR_TYPES_H
