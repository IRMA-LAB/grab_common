/**
 * @file kinematics.h
 * @author Edoardo Idà, Simone Comari
 * @date 01 Aug 2019
 * @brief File containing kinematics-related functions to be included in the GRAB CDPR
 * library.
 */

#ifndef GRABCOMMON_LIBCDPR_KINEMATICS_H
#define GRABCOMMON_LIBCDPR_KINEMATICS_H

#include "cdpr_types.h"
#include "matrix_utilities.h"
#include "rotations.h"

/**
 * @brief Namespace for CDPR-related utilities, such as kinematics and dynamics.
 */
namespace grabcdpr {
/** @defgroup ZeroOrderKinematics Zero Order Kinematics
 * This group collects all elements related to zero-order kinematics of a generic 6DoF
 * CDPR.
 * @{
 */

/**
 * @brief Update platform-related zero-order quantities (explicit).
 *
 * Given a new pose of the platform
 * @f$\mathbf{x} = (\mathbf{p}_P^T, \boldsymbol{\varepsilon}^T)^T@f$, the following
 * quantities are updated:
 * @f[
 * \mathbf{p}'_G = \mathbf{R}(\boldsymbol{\varepsilon}) ^\mathcal{P}\mathbf{p}'_G \\
 * \mathbf{p}_G = \mathbf{p}_P + \mathbf{p}'_G
 * @f]
 * being @f$^\mathcal{P}\mathbf{p}'_G@f$ a known parameter.
 * @param[in] position [m] Platform global position @f$\mathbf{p}_P@f$.
 * @param[in] orientation [rad] Platform global orientation expressed by angles
 * @f$\boldsymbol{\varepsilon}@f$.
 * @param[in] pos_PG_loc [m] Local CoG position @f$^\mathcal{P}\mathbf{p}'_G@f$.
 * @param[out] platform A pointer to the platform variables structure to be updated.
 * @note See @ref legend for symbols reference.
 */
void UpdatePlatformPose(const grabnum::Vector3d& position,
                        const grabnum::Vector3d& orientation,
                        const grabnum::Vector3d& pos_PG_loc, PlatformVars& platform);
/**
 * @brief Update platform-related zero-order quantities (explicit).
 *
 * Given a new pose of the platform
 * @f$\mathbf{x} = (\mathbf{p}_P^T, \boldsymbol{\varepsilon}_q^T)^T@f$, the following
 * quantities are updated:
 * @f[
 * \mathbf{p}'_G = \mathbf{R}(\boldsymbol{\varepsilon}_q) ^\mathcal{P}\mathbf{p}'_G \\
 * \mathbf{p}_G = \mathbf{p}_P + \mathbf{p}'_G
 * @f]
 * being @f$^\mathcal{P}\mathbf{p}'_G@f$ a known parameter.
 * @param[in] position [m] Platform global position @f$\mathbf{p}_P@f$.
 * @param[in] orientation [rad] Platform global orientation expressed by angles
 * @f$\boldsymbol{\varepsilon}@f$.
 * @param[in] pos_PG_loc [m] Local CoG position @f$^\mathcal{P}\mathbf{p}'_G@f$.
 * @param[out] platform A pointer to the platform variables structure to be updated.
 * @note See @ref legend for symbols reference.
 */
void UpdatePlatformPose(const grabnum::Vector3d& position,
                        const grabgeom::Quaternion& orientation,
                        const grabnum::Vector3d& pos_PG_loc, PlatformQuatVars& platform);
/**
 * @brief Update platform-related zero-order quantities (implicit).
 * @param[in] position [m] Platform global position @f$\mathbf{p}_P@f$.
 * @param[in] orientation [rad] Platform global orientation expressed by angles
 * @f$\boldsymbol{\varepsilon}@f$.
 * @param[in] params A pointer to the platform parameters structure.
 * @param[out] platform A pointer to the platform variables structure to be updated.
 * @see UpdatePlatformPose()
 */
void UpdatePlatformPose(const grabnum::Vector3d& position,
                        const grabnum::Vector3d& orientation,
                        const PlatformParams& params, PlatformVars& platform);
/**
 * @brief Update platform-related zero-order quantities (implicit).
 * @param[in] position [m] Platform global position @f$\mathbf{p}_P@f$.
 * @param[in] orientation Platform global orientation expressed by quaternion
 * @f$\boldsymbol{\varepsilon}_q@f$.
 * @param[in] params A pointer to the platform parameters structure.
 * @param[out] platform A pointer to the platform variables structure to be updated.
 * @see UpdatePlatformPose()
 */
void UpdatePlatformPose(const grabnum::Vector3d& position,
                        const grabgeom::Quaternion& orientation,
                        const PlatformParams& params, PlatformQuatVars& platform);

/**
 * @brief Update global position of point @f$A_i@f$ and relative segments.
 *
 * Given current platform variables @f$\mathbf{R}, \mathbf{p}_P@f$, the following
 * quantities are updated:
 * @f[
 * \mathbf{a}'_i = \mathbf{R}(\boldsymbol{\varepsilon}) ^\mathcal{P}\mathbf{a}'_i \\
 * \mathbf{a}_i = \mathbf{p}_P + \mathbf{a}'_i \\
 * \boldsymbol{\rho}^*_i = \mathbf{a}_i - \mathbf{d}_i
 * @f]
 * being @f$^\mathcal{P}\mathbf{a}'_i, \mathbf{d}_i@f$ known parameters.
 * @param[in] params A pointer to cable parameters.
 * @param[in] platform A pointer to the updated platform structure.
 * @param[out] cable A pointer to the cable structure including the positions to be
 * updated.
 * @note See @ref legend for symbols reference.
 */
void UpdatePosA(const ActuatorParams& params, const PlatformVars& platform,
                CableVars& cable);
/**
 * @brief Update global position of point @f$A_i@f$ and relative segments when
 * using quaternion to express platform orientation.
 * @param[in] params A pointer to cable parameters.
 * @param[in] platform A pointer to the updated platform structure.
 * @param[out] cable A pointer to the cable structure including the positions
 * to be updated.
 * @see UpdatePosA()
 */
void UpdatePosA(const ActuatorParams& params, const PlatformQuatVars& platform,
                CableVarsQuat& cable);

/**
 * @brief Calculate swivel pulley versors @f$\hat{\mathbf{u}}_i, \hat{\mathbf{w}}_i@f$.
 *
 * Given current swivel angle @f$\sigma_i@f$, the following versors are updated:
 * @f[
 * \hat{\mathbf{u}}_i = \hat{\mathbf{i}}_i \cos(\sigma_i) + \hat{\mathbf{j}}_i
 * \sin(\sigma_i) \\ \hat{\mathbf{w}}_i = -\hat{\mathbf{i}}_i \sin(\sigma_i) +
 * \hat{\mathbf{j}}_i \cos(\sigma_i)
 * @f]
 * being @f$\hat{\mathbf{i}}_i, \hat{\mathbf{j}}_i@f$ known parameters.
 * @param[in] params Swivel pulley parameters.
 * @param[in] swivel_ang [rad] Swivel angle @f$\sigma_i@f$.
 * @param[out] cable A pointer to the cable structure including the versors to be updated.
 * @note See @ref legend for symbols reference.
 * @note This expression results from the fact that, by definition,
 * @f$ \hat{\mathbf{u}}_i \perp \hat{\mathbf{w}}_i \perp \hat{\mathbf{k}}_i @f$.
 */
void CalcPulleyVersors(const PulleyParams& params, const double swivel_ang,
                       CableVarsBase& cable);
/**
 * @brief Calculate swivel pulley versors @f$\hat{\mathbf{u}}_i, \hat{\mathbf{w}}_i@f$.
 * @param[in] params Swivel pulley parameters.
 * @param[in,out] cable A pointer to the cable structure including the versors to be
 * updated.
 * @see CalcPulleyVersors()
 */
void CalcPulleyVersors(const PulleyParams& params, CableVarsBase& cable);

/**
 * @brief Calculate pulley swivel angle @f$\sigma_i@f$.
 *
 * Given current vector @f$\boldsymbol{\rho}^*_i@f$, the swivel angle is calculated as
 * follows:
 * @f[
 * \sigma_i = \arctan_2(\hat{\mathbf{i}}_i \cdot \boldsymbol{\rho}^*_i ,
 * \hat{\mathbf{j}}_i \cdot \boldsymbol{\rho}^*_i)
 * @f]
 * being @f$\hat{\mathbf{i}}_i, \hat{\mathbf{j}}_i@f$ known parameters.
 * @param[in] params Swivel pulley parameters.
 * @param[in] pos_DA_glob [m] Vector @f$\boldsymbol{\rho}^*_i@f$.
 * @return Swivel angle @f$\sigma_i@f$ in radians.
 * @note See @ref legend for symbols reference.
 * @note This expression results from the application of the 1<sup>st</sup> kinematic
 * constraint
 * @f[ \hat{\mathbf{w}}_i \cdot \boldsymbol{\rho}^*_i = 0 @f]
 */
double CalcSwivelAngle(const PulleyParams& params, const grabnum::Vector3d& pos_DA_glob);
/**
 * @brief Calculate pulley swivel angle @f$\sigma_i@f$.
 * @param[in] params Swivel pulley parameters.
 * @param[in] cable A pointer to the cable structure.
 * @return Swivel angle @f$\sigma_i@f$ in radians.
 * @see CalcSwivelAngle()
 */
double CalcSwivelAngle(const PulleyParams& params, const CableVarsBase& cable);

/**
 * @brief Calculate pulley tangent angle @f$\psi_i@f$.
 *
 * Given current vectors @f$\boldsymbol{\rho}^*_i, \hat{\mathbf{u}}_i@f$, the tangent
 * angle is calculated as follows:
 * @f[
 * \psi_i = 2 \arctan\left[
 *      \frac{\hat{\mathbf{k}}_i \cdot \boldsymbol{\rho}^*_i}{s_i} + \sqrt{1 -
 * \frac{2r_i}{s_i} + \left( \frac{\hat{\mathbf{k}}_i \cdot \boldsymbol{\rho}^*_i}{s_i}
 * \right)^2}\right]
 * @f]
 * being @f$\hat{\mathbf{k}}_i, r_i@f$ known parameters and @f$s_i = \hat{\mathbf{u}}_i
 * \cdot \boldsymbol{\rho}^*_i@f$.
 * @param[in] params Swivel pulley parameters.
 * @param[in] vers_u Versor @f$\hat{\mathbf{u}}_i@f$.
 * @param[in] pos_DA_glob [m] Vector @f$\boldsymbol{\rho}^*_i@f$.
 * @return Tangent angle @f$\psi_i@f$  in radians.
 * @note See @ref legend for symbols reference.
 * @note This expression results from the application of the 2<sup>nd</sup> kinematic
 * constraint
 * @f[ \hat{\mathbf{n}}_i \cdot \boldsymbol{\rho}_i = 0 @f]
 */
double CalcTangentAngle(const PulleyParams& params, const grabnum::Vector3d& vers_u,
                        const grabnum::Vector3d& pos_DA_glob);
/**
 * @brief Calculate pulley tangent angle @f$\psi_i@f$.
 * @param[in] params Swivel pulley parameters.
 * @param[in] cable A pointer to the cable structure.
 * @return Tangent angle @f$\psi_i@f$  in radians.
 * @see CalcTangentAngle()
 */
double CalcTangentAngle(const PulleyParams& params, const CableVarsBase& cable);

/**
 * @brief Calculate cable versors @f$\hat{\mathbf{n}}_i, \hat{\mathbf{t}}_i@f$ and
 * cable vector @f$\boldsymbol{\rho}_i@f$.
 *
 * Given current tangent angle @f$\psi_i@f$ and versor @f$\hat{\mathbf{u}}_i@f$, the
 * following versors are calculated:
 * @f[
 * \hat{\mathbf{n}}_i = \hat{\mathbf{u}}_i \cos(\psi_i) + \hat{\mathbf{k}}_i \sin(\psi_i)
 * \\ \hat{\mathbf{t}}_i = \hat{\mathbf{u}}_i \sin(\psi_i) - \hat{\mathbf{k}}_i
 * \cos(\psi_i)
 * @f]
 * being @f$\hat{\mathbf{k}}_i@f$ a known parameter.
 * Then, from @f$ \boldsymbol{\rho}^*_i@f$ cable vector is calculated as:
 * @f[
 * \boldsymbol{\rho}_i = \boldsymbol{\rho}^*_i - r_i (\hat{\mathbf{u}}_i +
 * \hat{\mathbf{n}}_i )
 * @f]
 * being @f$r_i@f$ a known parameter too.
 * @param[in] params Swivel pulley parameters.
 * @param[in] vers_u Versor @f$\hat{\mathbf{u}}_i@f$.
 * @param[in] pos_DA_glob [m] Vector @f$\boldsymbol{\rho}^*_i@f$.
 * @param[in] tan_ang [rad] Tangent angle @f$\psi_i@f$.
 * @param[out] cable A pointer to the cable structure including the variables to be
 * calculated.
 * @note See @ref legend for symbols reference.
 * @note This expressions result from the application of the 1<sup>st</sup> kinematic
 * constraint
 * @f[ \hat{\mathbf{w}}_i \cdot \boldsymbol{\rho}^*_i = 0 @f]
 * together with the fact that, by definition,
 * @f$ \hat{\mathbf{w}}_i \perp \hat{\mathbf{t}}_i \perp \hat{\mathbf{n}}_i @f$.
 */
void CalcCableVectors(const PulleyParams& params, const grabnum::Vector3d& vers_u,
                      const grabnum::Vector3d& pos_DA_glob, const double tan_ang,
                      CableVarsBase& cable);
/**
 * @brief Calculate cable versors @f$\hat{\mathbf{n}}_i, \hat{\mathbf{t}}_i@f$ and
 * cable vector @f$\boldsymbol{\rho}_i@f$.
 * @param[in] params Swivel pulley parameters.
 * @param[in,out] cable A pointer to the cable structure including the variables to be
 * calculated.
 * @see CalcCableVectors()
 */
void CalcCableVectors(const PulleyParams& params, CableVarsBase& cable);

/**
 * @brief Calculate cable length @f$l_i@f$.
 *
 * Given current cable vector @f$\boldsymbol{\rho}_i@f$, cable length is nothing but:
 * @f[
 * l_i = \|\boldsymbol{\rho}_i\|
 * @f]
 * @param[in] pos_BA_glob [m] Cable vector @f$\boldsymbol{\rho}_i@f$.
 * @return Cable length @f$l_i@f$ in meters.
 * @note See @ref legend for symbols reference.
 * @note This expression results from the application of the 3<sup>rd</sup> kinematic
 * constraint
 * @f[ \boldsymbol{\rho}_i \cdot \boldsymbol{\rho}_i = l_i^2 @f]
 */
double CalcCableLen(const grabnum::Vector3d& pos_BA_glob);

/**
 * @brief Calculate motor counts @f$q_i@f$.
 *
 * Given current cable length @f$l_i@f$ and tangent angle @f$\psi_i@f$,
 * motor position (e.g. counts) is calculated as follows:
 * @f[
 * q_i = (l_i + r_i(\pi - \psi_i)) / \tau_i
 * @f]
 * being @f$\tau_i, r_i@f$ known parameters.
 * @param[in] tau Transmittion ratio.
 * @param[in] cable_len [m] Cable length @f$l_i@f$.
 * @param[in] pulley_radius [m] Swivel pulley radius @f$r_i@f$.
 * @param[in] tan_ang [rad] Tangent angle @f$\psi_i@f$.
 * @return Motor counts @f$q_i@f$.
 * @note See @ref legend for symbols reference.
 * @note This expression results from the application of the 3<sup>rd</sup> kinematic
 * constraint
 * @f[ \boldsymbol{\rho}_i \cdot \boldsymbol{\rho}_i = l_i^2 @f]
 */
double CalcMotorCounts(const double tau, const double cable_len,
                       const double pulley_radius, const double tan_ang);
/**
 * @brief Calculate motor counts @f$q_i@f$.
 * @param[in,out] params Actuator parameters.
 * @param[in] cable A pointer to cable variables structure.
 * @return Motor counts @f$q_i@f$.
 * @see CalcCableLen()
 */
double CalcMotorCounts(ActuatorParams& params, const CableVarsBase& cable);

/**
 * @brief UpdateJacobiansRow
 * @param H_mat
 * @param cable
 */
void UpdateJacobiansRow(const grabnum::Matrix3d H_mat, CableVars& cable);
/**
 * @brief UpdateJacobiansRow
 * @param H_mat
 * @param cable
 */
void UpdateJacobiansRow(const grabnum::MatrixXd<3, 4> H_mat, CableVarsQuat& cable);

/**
 * @brief Update all zero-order variables of a single cable at once.
 * @param[in] platform A pointer to the updated platform structure.
 * @param[in] params A pointer to _i-th_ cable parameters.
 * @param[out] cable A pointer to _i-th_ cable variables structure to be updated.
 */
void UpdateCableZeroOrd(const ActuatorParams& params, const PlatformVars& platform,
                        CableVars& cable);
/**
 * @brief Update all zero-order variables of a single cable at once when using
 * quaternions.
 * @param[in] platform A pointer to the updated platform structure.
 * @param[in] params A pointer to _i-th_ cable parameters.
 * @param[out] cable A pointer to _i-th_ cable variables structure to be updated.
 */
void UpdateCableZeroOrd(const ActuatorParams& params, const PlatformQuatVars& platform,
                        CableVarsQuat& cable);

/**
 * @brief Update all robots zero-order variables at once (inverse kinematics problem).
 * @param[in] position [m] Platform global position @f$\mathbf{p}_P@f$.
 * @param[in] orientation [rad] Platform global orientation expressed by angles
 * @f$\boldsymbol{\varepsilon}@f$.
 * @param[in] params A pointer to the robot parameters structure.
 * @param[out] vars A pointer to the robot variables structure to be updated.
 */
void UpdateIK0(const grabnum::Vector3d& position, const grabnum::Vector3d& orientation,
               const RobotParams& params, RobotVars& vars);
/**
 * @brief Update all robots zero-order variables at once (inverse kinematics problem) when
 * usign quaternions.
 * @param[in] position [m] Platform global position @f$\mathbf{p}_P@f$.
 * @param[in] orientation [rad] Platform global orientation expressed by angles
 * @f$\boldsymbol{\varepsilon}@f$.
 * @param[in] params A pointer to the robot parameters structure.
 * @param[out] vars A pointer to the robot variables structure to be updated.
 */
void UpdateIK0(const grabnum::Vector3d& position, const grabgeom::Quaternion& orientation,
               const RobotParams& params, RobotVarsQuat& vars);

/** @} */ // end of ZeroOrderKinematics group

} // end namespace grabcdpr

#endif // GRABCOMMON_LIBCDPR_KINEMATICS_H
