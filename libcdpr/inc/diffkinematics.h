/**
 * @file diffkinematics.h
 * @author Edoardo Idà, Simone Comari
 * @date 07 Feb 2020
 * @brief File containing first-order differential kinematics-related functions to be
 * included in the GRAB CDPR library.
 */

#ifndef GRABCOMMON_LIBCDPR_DIFFKINEMATICS_H
#define GRABCOMMON_LIBCDPR_DIFFKINEMATICS_H

#include "matrix_utilities.h"
#include "rotations.h"

#include "cdpr_types.h"
#include "kinematics.h"

/**
 * @brief Namespace for CDPR-related utilities, such as kinematics and dynamics.
 */
namespace grabcdpr {

/** @defgroup FirstOrderKinematics First Order Kinematics
 * This group collects all elements related to first-order kinematics of a generic 6DoF
 * CDPR.
 * @{
 */

/**
 * @brief Update platform-related first-order quantities.
 *
 * Given a new velocity of the platform
 * @f$\dot{\mathbf{x}} = (\dot{\mathbf{p}}_P^T, \dot{\boldsymbol{\varepsilon}}^T)^T@f$,
 * the following quantities are updated:
 * @f[
 * \boldsymbol{\omega} = \mathbf{H}(\boldsymbol{\varepsilon})
 * \dot{\boldsymbol{\varepsilon}} \\
 * \dot{\mathbf{p}}_G = \dot{\mathbf{p}}_P + \boldsymbol{\omega} \times \mathbf{p}'_G =
 * \dot{\mathbf{p}}_P + \tilde{\boldsymbol{\Omega}}\mathbf{p}'_G
 * @f]
 * being @f$\tilde{\boldsymbol{\Omega}}@f$ the skew-symmetric matrix of
 * @f$\boldsymbol{\omega}@f$.
 * @param[in] velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}_P@f$.
 * @param[in] orientation_dot Platform orientation time-derivative
 * @f$\dot{\boldsymbol{\varepsilon}}@f$.
 * @param[in] pos_PG_glob [m] Global CoG position @f$\mathbf{p}'_G@f$.
 * @param[out] platform The platform structure including vars to be updated.
 * @note See @ref legend for symbols reference.
 */
void updatePlatformVel(const Vector3d& velocity, const Vector3d& orientation_dot,
                       const Vector3d& pos_PG_glob, PlatformVars& platform);
/**
 * @brief Update platform-related first-order quantities.
 *
 * Given a new velocity of the platform
 * @f$\dot{\mathbf{x}} = (\dot{\mathbf{p}}_P^T, \dot{\boldsymbol{\varepsilon}}^T)^T@f$,
 * the following quantities are updated:
 * @f[
 * \boldsymbol{\omega} = \mathbf{H}(\boldsymbol{\varepsilon})
 * \dot{\boldsymbol{\varepsilon}} \\
 * \dot{\mathbf{p}}_G = \dot{\mathbf{p}}_P + \boldsymbol{\omega} \times \mathbf{p}'_G =
 * \dot{\mathbf{p}}_P + \tilde{\boldsymbol{\Omega}}\mathbf{p}'_G
 * @f]
 * being @f$\tilde{\boldsymbol{\Omega}}@f$ the skew-symmetric matrix of
 * @f$\boldsymbol{\omega}@f$.
 * @param[in] velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}_P@f$.
 * @param[in] orientation_dot Platform orientation time-derivative in quaternion form
 * @f$\dot{\boldsymbol{\varepsilon}}_q@f$.
 * @param[in] pos_PG_glob [m] Global CoG position @f$\mathbf{p}'_G@f$.
 * @param[out] platform The platform structure including vars to be updated.
 * @note See @ref legend for symbols reference.
 */
void updatePlatformVel(const Vector3d& velocity,
                       const grabgeom::Quaternion& orientation_dot,
                       const Vector3d& pos_PG_glob, PlatformVarsQuat& platform);
/**
 * @brief Update platform-related first-order quantities.
 * @param[in] velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}_P@f$.
 * @param[in] orientation_dot Platform orientation time-derivative
 * @f$\dot{\boldsymbol{\varepsilon}}@f$.
 * @param[in,out] platform A reference to the platform structure including vars to be
 * updated.
 * @see UpdatePlatformVel()
 */
void updatePlatformVel(const Vector3d& velocity, const Vector3d& orientation_dot,
                       PlatformVars& platform);
/**
 * @brief Update platform-related first-order quantities.
 * @param[in] velocity [m/s] Platform global linear velocity @f$\dot{\mathbf{p}}_P@f$.
 * @param[in] orientation_dot Platform orientation time-derivative in quaternion form
 * @f$\dot{\boldsymbol{\varepsilon}}_q@f$.
 * @param[in,out] platform A reference to the platform structure including vars to be
 * updated.
 * @see UpdatePlatformVel()
 */
void updatePlatformVel(const Vector3d& velocity,
                       const grabgeom::Quaternion& orientation_dot,
                       PlatformVarsQuat& platform);

/**
 * @brief Calculate global velocity of point @f$A_i@f$ and relative segments.
 *
 * Given current @f$\mathbf{a}'_i@f$, the following vector is updated:
 * @f[
 * \dot{\mathbf{a}}_i = \dot{\mathbf{p}}_P + \tilde{\boldsymbol{\Omega}}\mathbf{a}'_i
 * @f]
 * being @f$\tilde{\boldsymbol{\Omega}}@f$ the skew-symmetric matrix of
 * @f$\boldsymbol{\omega}@f$.
 * @param[in] pos_PA_glob [m] Vector @f$\mathbf{a}'_i@f$.
 * @param[in] platform The updated platform variables structure.
 * @return Global velocity of point @f$A_i@f$.
 * @note See @ref legend for symbols reference.
 */
Vector3d calcVelA(const Vector3d& pos_PA_glob, const PlatformVarsBase& platform);

/**
 * @brief Update global velocity of point @f$A_i@f$ and relative segments.
 * @param[in] platform The updated platform variables structure.
 * @param[in,out] cable The cable variables structure including the
 * velocities to be updated.
 * @see UpdateVelA()
 */
void updateVelA(const PlatformVarsBase& platform, CableVarsBase& cable);

/**
 * @brief Calculate swivel pulley versors speed
 * @f$\dot{\hat{\mathbf{u}}}_i, \dot{\hat{\mathbf{w}}}_i@f$.
 *
 * Given current @f$\hat{\mathbf{w}}_i, \hat{\mathbf{u}}_i, \dot{\sigma}_i@f$, the
 * following versors are updated:
 * @f[
 * \dot{\hat{\mathbf{u}}}_i = \hat{\mathbf{w}}_i \dot{\sigma}_i \\
 * \dot{\hat{\mathbf{w}}}_i = -\hat{\mathbf{u}}_i \dot{\sigma}_i
 * @f]
 * @param[in] vers_u Versor @f$\hat{\mathbf{u}}_i@f$.
 * @param[in] vers_w Versor @f$\hat{\mathbf{w}}_i@f$.
 * @param[in] swivel_ang_vel [rad/s] Swivel angle speed @f$\dot{\sigma}@f$.
 * @param[out] vers_u_dot A reference to versor speed @f$\dot{\hat{\mathbf{u}}}_i@f$.
 * @param[out] vers_w_dot A reference to versor speed @f$\dot{\hat{\mathbf{w}}}_i@f$.
 * @note See @ref legend for symbols reference.
 * @see updatePulleyVersorsDot()
 */
void calcPulleyVersorsDot(const Vector3d& vers_u, const Vector3d& vers_w,
                          const double swivel_ang_vel, Vector3d& vers_u_dot,
                          Vector3d& vers_w_dot);
/**
 * @brief Update swivel pulley versors speed.
 * @f$\dot{\hat{\mathbf{u}}}_i, \dot{\hat{\mathbf{w}}}_i@f$.
 * @param[in,out] cable A reference to the cable structure including the updated versors
 * and their derivatives to be updated.
 * @see CalcPulleyVersorsDot()
 */
void updatePulleyVersorsDot(CableVarsBase& cable);

/**
 * @brief Calculate pulley swivel angle speed @f$\dot{\sigma}_i@f$.
 *
 * Given current
 * @f$\hat{\mathbf{u}}_i, \hat{\mathbf{w}}_i, \dot{\mathbf{a}}_i,
 * \boldsymbol{\rho}^*_i@f$, the swivel angle speed is calculated as:
 * @f[
 * \dot{\sigma}_i = \frac{\hat{\mathbf{w}}_i \cdot \dot{\mathbf{a}}_i}
 *        {\hat{\mathbf{u}}_i \cdot \boldsymbol{\rho}^*_i}
 * @f]
 * @param[in] vers_u Versor @f$\hat{\mathbf{u}}_i@f$.
 * @param[in] vers_w Versor @f$\hat{\mathbf{w}}_i@f$.
 * @param[in] vel_OA_glob [m/s] Vector @f$\dot{\mathbf{a}}_i@f$.
 * @param[in] pos_DA_glob [m] Vector @f$\boldsymbol{\rho}^*_i@f$.
 * @return Swivel angle speed @f$\dot{\sigma}_i@f$ in _rad/s_.
 * @note See @ref legend for symbols reference.
 */
double calcSwivelAngSpeed(const Vector3d& vers_u, const Vector3d& vers_w,
                          const Vector3d& vel_OA_glob, const Vector3d& pos_DA_glob);

/**
 * @brief Calculate pulley swivel angle speed @f$\dot{\sigma}_i@f$.
 * @param[in] cable A reference to the cable variables structure.
 * @return Swivel angle speed @f$\dot{\sigma}_i@f$ in _rad/s_.
 * @see CalcSwivelAngSpeed()
 */
void updateSwivelAngSpeed(CableVarsBase& cable);

/**
 * @brief Calculate pulley tangent angle speed @f$\dot{\psi}_i@f$.
 *
 * Given current
 * @f$\hat{\mathbf{n}}_i, \dot{\mathbf{a}}_i, \boldsymbol{\rho}_i@f$, the
 * tangent angle speed is calculated as:
 * @f[
 * \dot{\psi}_i = \frac{\hat{\mathbf{n}}_i \cdot \dot{\mathbf{a}}_i}
 *        {l_i}
 * @f]
 * @param[in] vers_n Versor @f$\hat{\mathbf{n}}_i@f$.
 * @param[in] vel_OA_glob [m/s] Vector @f$\dot{\mathbf{a}}_i@f$.
 * @param[in] pos_BA_glob [m] Vector @f$\boldsymbol{\rho}_i@f$.
 * @return Tangent angle speed @f$\dot{\psi}_i@f$ in _rad/s_.
 * @note See @ref legend for symbols reference.
 */
double calcTangAngSpeed(const Vector3d& vers_n, const Vector3d& vel_OA_glob,
                        const Vector3d& pos_BA_glob);
/**
 * @brief Calculate pulley tangent angle speed @f$\dot{\psi}_i@f$.
 * @param[in] cable A reference to the cable variables structure.
 * @return Tangent angle speed @f$\dot{\psi}_i@f$ in _rad/s_.
 * @see CalcTangAngSpeed()
 */
void updateTangAngSpeed(CableVarsBase& cable);

/**
 * @brief Calculate cable versors @f$\dot{\hat{\mathbf{n}}}_i,
 * \dot{\hat{\mathbf{t}}}_i@f$.
 *
 * Given current
 * @f$\hat{\mathbf{w}}_i, \hat{\mathbf{n}}_i, \hat{\mathbf{t}}_i, \psi_i,
 * \dot{\psi}_i, \dot{\sigma}_i@f$, the cable versors speed is calculated as:
 * @f[
 * \dot{\hat{\mathbf{n}}}_i = \hat{\mathbf{w}}_i \cos(\psi_i) \dot{\sigma}_i -
 *      \hat{\mathbf{t}}_i \dot{\psi}_i \\
 * \dot{\hat{\mathbf{t}}}_i = \hat{\mathbf{w}}_i \sin(\psi_i) \dot{\sigma}_i +
 *      \hat{\mathbf{n}}_i \dot{\psi}_i
 * @f]
 * @param[in] pulley_radius Swivel pulley's radius in meters.
 * @param[in] vers_w Versor @f$\hat{\mathbf{w}}_i@f$.
 * @param[in] vers_n Versor @f$\hat{\mathbf{n}}_i@f$.
 * @param[in] vers_t Versor @f$\hat{\mathbf{t}}_i@f$.
 * @param[in] vel_OA_glob Vector
 * @param[in] tan_ang [rad] Tangent angle @f$\psi_i@f$.
 * @param[in] tan_ang_vel [rad/s] Tangent angle speed @f$\dot{\psi}_i@f$.
 * @param[in] swivel_ang_vel [rad/s] Swivel angle speed in @f$\dot{\sigma}_i@f$.
 * @param[out] vers_n_dot A reference to versor @f$\dot{\hat{\mathbf{n}}}_i@f$.
 * @param[out] vers_t_dot A reference to versor @f$\dot{\hat{\mathbf{t}}}_i@f$.
 * @param[out] vel_BA_glob A reference to vector @f$\dot{\boldsymbol{\rho}}_i@f$.
 * @note See @ref legend for symbols reference.
 * @see updateCableVersorsDot()
 */
void calcCableVersorsDot(const double pulley_radius, const Vector3d& vers_w,
                         const Vector3d& vers_n, const Vector3d& vers_t,
                         const Vector3d& vel_OA_glob,
                         const double tan_ang, const double tan_ang_vel,
                         const double swivel_ang_vel,
                         Vector3d& vers_n_dot, Vector3d& vers_t_dot,
                         Vector3d& vel_BA_glob);
/**
 * @brief Update cable versors @f$\dot{\hat{\mathbf{n}}}_i,
 * \dot{\hat{\mathbf{t}}}_i@f$.
 * @param[in] params A reference to swivel pulley's parameters.
 * @param[in,out] cable A reference to the cable structure including the versors to be
 * calculated and the updated input quantities.
 * @see CalcCableVersorsDot()
 */
void updateCableVersorsDot(const PulleyParams& params, CableVarsBase& cable);

/**
 * @brief Calculate cable speed @f$\dot{l}_i@f$.
 *
 * Given current @f$\hat{\mathbf{t}}_i, \dot{\mathbf{a}}_i@f$, the cable speed is
 * calculated as:
 * @f[
 * \dot{l}_i = \hat{\mathbf{t}}_i \cdot \dot{\mathbf{a}}_i
 * @f]
 * @param[in] vers_t Versor @f$\hat{\mathbf{t}}_i@f$.
 * @param[in] vel_OA_glob [m/s] Vector @f$\dot{\mathbf{a}}_i@f$.
 * @return Cable speed @f$\dot{l}_i@f$ in _m/s_.
 * @note See @ref legend for symbols reference.
 */
double calcCableSpeed(const Vector3d& vers_t, const Vector3d& vel_OA_glob);

/**
 * @brief Calculate cable speed @f$\dot{l}_i@f$.
 * @param[in] cable A reference to the cable variables structure.
 * @return Cable speed @f$\dot{l}_i@f$ in _m/s_.
 * @see CalcCableSpeed()
 */
void updateCableSpeed(CableVarsBase& cable);

/**
 * @brief Update a single row of geometric and analytic jacobians first-derivative.
 * @param[in] platform A reference to the updated platform structure.
 * @param[in,out] cable A reference to the cable structure with updated zero-order
 * variables and first-order variables to be updated.
 */
void updateJacobiansRowD(const PlatformVars& platform, CableVars& cable);
/**
 * @brief Update a single row of geometric and analytic jacobians first-derivative when
 * using quaternion parametrization.
 * @param[in] platform A reference to the updated platform structure.
 * @param[in,out] cable A reference to the cable structure with updated zero-order
 * variables and first-order variables to be updated.
 */
void updateJacobiansRowD(const PlatformVarsQuat& platform, CableVarsQuat& cable);

/**
 * @brief Update all first-order variables of a single cable at once.
 * @param[in] params A reference to swivel pulley's parameters.
 * @param[in] platform A reference to the updated platform structure.
 * @param[in,out] cable A reference to the cable structure with updated zero-order
 * variables and first-order variables to be updated.
 */
void updateCableFirstOrd(const PulleyParams& params, const PlatformVars& platform,
                         CableVars& cable);
/**
 * @brief Update all first-order variables of a single cable at once.
 * @param[in] params A reference to swivel pulley's parameters.
 * @param[in] platform A reference to the updated platform structure parametrized with
 * quaternions.
 * @param[in,out] cable A reference to the cable structure with updated zero-order
 * variables and first-order variables to be updated.
 */
void updateCableFirstOrd(const PulleyParams& params, const PlatformVarsQuat& platform,
                         CableVarsQuat& cable);

/**
 * @brief Update all robots first-order variables at once (inverse kinematics problem).
 * @param[in] velocity [m/s] Platform global linear velocity.
 * @param[in] orientation_dot Platform orientation time-derivative.
 * @param[in] params A reference to robot's parameters.
 * @param[in,out] vars The robot structure with updated zero-order variables
 * and first-order variables to be updated.
 */
void updateIK1(const Vector3d& velocity, const Vector3d& orientation_dot,
               const RobotParams& params, RobotVars& vars);
/**
 * @brief Update all robots first-order variables at once (inverse kinematics problem).
 * @param[in] velocity [m/s] Platform global linear velocity.
 * @param[in] orientation_dot Platform orientation time-derivative.
 * @param[in] params A reference to robot's parameters.
 * @param[in,out] vars The robot structure with updated zero-order variables
 * and first-order variables to be updated.
 */
void updateIK1(const Vector3d& velocity, const Vector4d& orientation_dot,
               const RobotParams& params, RobotVarsQuat& vars);

/** @} */ // end of FirstOrderKinematics group

} // end namespace grabcdpr

#endif // GRABCOMMON_LIBCDPR_DIFFKINEMATICS_H
