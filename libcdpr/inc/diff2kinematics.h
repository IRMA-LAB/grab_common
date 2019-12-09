/**
 * @file diff2kinematics.h
 * @author Edoardo Idà, Simone Comari
 * @date 02 Dec 2019
 * @brief File containing second-order differential kinematics-related functions to be
 * included in the GRAB CDPR library.
 */

#ifndef GRABCOMMON_LIBCDPR_DIFF2KINEMATICS_H
#define GRABCOMMON_LIBCDPR_DIFF2KINEMATICS_H

#include "diffkinematics.h"

/**
 * @brief Namespace for CDPR-related utilities, such as kinematics and dynamics.
 */
namespace grabcdpr {

/** @defgroup SecondOrderKinematics Second Order Kinematics
 * This group collects all elements related to second-order kinematics of a generic 6DoF
 * CDPR.
 * @{
 */

/**
 * @brief Update platform-related second-order quantities.
 *
 * Given a new acceleration of the platform
 * @f$\ddot{\mathbf{x}} = (\ddot{\mathbf{p}}^T, \ddot{\boldsymbol{\varepsilon}}^T)^T
 * @f$, the following quantities are updated:
 * @f[
 * \boldsymbol{\alpha} = \dot{\mathbf{H}}
 * (\dot{\boldsymbol{\varepsilon}}, \boldsymbol{\varepsilon})
 * \dot{\boldsymbol{\varepsilon}} + \mathbf{H}(\boldsymbol{\varepsilon})
 * \ddot{\boldsymbol{\varepsilon}} \\
 * \ddot{\mathbf{r}} = \ddot{\mathbf{p}} + \boldsymbol{\alpha} \times \mathbf{p}'_G +
 * \boldsymbol{\omega}\times (\boldsymbol{\omega}\times \mathbf{p}'_G) =
 * \ddot{\mathbf{p}} + (\tilde{\mathbf{A}} +
 * \tilde{\boldsymbol{\Omega}} \tilde{\boldsymbol{\Omega}}) \mathbf{p}'_G
 * @f]
 * being @f$\tilde{\boldsymbol{\Omega}}, \tilde{\mathbf{A}}@f$ the anti-symmetric matrix
 * of @f$\boldsymbol{\omega}, \boldsymbol{\alpha}@f$ respectively.
 * @param[in] acceleration [m/s<sup>2</sup>] Platform global linear acceleration
 * @f$\ddot{\mathbf{p}}@f$.
 * @param[in] angles_acc [rad/s<sup>2</sup>] Platform orientation second time-derivative
 * @f$\ddot{\boldsymbol{\varepsilon}}@f$.
 * @param[in] pos_PG_glob [m] Global CoG position @f$\mathbf{p}'_G@f$.
 * @param[out] platform A pointer to the platform structure including vars to be updated.
 * @note See @ref legend for symbols reference.
 * @note Both orientation parametrizations are valid here, that is both angles and
 * quaternions can be used.
 */
void updatePlatformAcc(const Vector3d& acceleration, const Vector3d& angles_acc,
                       const Vector3d& pos_PG_glob, PlatformVars& platform);
void updatePlatformAcc(const Vector3d& acceleration, const grabgeom::Quaternion& quat_acc,
                       const Vector3d& pos_PG_glob, PlatformVarsQuat& platform);
/**
 * @brief Update platform-related second-order quantities.
 * @param[in] acceleration [m/s<sup>2</sup>] Platform global linear acceleration
 * @f$\ddot{\mathbf{p}}@f$.
 * @param[in] angles_acc [rad/s<sup>2</sup>] Platform orientation second time-derivative
 * @f$\ddot{\boldsymbol{\varepsilon}}@f$.
 * @param[in,out] platform A pointer to the platform structure including vars to be
 * updated.
 * @see UpdatePlatformAcc()
 * @note Both orientation parametrizations are valid here, that is both angles and
 * quaternions can be used.
 */
void updatePlatformAcc(const Vector3d& acceleration, const Vector3d& angles_acc,
                       PlatformVars& platform);
void updatePlatformAcc(const Vector3d& acceleration,
                       const grabgeom::Quaternion& angles_acc,
                       PlatformVarsQuat& platform);

/**
 * @brief Update global velocity of point @f$A_i@f$ and relative segments.
 *
 * Given a current @f$\mathbf{a}'_i@f$ and platform variables, the following vector is
 * updated:
 * @f[
 * \ddot{\mathbf{a}}_i = \ddot{\mathbf{p}} + \boldsymbol{\alpha} \times \mathbf{a}'_i +
 * \boldsymbol{\omega}\times (\boldsymbol{\omega}\times \mathbf{a}'_i) =
 * \ddot{\mathbf{p}} + (\tilde{\mathbf{A}} +
 * \tilde{\boldsymbol{\Omega}} \tilde{\boldsymbol{\Omega}}) \mathbf{a}'_i
 * @f]
 * being @f$\tilde{\boldsymbol{\Omega}}, \tilde{\mathbf{A}}@f$ the anti-symmetric matrix
 * of @f$\boldsymbol{\omega}, \boldsymbol{\alpha}@f$ respectively.
 * @param[in] pos_PA_glob [m] Vector @f$\mathbf{a}'_i@f$.
 * @param[in] platform A pointer to the updated platform structure.
 * @param[out] cable A pointer to the cable structure including the accelerations to be
 * updated.
 * @note See @ref legend for symbols reference.
 * @note Both orientation parametrizations are valid here, that is both angles and
 * quaternions can be used.
 */
Vector3d calcAccA(const Vector3d& pos_PA_glob, const PlatformVarsBase& platform);

/**
 * @brief Update global velocity of point @f$A_i@f$ and relative segments.
 * @param[in] platform A pointer to the updated platform structure.
 * @param[in,out] cable A pointer to the cable structure including the accelerations to be
 * updated.
 * @see UpdateAccA()
 * @note Both orientation parametrizations are valid here, that is both angles and
 * quaternions can be used.
 */
void updateAccA(const PlatformVarsBase& platform, CableVarsBase& cable);

/**
 * @brief Calculate pulley swivel angle acceleration @f$\ddot{\sigma}_i@f$.
 *
 * Given current @f$\hat{\mathbf{u}}_i, \hat{\mathbf{w}}_i, \dot{\mathbf{a}}_i,
 * \boldsymbol{\rho}^*_i, \ddot{\mathbf{a}}_i, \dot{\sigma}_i@f$, the swivel angle
 * acceleration is calculated as:
 * @f[
 * \ddot{\sigma}_i = \frac{\hat{\mathbf{w}}_i \cdot \ddot{\mathbf{a}}_i -
 *        2 \hat{\mathbf{u}}_i \cdot \dot{\mathbf{a}}_i \dot{\sigma}_i}
 *        {\hat{\mathbf{u}}_i \cdot \boldsymbol{\rho}^*_i}
 * @f]
 * @param[in] vers_u Versor @f$\hat{\mathbf{u}}_i@f$.
 * @param[in] vers_w Versor @f$\hat{\mathbf{w}}_i@f$.
 * @param[in] vel_OA_glob [m/s] Vector @f$\dot{\mathbf{a}}_i@f$.
 * @param[in] pos_DA_glob [m] Vector @f$\boldsymbol{\rho}^*_i@f$.
 * @param[in] acc_OA_glob [m/s<sup>2</sup>] Vector @f$\ddot{\mathbf{a}}_i@f$.
 * @param[in] swivel_ang_vel [rad/s] Swivel angle speed @f$\dot{\sigma}_i@f$.
 * @return Swivel angle acceleration @f$\ddot{\sigma}_i@f$ in _rad/s<sup>2</sup>_.
 * @note See @ref legend for symbols reference.
 */
double calcSwivelAngAcc(const Vector3d& vers_u, const Vector3d& vers_w,
                        const Vector3d& vel_OA_glob, const Vector3d& pos_DA_glob,
                        const Vector3d& acc_OA_glob, const double swivel_ang_vel);

/**
 * @brief Calculate pulley swivel angle acceleration @f$\ddot{\sigma}_i@f$.
 * @param[in] cable A pointer to the updated cable variables structure.
 * @return Swivel angle acceleration @f$\ddot{\sigma}_i@f$ in _rad/s<sup>2</sup>_.
 * @see CalcSwivelAngAcc()
 */
void updateSwivelAngAcc(CableVarsBase& cable);

/**
 * @brief Calculate pulley tangent angle acceleration @f$\ddot{\psi}_i@f$.
 *
 * Given current @f$\hat{\mathbf{u}}_i, \hat{\mathbf{n}}_i, \boldsymbol{\rho}^*_i,
 * \boldsymbol{\rho}_i, \ddot{\mathbf{a}}_i, \dot{l}_i, \psi_i, \dot{\psi}_i,
 * \dot{\sigma}_i@f$, the tangent angle acceleration is calculated as:
 * @f[
 * \ddot{\psi}_i = \frac{\hat{\mathbf{n}}_i \cdot \ddot{\mathbf{a}}_i +
 *        \hat{\mathbf{u}}_i \cdot \boldsymbol{\rho}^*_i \cos(\psi_i) \dot{\sigma}_i -
 *        (2 \dot{l}_i + r_i \dot{\psi}_i) \dot{\psi}_i}
 *        {\|\boldsymbol{\rho}_i\|}
 * @f]
 * being @f$r_i@f$ a known parameter.
 * @param[in] pulley_radius [m] Swivel pulley radius @f$r_i@f$.
 * @param[in] vers_u Versor @f$\hat{\mathbf{u}}_i@f$.
 * @param[in] vers_n Versor @f$\hat{\mathbf{n}}_i@f$.
 * @param[in] pos_DA_glob [m] Vector @f$\boldsymbol{\rho}^*_i@f$.
 * @param[in] pos_BA_glob [m] Vector @f$\boldsymbol{\rho}_i@f$.
 * @param[in] acc_OA_glob [m/s<sup>2</sup>] Vector @f$\ddot{\mathbf{a}}_i@f$.
 * @param[in] speed [m/s] Cable speed @f$\dot{l}_i@f$.
 * @param[in] tan_ang [rad] Tangent angle @f$\psi_i@f$.
 * @param[in] tan_ang_vel [rad/s] Tangent angle speed @f$\dot{\psi}_i@f$.
 * @param[in] swivel_ang_vel [rad/s] See @f$\dot{\sigma}_i@f$.
 * @return Tangent angle acceleration @f$\ddot{\psi}_i@f$ in _rad/s<sup>2</sup>_.
 * @note See @ref legend for symbols reference.
 */
double calcTangAngAcc(const double pulley_radius, const Vector3d& vers_u,
                      const Vector3d& vers_n, const Vector3d& pos_DA_glob,
                      const Vector3d& pos_BA_glob, const Vector3d& acc_OA_glob,
                      const double speed, const double tan_ang, const double tan_ang_vel,
                      const double swivel_ang_vel);
/**
 * @brief Calculate pulley tangent angle acceleration @f$\ddot{\psi}_i@f$.
 * @param[in] pulley_radius [m] Swivel pulley radius @f$r_i@f$.
 * @param[in] cable A pointer to the updated cable variables structure.
 * @return Tangent angle acceleration @f$\ddot{\psi}_i@f$ in _rad/s<sup>2</sup>_.
 * @see CalcTangAngAcc()
 */
void updateTangAngAcc(const double pulley_radius, CableVarsBase& cable);

/**
 * @brief Calculate cable acceleration @f$\ddot{l}_i@f$.
 *
 * Given current @f$\hat{\mathbf{u}}_i, \hat{\mathbf{t}}_i, \ddot{\mathbf{a}}_i,
 * \boldsymbol{\rho}^*_i, \boldsymbol{\rho}_i, \psi_i, \dot{\psi}_i, \dot{\sigma}_i@f$,
 * the cable acceleration is calculated as:
 * @f[
 * \ddot{l}_i = \hat{\mathbf{u}}_i \cdot \boldsymbol{\rho}^*_i \sin(\psi_i)
 * \dot{\sigma}_i^2 +
 *        \|\boldsymbol{\rho}_i\| \dot{\psi}_i +
 *        \hat{\mathbf{t}}_i \cdot \ddot{\mathbf{a}}_i
 * @f]
 * @param[in] vers_u Versor @f$\hat{\mathbf{u}}_i@f$.
 * @param[in] vers_t Versor @f$\hat{\mathbf{t}}_i@f$.
 * @param[in] pos_DA_glob [m] Vector @f$\boldsymbol{\rho}^*_i@f$.
 * @param[in] pos_BA_glob [m] Vector @f$\boldsymbol{\rho}_i@f$.
 * @param[in] acc_OA_glob [m/s<sup>2</sup>] Vector @f$\ddot{\mathbf{a}}_i@f$.
 * @param[in] tan_ang [rad] Tangent angle @f$\psi_i@f$.
 * @param[in] tan_ang_vel [rad/s] Tangent angle speed @f$\dot{\psi}_i@f$.
 * @param[in] swivel_ang_vel [rad/s] Swivel angle speed @f$\dot{\sigma}_i@f$.
 * @return Cable acceleration @f$\ddot{l}_i@f$ in _m/s<sup>2</sup>_.
 * @note See @ref legend for symbols reference.
 */
double calcCableAcc(const Vector3d& vers_u, const Vector3d& vers_t,
                    const Vector3d& pos_DA_glob, const Vector3d& pos_BA_glob,
                    const Vector3d& acc_OA_glob, const double tan_ang,
                    const double tan_ang_vel, const double swivel_ang_vel);

/**
 * @brief Calculate cable acceleration @f$\ddot{l}_i@f$.
 * @param[in] cable A pointer to the updated cable variables structure.
 * @return Cable acceleration @f$\ddot{l}_i@f$ in _m/s<sup>2</sup>_.
 * @see CalcCableAcc()
 */
void updateCableAcc(CableVarsBase& cable);

/**
 * @brief Update all second-order variables of a single cable at once.
 * @param[in] params Swivel pulley parameters.
 * @param[in] platform A pointer to the updated platform structure.
 * @param[in,out] cable A pointer to the cable structure with updated zero and first-order
 * variables and second-order variables to be updated.
 * @note Both orientation parametrizations are valid here, that is both angles and
 * quaternions can be used.
 */
void updateCableSecondOrd(const PulleyParams& params, const PlatformVarsBase& platform,
                          CableVarsBase& cable);

/**
 * @brief Update all robots second-order variables at once (inverse kinematics problem).
 * @param[in] acceleration [m/s<sup>2</sup>] Platform global linear acceleration
 * @f$\ddot{\mathbf{p}}@f$.
 * @param[in] orientation_ddot [rad/s<sup>2</sup>] Platform orientation second
 * time-derivative
 * @f$\ddot{\boldsymbol{\varepsilon}}@f$.
 * @param[in] params A pointer to the robot parameters structure.
 * @param[in,out] vars A pointer to the robot structure with updated zero and first-order
 * variables and second-order variables to be updated.
 * @note Both orientation parametrizations are valid here, that is both angles and
 * quaternions can be used.
 */
void updateIK2(const Vector3d& acceleration, const Vector3d& orientation_ddot,
               const RobotParams& params, RobotParams& vars);
void updateIK2(const Vector3d& acceleration, const grabgeom::Quaternion& orientation_ddot,
               const RobotParams& params, RobotVarsQuat& vars);

/** @} */ // end of SecondOrderKinematics group

/**
 * @brief Update all robots variables at once (full inverse kinematics problem).
 * @param[in] position Platform global position.
 * @param[in] orientation Platform global orientation.
 * @param[in] velocity [m/s] Platform global linear velocity.
 * @param[in] orientation_dot Platform orientation time-derivative.
 * @param[in] acceleration [m/s<sup>2</sup>] Platform global linear acceleration.
 * @param[in] orientation_ddot Platform orientation time-derivative.
 * @param[in] params A pointer to the robot parameters structure.
 * @param[out] vars A pointer to the robot structure to be updated.
 * @note Both orientation parametrizations are valid here, that is both angles and
 * quaternions can be used.
 */
void updateIK(const Vector3d& position, const Vector3d& orientation,
              const Vector3d& velocity, const Vector3d& orientation_dot,
              const Vector3d& acceleration, const Vector3d& orientation_ddot,
              const RobotParams& params, RobotVars& vars);
void updateIK(const Vector3d& position, const grabgeom::Quaternion& orientation,
              const Vector3d& velocity, const grabgeom::Quaternion& orientation_dot,
              const Vector3d& acceleration, const grabgeom::Quaternion& orientation_ddot,
              const RobotParams& params, RobotVarsQuat& vars);

} // end namespace grabcdpr

#endif // GRABCOMMON_LIBCDPR_DIFF2KINEMATICS_H
