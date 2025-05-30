/**
 * @file statics.h
 * @author Simone Comari
 * @date 07 Feb 2020
 * @brief This file include functions strictly related to CDPR statics.
 */

#ifndef GRABCOMMON_LIBCDPR_STATICS_H
#define GRABCOMMON_LIBCDPR_STATICS_H

#include <assert.h>

#include "grabcommon.h"
#include "matrix_utilities.h"

#include "cdpr_types.h"
#include "dynamics.h"
#include "kinematics.h"

using namespace grabnum;

/**
 * @brief Namespace for CDPR-related utilities, such as kinematics and dynamics.
 */
namespace grabcdpr {
/** @defgroup Dynamics Dynamics
 * This group collects all elements related to the dynamics of a generic 6DoF CDPR.
 * @{
 */

/**
 * @brief Calculate cables static tension.
 * @param[in] geom_jacobian Geometric jacobian of the CDPR.
 * @param[in] ext_load External loads acting on the platform.
 * @return A vector of cables static tension.
 */
arma::vec calcCablesStaticTension(const arma::mat& geom_jacobian,
                                  const Vector6d& ext_load);

/**
 * @brief Update cables static tension given a certain CDPR status using 3-angle
 * orientation parametrization.
 * @param[in, out] vars A reference to the robot variables structure to be updated.
 */
void updateCablesStaticTension(RobotVars& vars);
/**
 * @brief Update cables static tension given a certain CDPR status using quaternion
 * orientation parametrization.
 * @param[in, out] vars A reference to the robot variables structure to be updated.
 */
void updateCablesStaticTension(RobotVarsQuat& vars);

/** @} */ // end of Dynamics group

} // end namespace grabcdpr

#endif // GRABCOMMON_LIBCDPR_STATICS_H
