/**
 * @file statics.cpp
 * @author Simone Comari
 * @date 07 Feb 2020
 * @brief File containing definitions of functions declared in statics.h.
 */

#include "statics.h"

namespace grabcdpr {

arma::vec calcCablesStaticTension(const arma::mat& geom_jacobian,
                                  const Vector6d& ext_load)
{
  arma::mat A              = geom_jacobian * geom_jacobian.t();
  arma::vec b              = geom_jacobian * toArmaVec(ext_load);
  arma::vec tension_vector = arma::solve(A, b);
  // Tensions cannot be negative
  if (tension_vector.min() < 0.0)
  {
    PrintColor('y', "WARNING: negative cables tension! Values will be clamped to 0.");
    tension_vector = arma::clamp(tension_vector, 0.0, tension_vector.max());
  }
  return tension_vector;
}

void updateCablesStaticTension(RobotVars& vars)
{
  vars.tension_vector =
    calcCablesStaticTension(vars.geom_jacobian, vars.platform.ext_load);
}

void updateCablesStaticTension(RobotVarsQuat& vars)
{
  vars.tension_vector =
    calcCablesStaticTension(vars.geom_jacobian, vars.platform.ext_load);
}

} // namespace grabcdpr
