#ifndef GRABCOMMON_LIBCDPR_TENSIONDISTRIBUTION_H
#define GRABCOMMON_LIBCDPR_TENSIONDISTRIBUTION_H

#include "grabcommon.h"
#include "matrix_utilities.h"
#include "cdpr_types.h"


using namespace grabnum;


/**
 * @brief Namespace for CDPR-related utilities, such as kinematics and dynamics.
 */

namespace grabcdpr{

struct Index_and_limits{
  Index_and_limits()
  {
    indices_to_set(1) = 0;
    indices_to_set(2) = 0;
    limits_to_set(1) = 0;
    limits_to_set(2) = 0;
    CTL_for_TD(1)=15;
    CTL_for_TD(2)=250;
  }
  Vector2d CTL_for_TD;
  Vector2u indices_to_set;
  Vector2d limits_to_set;
};


/**
 * @brief Update cables tension distribution given a certain CDPR status using 3-angle
 * orientation parametrization.
 * @param[in, out] vars A reference to the robot variables structure to be updated.
 * @return a bool that states if a solution was found
 */
bool updateCablesTensionDistribution(RobotVars& vars);

/**
 * @brief Inserts the limit tensions in lower dimensio tension vector when multiple
 * iterations are required
 * @param[in] tau_h_ lower dimension vector containing calculated
 * tension values
 *  @param[in] Index_to_set Indices where limit tension must be
 * inserted
 * @param[in] TensionLimit Tension Limit that must be inserted
 * @return a tension vector with some component set to a limit value
 */

 MatrixXd<7,1> insertLimitTension6to7(const MatrixXd<6,1>& tau_h_,
                            const uint& index_to_set,
                             const double& TesionLimit);

 MatrixXd<8,1> insertLimitTension7to8(const MatrixXd<7,1>& tau_h_,
                                  const uint& index_to_set,
                                  const double& TesionLimit);
}




#endif // GRABCOMMON_LIBCDPR_TENSIONDISTRIBUTION_H
