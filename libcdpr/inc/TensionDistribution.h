#ifndef GRABCOMMON_LIBCDPR_TENSIONDISTRIBUTION_H
#define GRABCOMMON_LIBCDPR_TENSIONDISTRIBUTION_H

#include "grabcommon.h"
#include "matrix_utilities.h"
#include "cdpr_types.h"
#include <iostream>

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
    CTL_for_TD(2)=280;
  }
  Vector2d CTL_for_TD;
  Vector2u indices_to_set;
  Vector2d limits_to_set;
};


/**
 * @brief Update cables tension distribution with Improved closed form method given a certain CDPR status using 3-angle
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


 // ________________for Gouttefarde_______________-


 class GF
 {
  public:
   GF()
   {
     tau_max.Fill(std::vector<double>{maxTension,maxTension,maxTension,maxTension,maxTension,maxTension,maxTension,maxTension});
     tau_min.Fill(std::vector<double>{minTension,minTension,minTension,minTension,minTension,minTension,minTension,minTension});
   }

   /**
    * @brief Update cables tension distribution with Gouttefarde algorithm given a certain CDPR status using 3-angle
    * orientation parametrization.
    * @param[in, out] vars A reference to the robot variables structure to be updated.
    * @return a bool that states if a solution was found
    */
   bool updateCablesTensionGF(RobotVars& vars);

   MatrixXd<1,8> calcalphas(MatrixXd<2,1> fc_, MatrixXd<8,1> qmin_, MatrixXd<8,1> qmax_,MatrixXd<1,2> ni_p_);

   MatrixXd<1,8> roundalphas(MatrixXd<1,8> alpha_,int decimal);

   MatrixXd<2,1> TDBaricenter(arma::mat Polygon);

   int getcount(){return cnt;};
  private:

   double maxTension=250;
   double minTension=15;

   MatrixXd<8,1> tau_max;
   MatrixXd<8,1> tau_min;
   MatrixXd<8,2> N;
   arma::mat Polygon;
   int cnt;


   int ii=7;
   int jj=8; //starting point must be optimized

   double eps1=1e-8;
   double eps2=1e-5;


 };



        // ============================================================================
        // TurnAround tension-distribution implementation (Gouttefarde et al., 2015)
        // Implemented by Nicolas Testard
        // ============================================================================

 // /!\ For mode 5, robot kinematic must have been updated before

 /**
  * @brief Tension selection mode used by the TurnAround algorithm.
  *   * Values follow the MATLAB / paper convention:
  *  1 = centroid,
  *  2 = weighted barycenter,
  *  3 = minimum L1 tension,
  *  4 = minimum L2 tension.
  *  5 = maximum directional geometric stiffness along global +Y
  *      (e = [0 1 0 0 0 0]^T).
  */
 enum class TurnaroundMode : int
 {
   Centroid = 1,
   WeightedBarycenter = 2,
   MinL1 = 3,
   MinL2 = 4,
   MaxDirectionalStiffnessY = 5
 };

 /**
  * @brief Two-dimensional affine tension space t = tp + N*lambda.
  *   * The rows are stored in physical cable order [T1 ... T8].
  * N is an orthonormal basis of ker(W), tp is the minimum-norm
  * particular solution, and qmin/qmax define the inequalities
  *   *     qmin <= N*lambda <= qmax.
  *   * polygon is filled by the TurnAround traversal for the centroid and
  * weighted-barycenter modes.  The maximum number of vertices is 2*m = 16.
  */
 struct LambdaSpaceTurnaround
 {
   MatrixXd<8,2> N;
   MatrixXd<8,1> tp;
   MatrixXd<8,1> qmin;
   MatrixXd<8,1> qmax;
   MatrixXd<2,16> polygon;
   unsigned int vertex_count;
 };

 /**
  * @brief Compute the 2-D lambda space used by the TurnAround algorithm.
  *   * The admissible tension limits are explicit inputs.  This function only
  * builds N, tp and the inequality bounds; the polygon traversal and the
  * selected optimum are performed by UpdateCablesTensionTurnaround().
  *   * @param[in] vars Current robot variables.
  * @param[in] min_tension Minimum admissible cable tension.
  * @param[in] max_tension Maximum admissible cable tension.
  * @param[out] space Computed affine lambda space.
  * @return true if the lambda-space construction succeeds.
  */
 bool InitializeLambdaSpaceTurnaround(const RobotVars& vars,
                                   const double min_tension,
                                   const double max_tension,
                                   LambdaSpaceTurnaround& space);

 /**
  * @brief Compute the cable tensions with the original TurnAround traversal.
  *   * The function obtains the project tension bounds from
  * Index_and_limits::CTL_for_TD, calls ComputeLambdaSpaceTurnaround(), follows
  * the inequality lines according to the TurnAround algorithm, and computes
  * the selected solution.  GF and the pre-existing tension-distribution
  * functions are untouched.
  *   * @param[in,out] vars Robot variables; tension_vector is updated on success.
  * @param[in] mode Desired tension selection mode.
  * @return true when a feasible tension distribution is obtained.
  */
 bool UpdateCablesTensionTurnaround(
   RobotVars& vars,
   TurnaroundMode mode = TurnaroundMode::WeightedBarycenter);

}




#endif // GRABCOMMON_LIBCDPR_TENSIONDISTRIBUTION_H
