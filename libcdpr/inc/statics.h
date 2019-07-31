#ifndef GRABCOMMON_LIBCDPR_STATICS_H
#define GRABCOMMON_LIBCDPR_STATICS_H

#include "opencv2/core.hpp"

#include "matrix_utilities.h"

#include "cdpr_types.h"
#include "kinematics.h"

using namespace grabnum;

cv::Mat toCvMat(Vector3d vect);
cv::Mat toCvMat(Matrix3d vect);

/**
 * @brief Namespace for CDPR-related utilities, such as kinematics and dynamics.
 */
namespace grabcdpr {

void calcGeometricStatic(const RobotParams& params,
                         const cv::Mat &fixed_coord,
                         const cv::Mat &var_coord,
                         const VectorXi<POSE_DIM>& mask, cv::Mat& mat, cv::Mat vector);

cv::Mat CalcGsJacobians(const RobotVars& vars, const cv::Mat& Ja, const cv::Mat& Ju,
                     const Vector3d& mg);

} // end namespace grabcdpr

#endif // GRABCOMMON_LIBCDPR_STATICS_H
