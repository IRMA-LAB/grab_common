/**
 * @file homogeneous_transf.h
 * @author Simone Comari
 * @date 16 Jul 2019
 * @brief File containing homogeneous transformation utils to be included in the GRAB
 * geometric library.
 */

#ifndef GRABCOMMON_LIBGEOM_HOMOGENEOUS_TRANSF_H
#define GRABCOMMON_LIBGEOM_HOMOGENEOUS_TRANSF_H

#include "rotations.h"

/**
 * @brief Namespace for GRAB geometric library.
 */
namespace grabgeom {

/**
 * @brief Build an identity homogeneous transformation matrix.
 *
 * This transformation is equivalent to a null rotation and a zero translation.
 * @return An identity homogeneous transformation matrix.
 */
grabnum::Matrix4d BuildHomogeneousTransf();
/**
 * @brief Build a homogeneous transformation matrix from a given rotation and translation.
 * @param[in] rot_mat A rotation matrix.
 * @param[in] transl_vect A translation vector.
 * @return The corresponding homogeneous transformation matrix.
 */
grabnum::Matrix4d BuildHomogeneousTransf(const grabnum::Matrix3d& rot_mat,
                                         const grabnum::Vector3d& transl_vect);

/**
 * @brief Set the rotational part of a given homogeneous transformation matrix.
 * @param[in] rot_mat The new rotation matrix.
 * @param homg_transf The original homogeneous transformation matrix.
 */
void SetHomgTransfRot(const grabnum::Matrix3d& rot_mat, grabnum::Matrix4d& homg_transf);

/**
 * @brief Set the translational part of a given homogeneous transformation matrix.
 * @param[in] transl_vect The new translation vector.
 * @param homg_transf The original homogeneous transformation matrix.
 */
void SetHomgTransfTransl(const grabnum::Vector3d& transl_vect,
                         grabnum::Matrix4d& homg_transf);

/**
 * @brief Extract the rotational part of a given homogeneous transformation matrix.
 * @param[in] homg_transf An homogeneous transformation matrix.
 * @return A rotation matrix.
 */
grabnum::Matrix3d GetHomgTransfRot(const grabnum::Matrix4d& homg_transf);

/**
 * @brief Extract the translational part of a given homogeneous transformation matrix.
 * @param[in] homg_transf An homogeneous transformation matrix.
 * @return A translation vector.
 */
grabnum::Vector3d GetHomgTransfTransl(const grabnum::Matrix4d& homg_transf);

/**
 * @brief Compute the inverse of a given homogeneous transformation matrix.
 * @param[in] homg_transf An homogeneous transformation matrix.
 * @return The inverse of a given homogeneous transformation matrix.
 */
grabnum::Matrix4d InverseTransformation(const grabnum::Matrix4d& homg_transf);

} // end namespace grabgeom

#endif // GRABCOMMON_LIBGEOM_HOMOGENEOUS_TRANSF_H
