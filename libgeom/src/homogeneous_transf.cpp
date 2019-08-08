/**
 * @file homogeneous_transf.cpp
 * @author Simone Comari
 * @date 16 Jul 2019
 * @brief File containing definitions of homogeneous_transf.h.
 */

#include "homogeneous_transf.h"

namespace grabgeom {

grabnum::Matrix4d BuildHomogeneousTransf()
{
  return grabnum::Matrix4d(1.0); // init as unitary diagonal matrix
}

grabnum::Matrix4d BuildHomogeneousTransf(const grabnum::Matrix3d& rot_mat,
                                         const grabnum::Vector3d& transl_vect)
{
  grabnum::Matrix4d homg_mat(1.0); // init as unitary diagonal matrix
  homg_mat.SetBlock<3, 3>(1, 1, rot_mat);
  homg_mat.SetBlock<3, 1>(1, 4, transl_vect);
  return homg_mat;
}

void SetHomgTransfRot(const grabnum::Matrix3d& rot_mat, grabnum::Matrix4d& homg_transf)
{
  homg_transf.SetBlock<3, 3>(0, 0, rot_mat);
}

void SetHomgTransfTransl(const grabnum::Vector3d& transl_vect,
                         grabnum::Matrix4d& homg_transf)
{
  homg_transf.SetBlock<3, 1>(0, 3, transl_vect);
}

grabnum::Matrix3d GetHomgTransfRot(const grabnum::Matrix4d& homg_transf)
{
  return homg_transf.GetBlock<3, 3>(1, 1);
}

grabnum::Vector3d GetHomgTransfTransl(const grabnum::Matrix4d& homg_transf)
{
  return homg_transf.GetBlock<3, 1>(1, 4);
}

grabnum::Matrix4d InverseTransformation(const grabnum::Matrix4d& homg_transf)
{
  grabnum::Matrix4d inv_trans(1.0); // init as unitary diagonal matrix
  grabnum::Matrix3d rot_mat_inv = GetHomgTransfRot(homg_transf).Transpose();
  inv_trans.SetBlock<3, 3>(1, 1, rot_mat_inv);
  inv_trans.SetBlock<3, 1>(1, 4, -rot_mat_inv * GetHomgTransfTransl(homg_transf));
  return inv_trans;
}

} // end namespace grabgeom
