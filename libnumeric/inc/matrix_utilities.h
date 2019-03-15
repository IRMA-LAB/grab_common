/**
 * @file matrix_utilities.h
 * @author Edoardo Idà, Simone Comari
 * @date 15 Mar 2019
 * @brief File containing matrix utilities to be included in the GRAB numeric library.
 *
 * In this file there are some external independent functions which provide extra
 * utilities related to matrix operations. All elements of this file are templated to be
 * compatible with all different types and leave the user more flexibility.
 */

#ifndef GRABCOMMON_LIBNUMERIC_MATRIX_UTILITIES_H
#define GRABCOMMON_LIBNUMERIC_MATRIX_UTILITIES_H

#include "matrix.h"

/**
 * @brief Namespace for GRAB numeric library.
 */
namespace grabnum {

#if (MCU_TARGET == 0)
/**
 * Print function for matrix.
 *
 * @param[in] stream A std output stream.
 * @param[in] matrix A @f$m\times n@f$ matrix.
 * @return A reference to the input stream.
 */
template <typename T, uint8_t rows, uint8_t cols>
std::ostream& operator<<(std::ostream& stream, const Matrix<T, rows, cols>& matrix);
#endif

/**
 * Addition between a scalar and a matrix.
 *
 * @param[in] scalar A scalar value.
 * @param[in] matrix A @f$m\times n@f$ matrix.
 * @return A @f$m\times n@f$ matrix, result of the addition.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator+(const T& scalar, const Matrix<T, rows, cols>& matrix);

/**
 * Addition between a matrix and a scalar.
 *
 * @param[in] matrix A @f$m\times n@f$ matrix.
 * @param[in] scalar A scalar value.
 * @return A @f$m\times n@f$ matrix.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator+(const Matrix<T, rows, cols>& matrix, const T& scalar);

/**
 * Addition between two matrices.
 *
 * @param[in] matrix1 A @f$m\times n@f$ matrix.
 * @param[in] matrix2 A @f$m\times n@f$ matrix.
 * @return A @f$m\times n@f$ matrix.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator+(const Matrix<T, rows, cols>& matrix1,
                                const Matrix<T, rows, cols>& matrix2);

/**
 * Subtraction between a scalar and a matrix.
 *
 * @param[in] scalar A scalar value.
 * @param[in] matrix A @f$m\times n@f$ matrix.
 * @return A @f$m\times n@f$ matrix.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator-(const T& scalar, const Matrix<T, rows, cols>& matrix);

/**
 * Subtraction between a matrix and a scalar.
 *
 * @param[in] matrix A @f$m\times n@f$ matrix.
 * @param[in] scalar A scalar value.
 * @return A @f$m\times n@f$ matrix, result of the subtraction.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator-(const Matrix<T, rows, cols>& matrix, const T& scalar);

/**
 * Subtraction between two matrices.
 *
 * @param[in] matrix1 A @f$m\times n@f$ matrix.
 * @param[in] matrix2 A @f$m\times n@f$ matrix.
 * @return A @f$m\times n@f$ matrix.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator-(const Matrix<T, rows, cols>& matrix1,
                                const Matrix<T, rows, cols>& matrix2);

/**
 * Returns the opposite of a matrix.
 * This is equivalent to multiplication by -1.
 *
 * @param[in] matrix A @f$m\times n@f$ matrix.
 * @return A @f$m\times n@f$ matrix.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator-(const Matrix<T, rows, cols>& matrix);

/**
 * Row-column matrix multiplication.
 *
 * @param[in] matrix1 A @f$m\times n@f$ matrix.
 * @param[in] matrix2 A @f$n\times p@f$ matrix.
 * @return A @f$m\times p@f$ matrix.
 */
template <typename T, uint8_t rows1, uint8_t dim_common, uint8_t cols2>
Matrix<T, rows1, cols2> operator*(const Matrix<T, rows1, dim_common>& matrix1,
                                  const Matrix<T, dim_common, cols2>& matrix2);

/**
 * Outer product operation.
 *
 * @param[in] vvect A m-dimensional vertical vector.
 * @param[in] hvect A n-dimensional horizontal vector.
 * @return A @f$m\times n@f$ matrix.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator*(const VectorX<T, rows>& vvect,
                                const Matrix<T, 1, cols>& hvect);

/**
 * Scalar product operation.
 *
 * @param[in] scalar A scalar value.
 * @param[in] matrix A @f$m\times n@f$ matrix.
 * @return A @f$m\times n@f$ matrix.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator*(const T& scalar, const Matrix<T, rows, cols>& matrix);

/**
 * Scalar product operation.
 *
 * @param[in] matrix A @f$m\times n@f$ matrix.
 * @param[in] scalar A scalar value.
 * @return A @f$m\times n@f$ matrix.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator*(const Matrix<T, rows, cols>& matrix, const T& scalar);

/**
 * Element-wise vector multiplication.
 *
 * @param[in] vvect1 A m-dimensional vertical vector.
 * @param[in] vvect2 A m-dimensional vertical vector.
 * @return A m-dimensional vertical vector.
 */
template <typename T, uint8_t rows>
VectorX<T, rows> operator*(const VectorX<T, rows>& vvect1,
                           const VectorX<T, rows>& vvect2);

/**
 * Element-wise vector multiplication.
 *
 * @param[in] hvect1 A m-dimensional horizontal vector.
 * @param[in] hvect2 A m-dimensional horizontal vector.
 * @return A m-dimensional horizontal vector.
 */
template <typename T, uint8_t cols>
Matrix<T, 1, cols> operator*(const Matrix<T, 1, cols>& hvect1,
                             const Matrix<T, 1, cols>& hvect2);

/**
 * Matrix division by scalar operation.
 *
 * @param[in] matrix A @f$m\times n@f$ matrix.
 * @param[in] scalar A scalar value.
 * @return A @f$m\times n@f$ matrix.
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols> operator/(const Matrix<T, rows, cols>& matrix, const T& scalar);

/**
 * Matrix horizontal concatenation.
 *
 * @param[in] matrix_lx A @f$m\times n@f$ matrix.
 * @param[in] matrix_rx A @f$m\times p@f$ matrix.
 * @return A @f$m\times (n+p)@f$ matrix.
 */
template <typename T, uint8_t rows, uint8_t cols1, uint8_t cols2>
Matrix<T, rows, cols1 + cols2> HorzCat(const Matrix<T, rows, cols1>& matrix_lx,
                                       const Matrix<T, rows, cols2>& matrix_rx);

/**
 * Matrix vertical concatenation.
 *
 * @param[in] matrix_up A @f$m\times n@f$ matrix.
 * @param[in] matrix_down A @f$p\times n@f$ matrix.
 * @return A @f$(m+p)\times n@f$ matrix.
 */
template <typename T, uint8_t rows1, uint8_t rows2, uint8_t cols>
Matrix<T, rows1 + rows2, cols> VertCat(const Matrix<T, rows1, cols>& matrix_up,
                                       const Matrix<T, rows2, cols>& matrix_down);

/**
 * Vector dot-product operation.
 *
 * @param[in] vvect1 A m-dimensional vertical vector.
 * @param[in] vvect2 A m-dimensional vertical vector.
 * @return A scalar value.
 */
template <typename T, uint8_t dim>
T Dot(const VectorX<T, dim>& vvect1, const VectorX<T, dim>& vvect2);

/**
 * Vector dot-product operation.
 *
 * @param[in] hvect A m-dimensional horizontal vector.
 * @param[in] vvect A m-dimensional vertical vector.
 * @return A scalar value.
 */
template <typename T, uint8_t dim>
T Dot(const Matrix<T, 1, dim>& hvect, const VectorX<T, dim>& vvect);

/**
 * Vector L2-norm (i.e. Euclidean norm).
 *
 * @param[in] vvect A m-dimensional vertical vector.
 * @return A scalar value.
 */
template <typename T, uint8_t dim> double Norm(const VectorX<T, dim>& vvect);

/**
 * Vector L2-norm (i.e. Euclidean norm).
 *
 * @param[in] hvect A m-dimensional horizontal vector.
 * @return A scalar value.
 */
template <typename T, uint8_t dim> double Norm(const Matrix<T, 1, dim>& hvect);

/**
 * Vector cross-product operation.
 *
 * @param[in] vvect3d1 A 3-dimensional vertical vector.
 * @param[in] vvect3d2 A 3-dimensional vertical vector.
 * @return A 3x3 matrix.
 */
template <typename T>
Vector3<T> Cross(const Vector3<T>& vvect3d1, const Vector3<T>& vvect3d2);

/**
 * Computes the skew-symmetric matrix of a 3D vector.
 *
 * @param[in] vvect3d A 3-dimensional vertical vector.
 * @return A 3x3 matrix.
 */
template <typename T> Matrix3<T> Skew(const Vector3<T>& vvect3d);

/**
 * Function to get cofactor of a matrix given entry position.
 *
 * Given a generic @f$n\times m@f$ matrix @f$\mathbf{A}@f$ and an entry position (_p_,_q_)
 * , it is defined _co-factor_ a @f$(n-1)\times (m-1)@f$ matrix @f$\mathbf{C}@f$ obtained
 * by removing _p_-th row and _q_-th column from @f$\mathbf{A}@f$.
 *
 * @param[in] matrix The matrix from where to extract the cofactor.
 * @param[in] p The row number of the entry value.
 * @param[in] q The column number of the entry value.
 * @return The cofactor of given matrix and entry position.
 * @note This function is used to compute the determinant in Matrix::Det().
 */
template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows - 1, cols - 1> GetCofactor(const Matrix<T, rows, cols>& matrix,
                                          const uint8_t p, const uint8_t q);

/**
 * Recursive function for finding determinant of matrix using cofactors approach.
 *
 * @param[in] matrix The matrix whose determinant is to be found.
 * @return The determinant of the matrix.
 */
template <typename T, uint8_t dim> T Det(const Matrix<T, dim, dim>& matrix);

/**
 * Returns _Cholesky factor_ of given symmetric, positive-definite matrix.
 *
 * Every symmetric, positive-definite matrix @f$\mathbf{A}@f$ can be decomposed into a
 * product of a unique lower triangular matrix @f$\mathbf{L}@f$ and its transpose:
 * @f[
 * \mathbf{A} = \mathbf{L}\mathbf{L}^T
 * @f]
 * where @f$\mathbf{L}@f$ is called the _Cholesky factor_ of @f$\mathbf{A}@f$, and can be
 * interpreted as a generalized square root of @f$\mathbf{A}@f$.
 * @param[in] matrix The matrix @f$\mathbf{A}@f$ whose Cholesky factor is to be found.
 * @return The Cholesky factor @f$\mathbf{L}@f$ of @f$\mathbf{A}@f$.
 * @note This function is used to check whether a matrix is positive-definite by
 * Matrix::IsPositiveDefinite().
 */
template <typename T, uint8_t dim>
MatrixXd<dim, dim> Cholesky(const Matrix<T, dim, dim>& matrix);

/**
 * Mean value of a vector.
 *
 * @param[in] vvect A m-dimensional vertical vector.
 * @return A scalar value.
 */
template <typename T, uint8_t dim> double Mean(const VectorX<T, dim>& vvect);

/**
 * Standard deviation of a vector.
 *
 * @param[in] vvect A m-dimensional vertical vector.
 * @return A scalar value.
 */
template <typename T, uint8_t dim> double Std(const VectorX<T, dim>& vvect);

} //  end namespace grabnum

// This is a trick to define templated functions in a source file.
#include "../src/matrix_utilities.tcc"

#endif // GRABCOMMON_LIBNUMERIC_MATRIX_UTILITIES_H
