/**
 * @file matrix_utilities.tcc
 * @author Edoardo Idà, Simone Comari
 * @date 30 Aug 2019
 * @brief File containing definitions and implementation of matrix utilities.
 */

#ifndef GRABCOMMON_LIBNUMERIC_MATRIX_UTILITIES_H
#error Do not include this file directly, include matrix_utilities.h instead
#endif

#include "matrix.h"

namespace grabnum {

#ifndef GRABNUM_MCU_TARGET
template <typename T, uint rows, uint cols>
std::ostream& operator<<(std::ostream& stream, const Matrix<T, rows, cols>& matrix)
{

  for (uint row = 1; row <= rows; ++row)
  {
    if (row == 1)
      stream << "[";
    else
      stream << " ";
    for (uint col = 1; col <= cols; ++col)
    {
      stream << std::setw(15) << std::setprecision(7) << matrix(row, col);
      if (row == rows && col == cols)
        stream << "          ]";
    }
    stream << "\n";
  }
  return stream;
}
#endif

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols> operator+(const T& scalar, const Matrix<T, rows, cols>& matrix)
{
  Matrix<T, rows, cols> sum;
  for (uint row = 1; row <= rows; ++row)
    for (uint col = 1; col <= cols; ++col)
      sum(row, col) = matrix(row, col) + scalar;
  return sum;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols> operator+(const Matrix<T, rows, cols>& matrix, const T& scalar)
{
  Matrix<T, rows, cols> sum;
  for (uint row = 1; row <= rows; ++row)
    for (uint col = 1; col <= cols; ++col)
      sum(row, col) = matrix(row, col) + scalar;
  return sum;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols> operator+(const Matrix<T, rows, cols>& matrix1,
                                const Matrix<T, rows, cols>& matrix2)
{
  Matrix<T, rows, cols> sum;
  for (uint row = 1; row <= rows; ++row)
    for (uint col = 1; col <= cols; ++col)
      sum(row, col) = matrix1(row, col) + matrix2(row, col);
  return sum;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols> operator-(const T& scalar, const Matrix<T, rows, cols>& matrix)
{
  Matrix<T, rows, cols> result;
  for (uint row = 1; row <= rows; ++row)
    for (uint col = 1; col <= cols; ++col)
      result(row, col) = scalar - matrix(row, col);
  return result;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols> operator-(const Matrix<T, rows, cols>& matrix, const T& scalar)
{
  Matrix<T, rows, cols> diff;
  for (uint row = 1; row <= rows; ++row)
    for (uint col = 1; col <= cols; ++col)
      diff(row, col) = matrix(row, col) - scalar;
  return diff;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols> operator-(const Matrix<T, rows, cols>& matrix1,
                                const Matrix<T, rows, cols>& matrix2)
{
  Matrix<T, rows, cols> diff;
  for (uint row = 1; row <= rows; ++row)
    for (uint col = 1; col <= cols; ++col)
      diff(row, col) = matrix1(row, col) - matrix2(row, col);
  return diff;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols> operator-(const Matrix<T, rows, cols>& matrix)
{
  Matrix<T, rows, cols> opposite;
  for (uint row = 1; row <= rows; ++row)
    for (uint col = 1; col <= cols; ++col)
      opposite(row, col) = -matrix(row, col);
  return opposite;
}

template <typename T, uint rows1, uint dim_common, uint cols2>
Matrix<T, rows1, cols2> operator*(const Matrix<T, rows1, dim_common>& matrix1,
                                  const Matrix<T, dim_common, cols2>& matrix2)
{
  Matrix<T, rows1, cols2> prod;
  for (uint row = 1; row <= rows1; ++row)
  {
    for (uint col = 1; col <= cols2; ++col)
    {
      T sum = 0;
      for (uint j = 1; j <= dim_common; j++)
        sum += matrix1(row, j) * matrix2(j, col);
      prod(row, col) = sum;
    }
  }
  return prod;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols> operator*(const VectorX<T, rows>& vvect,
                                const Matrix<T, 1, cols>& hvect)
{
  Matrix<T, rows, cols> prod;
  for (uint row = 1; row <= rows; ++row)
  {
    for (uint col = 1; col <= cols; ++col)
    {
      prod(row, col) = vvect(row) * hvect(col);
    }
  }
  return prod;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols> operator*(const T& scalar, const Matrix<T, rows, cols>& matrix)
{
  Matrix<T, rows, cols> prod;
  for (uint row = 1; row <= rows; ++row)
    for (uint col = 1; col <= cols; ++col)
      prod(row, col) = matrix(row, col) * scalar;
  return prod;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols> operator*(const Matrix<T, rows, cols>& matrix, const T& scalar)
{
  Matrix<T, rows, cols> prod;
  for (uint row = 1; row <= rows; ++row)
    for (uint col = 1; col <= cols; ++col)
      prod(row, col) = matrix(row, col) * scalar;
  return prod;
}

template <typename T, uint rows>
VectorX<T, rows> operator*(const VectorX<T, rows>& vvect1, const VectorX<T, rows>& vvect2)
{
  VectorX<T, rows> prod;
  for (uint row = 1; row <= rows; ++row)
    prod(row) = vvect1(row) * vvect2(row);
  return prod;
}

template <typename T, uint cols>
Matrix<T, 1, cols> operator*(const Matrix<T, 1, cols>& hvect1,
                             const Matrix<T, 1, cols>& hvect2)
{
  Matrix<T, 1, cols> prod;
  for (uint col = 1; col <= cols; ++col)
    prod(col) = hvect1(col) * hvect2(col);
  return prod;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols> operator/(const Matrix<T, rows, cols>& matrix, const T& scalar)
{
  Matrix<T, rows, cols> result;
  for (uint row = 1; row <= rows; ++row)
    for (uint col = 1; col <= cols; ++col)
      result(row, col) = matrix(row, col) / scalar;
  return result;
}

template <typename T, uint rows, uint cols1, uint cols2>
Matrix<T, rows, cols1 + cols2> HorzCat(const Matrix<T, rows, cols1>& matrix_lx,
                                       const Matrix<T, rows, cols2>& matrix_rx)
{
  Matrix<T, rows, cols1 + cols2> result;

  for (uint row = 1; row <= rows; ++row)
  {
    uint col;
    for (col = 1; col <= cols1; ++col)
      result(row, col) = matrix_lx(row, col);
    for (col = 1; col <= cols2; ++col)
      result(row, col + cols1) = matrix_rx(row, col);
  }
  return result;
}

template <typename T, uint rows1, uint rows2, uint cols>
Matrix<T, rows1 + rows2, cols> VertCat(const Matrix<T, rows1, cols>& matrix_up,
                                       const Matrix<T, rows2, cols>& matrix_down)
{
  Matrix<T, rows1 + rows2, cols> result;
  for (uint col = 1; col <= cols; ++col)
  {
    uint row;
    for (row = 1; row <= rows1; ++row)
      result(row, col) = matrix_up(row, col);
    for (row = 1; row <= rows2; ++row)
      result(row + rows1, col) = matrix_down(row, col);
  }
  return result;
}

template <typename T, uint dim>
T Dot(const VectorX<T, dim>& vvect1, const VectorX<T, dim>& vvect2)
{
  T result = 0;
  for (uint i = 1; i <= dim; ++i)
  {
    result += vvect1(i) * vvect2(i);
  }
  return result;
}

template <typename T, uint dim>
T Dot(const Matrix<T, 1, dim>& hvect, const VectorX<T, dim>& vvect)
{
  T result = 0;
  for (uint i = 1; i <= dim; ++i)
  {
    result += hvect(i) * vvect(i);
  }
  return result;
}

template <typename T, uint dim> double Norm(const VectorX<T, dim>& vvect)
{
  T result = 0;
  for (uint i = 1; i <= dim; ++i)
    result += vvect(i) * vvect(i);
  return sqrt(result);
}

template <typename T, uint dim> double Norm(const Matrix<T, 1, dim>& hvect)
{
  T result = 0;
  for (uint i = 1; i <= dim; ++i)
    result += hvect(i) * hvect(i);
  return sqrt(result);
}

template <typename T>
Vector3<T> Cross(const Vector3<T>& vvect3d1, const Vector3<T>& vvect3d2)
{
  return Skew(vvect3d1) * vvect3d2;
}

template <typename T> Matrix3<T> Skew(const Vector3<T>& vvect3d)
{
  Matrix3<T> result;
  result(1, 1) = 0;
  result(2, 2) = 0;
  result(3, 3) = 0;
  result(1, 2) = -vvect3d(3);
  result(2, 1) = vvect3d(3);
  result(1, 3) = vvect3d(2);
  result(3, 1) = -vvect3d(2);
  result(2, 3) = -vvect3d(1);
  result(3, 2) = vvect3d(1);
  return result;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows - 1, cols - 1> GetCofactor(const Matrix<T, rows, cols>& matrix,
                                          const uint p, const uint q)
{
  uint i = 1, j = 1;
  Matrix<T, rows - 1, cols - 1> cofactor;

  // Looping for each element of the matrix
  for (uint row = 1; row <= rows; ++row)
  {
    for (uint col = 1; col <= cols; ++col)
    {
      // Copying into temporary matrix only those element which are not in given row and
      // column
      if (row == p || col == q)
        continue;
      cofactor(i, j++) = matrix(row, col);
      // Row is filled, so increase row index and reset col index
      if (j != cols)
        continue;
      j = 1;
      i++;
    }
  }
  return cofactor;
}

template <typename T, uint dim> T Det(const Matrix<T, dim, dim>& matrix)
{
  //  Base case : if matrix contains single element
  if (dim == 1)
    return matrix(1, 1);

  int8_t sign = 1; // to store sign multiplier
  T det       = 0; // initialize result
  // Iterate for each element of first row
  for (uint col = 1; col <= dim; ++col)
  {
    // Getting Cofactor of mat[0][f]
    Matrix<T, dim - 1, dim - 1> temp = GetCofactor(matrix, 1, col);
    det += sign * matrix(1, col) * Det(temp);
    // Terms are to be added with alternate sign
    sign = -sign;
  }
  return det;
}

template <typename T, uint dim> MatrixXd<dim, dim> Cholesky(const Matrix<T, dim, dim>& A)
{
  MatrixXd<dim, dim> L;
  // Build lower triangular matrix
  for (uint i = 1; i <= dim; ++i)
    for (uint j = 1; j <= i; ++j)
    {
      double s = 0.0;
      for (uint k = 1; k < j; ++k)
        s += L(i, k) * L(j, k);
      if (i == j)
      {
        if (A(i, i) - s <= 0)
          throw std::invalid_argument("Matrix must be positive-definite!");
        L(i, j) = sqrt(A(i, i) - s);
      }
      else
        L(i, j) = 1.0 / L(j, j) * (A(i, j) - s);
    }
  return L;
}

template <typename T, uint dim>
MatrixXd<dim, dim> Inverse(const Matrix<T, dim, dim>& matrix)
{
  MatrixXd<dim, dim> matrix_inv = matrix;
  float ratio, a;
  uint i, j, k, n;
  for (i = 0; i < n; i++)
    for (j = n; j < 2 * n; j++)
      if (i == (j - n))
        matrix_inv[i][j] = 1.0;
      else
        matrix_inv[i][j] = 0.0;
  for (i = 0; i < n; i++)
    for (j = 0; j < n; j++)
      if (i != j)
      {
        ratio = matrix_inv[j][i] / matrix_inv[i][i];
        for (k = 0; k < 2 * n; k++)
          matrix_inv[j][k] -= ratio * matrix_inv[i][k];
      }
  for (i = 0; i < n; i++)
  {
    a = matrix_inv[i][i];
    for (j = 0; j < 2 * n; j++)
      matrix_inv[i][j] /= a;
  }
  return matrix_inv;
}

template <typename T, uint dim> double Mean(const VectorX<T, dim>& vvect)
{
  T sum = 0;
  for (uint i = 1; i <= dim; ++i)
    sum += vvect(i);
  return static_cast<double>(sum) / static_cast<double>(dim);
}

template <typename T, uint dim> double Std(const VectorX<T, dim>& vvect)
{
  double mean = Mean(vvect);
  double sum  = 0;
  for (uint i = 1; i <= dim; ++i)
    sum += SQUARE(vvect(i) - mean);
  return sqrt(static_cast<double>(sum) / static_cast<double>(dim));
}

} //  end namespace grabnum
