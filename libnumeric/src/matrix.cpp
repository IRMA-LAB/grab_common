/**
 * @file matrix.cpp
 * @author Edoardo Idà, Simone Comari
 * @date 13 Mar 2019
 * @brief File containing definitions and implementation of matrix class.
 */

#include <assert.h>

#include "matrix.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)                                                                    \
  (sizeof((x)) / sizeof((x)[0])) /**< returns the size of a standard array*/
#endif

namespace grabnum {

//----- Constructors -----------------------------------------------------------------//

template <typename T, uint8_t rows, uint8_t cols> Matrix<T, rows, cols>::Matrix()
{
  SetZero();
}

template <typename T, uint8_t rows, uint8_t cols> Matrix<T, rows, cols>::Matrix(T scalar)
{
  if (scalar == 0)
    SetZero();
  else
    *this = SetIdentity() * scalar;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>::Matrix(const T* values, const uint16_t size)
{
  Fill(values, size);
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>::Matrix(const std::vector<T>& values)
{
  Fill(values);
}

template <typename T, uint8_t rows, uint8_t cols>
template <typename T2>
Matrix<T, rows, cols>::Matrix(const Matrix<T2, rows, cols>& other)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] = static_cast<T>(other(row + 1, col + 1));
}

//----- Operator Overloadings --------------------------------------------------------//

template <typename T, uint8_t rows, uint8_t cols>
template <typename NewT>
Matrix<T, rows, cols>::operator Matrix<NewT, rows, cols>()
{
  Matrix<NewT, rows, cols> result(*this);
  return result;
}

template <typename T, uint8_t rows, uint8_t cols>
template <typename NewT>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::
operator=(const Matrix<NewT, rows, cols>& other)
{
  *this = static_cast<Matrix<T, rows, cols>>(other);
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
bool Matrix<T, rows, cols>::operator==(const Matrix<T, rows, cols>& other) const
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
    {
      if (elements_[row][col] != other(row + 1, col + 1))
        return false;
    }
  return true;
}

template <typename T, uint8_t rows, uint8_t cols>
bool Matrix<T, rows, cols>::operator!=(const Matrix<T, rows, cols>& other) const
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
    {
      if (elements_[col][row] != other.elements_[col][row])
        return true;
    }
  return false;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::operator+=(const T& scalar)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] += scalar;
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::
operator+=(const Matrix<T, rows, cols>& other)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] += other(row + 1, col + 1);
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::operator-=(const T& scalar)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] -= scalar;
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::
operator-=(const Matrix<T, rows, cols>& other)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] -= other(row + 1, col + 1);
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::operator*=(const T& scalar)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] *= scalar;
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::operator/=(const T& scalar)
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] /= scalar;
  return *this;
}

//----- Setters ----------------------------------------------------------------------//

template <typename T, uint8_t rows, uint8_t cols>
template <uint8_t block_rows, uint8_t block_cols>
Matrix<T, rows, cols>&
Matrix<T, rows, cols>::SetBlock(const uint8_t start_row, const uint8_t start_col,
                                const Matrix<T, block_rows, block_cols>& other)
{
  assert(start_row + block_rows - 1 <= rows);
  assert(start_col + block_cols - 1 <= cols);

  for (uint8_t row = start_row; row < start_row + block_rows; ++row)
    for (uint8_t col = start_col; col < start_col + block_cols; ++col)
      elements_[row - 1][col - 1] = other(row - start_row + 1, col - start_col + 1);
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetCol(const uint8_t cl,
                                                     const Matrix<T, rows, 1>& matrix1d)
{
  for (uint8_t i = 0; i < rows; ++i)
  {
    elements_[i][cl - 1] = matrix1d(i + 1);
  }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetCol(const uint8_t cl, const T* vect,
                                                     const uint8_t size)
{
  assert(size == rows);

  for (uint8_t i = 0; i < rows; ++i)
  {
    elements_[i][cl - 1] = vect[i];
  }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetCol(const uint8_t cl,
                                                     const std::vector<T>& vect)
{
  assert(vect.size() == rows);

  for (uint8_t i = 0; i < rows; ++i)
  {
    elements_[i][cl - 1] = vect[i];
  }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
template <uint8_t _rows, uint8_t _cols>
Matrix<T, rows, cols>&
Matrix<T, rows, cols>::SetFromBlock(const uint8_t start_row, const uint8_t start_col,
                                    const Matrix<T, _rows, _cols>& other)
{
  assert(start_row + rows - 1 <= _rows);
  assert(start_col + cols - 1 <= _cols);

  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] = other(row + start_row, col + start_col);
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetIdentity()
{
#if (MCU_TARGET == 0)
  if (!IsSquare())
  {
    std::cerr << "WARNING: Matrix is not square! Pseudo-identity matrix is generated."
              << std::endl;
  }
#endif
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
    {
      if (row == col)
        elements_[row][col] = 1;
      else
        elements_[row][col] = 0;
    }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetRow(const uint8_t rw,
                                                     const Matrix<T, 1, cols>& matrix1d)
{
  for (uint8_t i = 0; i < cols; ++i)
  {
    elements_[rw - 1][i] = matrix1d(i + 1);
  }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetRow(const uint8_t rw, const T* vect,
                                                     const uint8_t size)
{
  assert(size == cols);

  for (uint8_t i = 0; i < cols; ++i)
  {
    elements_[rw - 1][i] = vect[i];
  }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetRow(const uint8_t rw,
                                                     const std::vector<T>& vect)
{
  assert(vect.size() == cols);

  for (uint8_t i = 0; i < cols; ++i)
  {
    elements_[rw - 1][i] = vect[i];
  }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetZero()
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] = 0;
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::Fill(const T* values, const uint16_t size)
{
  assert(size == this->Size());

  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] = values[row * cols + col];
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::Fill(const std::vector<T>& values)
{
  assert(values.size() == this->Size());

  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      elements_[row][col] = values[row * cols + col];
  return *this;
}

//----- Matrix manipulation ----------------------------------------------------------//

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, cols, rows> Matrix<T, rows, cols>::Transpose() const
{
  Matrix<T, cols, rows> transpose;
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
      transpose(col + 1, row + 1) = elements_[row][col];
  return transpose;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SwapRow(const uint8_t row1,
                                                      const uint8_t row2)
{
  T temp;
  for (uint8_t i = 0; i < cols; ++i)
  {
    temp                   = elements_[row1 - 1][i];
    elements_[row1 - 1][i] = elements_[row2 - 1][i];
    elements_[row2 - 1][i] = temp;
  }
  return *this;
}

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SwapCol(const uint8_t col1,
                                                      const uint8_t col2)
{
  T temp;
  for (uint8_t i = 0; i < rows; ++i)
  {
    temp                   = elements_[i][col1 - 1];
    elements_[i][col1 - 1] = elements_[i][col2 - 1];
    elements_[i][col2 - 1] = temp;
  }
  return *this;
}

//----- Getters ----------------------------------------------------------------------//

template <typename T, uint8_t rows, uint8_t cols>
Matrix<T, 1, cols> Matrix<T, rows, cols>::GetRow(const uint8_t row) const
{
  Matrix<T, 1, cols> row_vect;
  for (uint8_t col = 0; col < cols; ++col)
    row_vect(col + 1) = elements_[row - 1][col];
  return row_vect;
}

template <typename T, uint8_t rows, uint8_t cols>
VectorX<T, rows> Matrix<T, rows, cols>::GetCol(const uint8_t col) const
{
  VectorX<T, rows> col_vect;
  for (uint8_t row = 0; row < rows; ++row)
    col_vect(row + 1) = elements_[row][col - 1];
  return col_vect;
}

template <typename T, uint8_t rows, uint8_t cols>
template <uint8_t blk_rows, uint8_t blk_cols>
Matrix<T, blk_rows, blk_cols>
Matrix<T, rows, cols>::GetBlock(const uint8_t start_row, const uint8_t start_col) const
{
  Matrix<T, blk_rows, blk_cols> block;
  block.SetFromBlock(start_row, start_col, *this);
  return block;
}

//----- Check functions --------------------------------------------------------------//

template <typename T, uint8_t rows, uint8_t cols>
bool Matrix<T, rows, cols>::IsSquare() const
{
  return rows == cols;
}

template <typename T, uint8_t rows, uint8_t cols>
bool Matrix<T, rows, cols>::IsSymmetric() const
{
  if (!IsSquare())
    return false;
  return *this == Transpose();
}

template <typename T, uint8_t rows, uint8_t cols>
bool Matrix<T, rows, cols>::IsPositiveDefinite() const
{
  if (!IsSymmetric())
    return false;
  try
  {
    Cholesky(*this);
    return true;
  }
  catch (std::invalid_argument)
  {
    return false;
  }
}

template <typename T, uint8_t rows, uint8_t cols>
bool Matrix<T, rows, cols>::IsApprox(const Matrix<T, rows, cols>& other,
                                     const double tol /* = epsilon*/) const
{
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
    {
      if (!IsClose(elements_[row][col], other(row + 1, col + 1), tol))
        return false;
    }
  return true;
}

//----- Matrix internal utilities ----------------------------------------------------//

template <typename T, uint8_t rows, uint8_t cols>
uint16_t Matrix<T, rows, cols>::MaxIdx() const
{
  uint16_t index;
  uint8_t i_max = 0;
  uint8_t j_max = 0;
  T max         = elements_[0][0];
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
    {
      if (elements_[row][col] > max)
      {
        max   = elements_[row][col];
        i_max = row;
        j_max = col;
      }
    }
  index = i_max * cols + j_max + 1;
  return index;
}

template <typename T, uint8_t rows, uint8_t cols>
uint16_t Matrix<T, rows, cols>::MinIdx() const
{
  uint16_t index;
  uint8_t i_min = 0;
  uint8_t j_min = 0;
  T min         = elements_[0][0];
  for (uint8_t row = 0; row < rows; ++row)
    for (uint8_t col = 0; col < cols; ++col)
    {
      if (elements_[row][col] < min)
      {
        min   = elements_[row][col];
        i_min = row;
        j_min = col;
      }
    }
  index = i_min * cols + j_min + 1;
  return index;
}

} // end namespace grabnum
