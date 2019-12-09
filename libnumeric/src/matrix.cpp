/**
 * @file matrix.cpp
 * @author Edoardo Idà, Simone Comari
 * @date 30 Aug 2019
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

template <typename T, uint rows, uint cols> Matrix<T, rows, cols>::Matrix() { SetZero(); }

template <typename T, uint rows, uint cols> Matrix<T, rows, cols>::Matrix(T scalar)
{
  if (scalar == 0)
    SetZero();
  else
    *this = SetIdentity() * scalar;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>::Matrix(const T* values, const uint16_t size)
{
  Fill(values, size);
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>::Matrix(const std::vector<T>& values)
{
  Fill(values);
}

template <typename T, uint rows, uint cols>
template <class IteratorType>
Matrix<T, rows, cols>::Matrix(IteratorType it, IteratorType end)
{
  Fill(it, end);
}

template <typename T, uint rows, uint cols>
template <typename T2>
Matrix<T, rows, cols>::Matrix(const Matrix<T2, rows, cols>& other)
{
  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
      elements_[row][col] = static_cast<T>(other(row + 1, col + 1));
}

//----- Operator Overloadings --------------------------------------------------------//

template <typename T, uint rows, uint cols>
template <typename NewT>
Matrix<T, rows, cols>::operator Matrix<NewT, rows, cols>()
{
  Matrix<NewT, rows, cols> result(*this);
  return result;
}

template <typename T, uint rows, uint cols>
template <typename NewT>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::
operator=(const Matrix<NewT, rows, cols>& other)
{
  *this = static_cast<Matrix<T, rows, cols>>(other);
  return *this;
}

template <typename T, uint rows, uint cols>
bool Matrix<T, rows, cols>::operator==(const Matrix<T, rows, cols>& other) const
{
  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
    {
      if (elements_[row][col] != other(row + 1, col + 1))
        return false;
    }
  return true;
}

template <typename T, uint rows, uint cols>
bool Matrix<T, rows, cols>::operator!=(const Matrix<T, rows, cols>& other) const
{
  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
    {
      if (elements_[col][row] != other.elements_[col][row])
        return true;
    }
  return false;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::operator+=(const T& scalar)
{
  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
      elements_[row][col] += scalar;
  return *this;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::
operator+=(const Matrix<T, rows, cols>& other)
{
  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
      elements_[row][col] += other(row + 1, col + 1);
  return *this;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::operator-=(const T& scalar)
{
  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
      elements_[row][col] -= scalar;
  return *this;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::
operator-=(const Matrix<T, rows, cols>& other)
{
  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
      elements_[row][col] -= other(row + 1, col + 1);
  return *this;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::operator*=(const T& scalar)
{
  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
      elements_[row][col] *= scalar;
  return *this;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::operator/=(const T& scalar)
{
  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
      elements_[row][col] /= scalar;
  return *this;
}

//----- Setters ----------------------------------------------------------------------//

template <typename T, uint rows, uint cols>
template <uint block_rows, uint block_cols>
Matrix<T, rows, cols>&
Matrix<T, rows, cols>::SetBlock(const uint start_row, const uint start_col,
                                const Matrix<T, block_rows, block_cols>& other)
{
  assert(start_row + block_rows - 1 <= rows);
  assert(start_col + block_cols - 1 <= cols);

  for (uint row = start_row; row < start_row + block_rows; ++row)
    for (uint col = start_col; col < start_col + block_cols; ++col)
      elements_[row - 1][col - 1] = other(row - start_row + 1, col - start_col + 1);
  return *this;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetCol(const uint cl,
                                                     const Matrix<T, rows, 1>& matrix1d)
{
  for (uint i = 0; i < rows; ++i)
  {
    elements_[i][cl - 1] = matrix1d(i + 1);
  }
  return *this;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetCol(const uint cl, const T* vect,
                                                     const uint size)
{
  assert(size == rows);

  for (uint i = 0; i < rows; ++i)
  {
    elements_[i][cl - 1] = vect[i];
  }
  return *this;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetCol(const uint cl,
                                                     const std::vector<T>& vect)
{
  assert(vect.size() == rows);

  for (uint i = 0; i < rows; ++i)
  {
    elements_[i][cl - 1] = vect[i];
  }
  return *this;
}

template <typename T, uint rows, uint cols>
template <uint _rows, uint _cols>
Matrix<T, rows, cols>&
Matrix<T, rows, cols>::SetFromBlock(const uint start_row, const uint start_col,
                                    const Matrix<T, _rows, _cols>& other)
{
  assert(start_row + rows - 1 <= _rows);
  assert(start_col + cols - 1 <= _cols);

  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
      elements_[row][col] = other(row + start_row, col + start_col);
  return *this;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetIdentity()
{
#ifdef GRABNUM_PRINT_ERRORS
  if (!IsSquare())
  {
    std::cerr
      << "WARNING: grabnum::Matrix is not square! Pseudo-identity matrix is generated."
      << std::endl;
  }
#endif
  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
    {
      if (row == col)
        elements_[row][col] = 1;
      else
        elements_[row][col] = 0;
    }
  return *this;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetRow(const uint rw,
                                                     const Matrix<T, 1, cols>& matrix1d)
{
  assert(rw > 0);

  for (uint i = 0; i < cols; ++i)
    elements_[rw - 1][i] = matrix1d(i + 1);
  return *this;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetRow(const uint rw, const T* vect,
                                                     const uint size)
{
  assert(rw > 0 && size == cols);

  for (uint i = 0; i < cols; ++i)
    elements_[rw - 1][i] = vect[i];
  return *this;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetRow(const uint rw,
                                                     const std::vector<T>& vect)
{
  assert(rw > 0 && vect.size() == cols);

  for (uint i = 0; i < cols; ++i)
    elements_[rw - 1][i] = vect[i];
  return *this;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SetZero()
{
  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
      elements_[row][col] = 0;
  return *this;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::Fill(const T* values, const uint16_t size)
{
  assert(size == this->Size());

  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
      elements_[row][col] = values[row * cols + col];
  return *this;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::Fill(const std::vector<T>& values)
{
  assert(values.size() == this->Size());

  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
      elements_[row][col] = values[row * cols + col];
  return *this;
}

template <typename T, uint rows, uint cols>
template <class IteratorType>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::Fill(IteratorType it, IteratorType end)
{
  assert(end - it == this->Size());

  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
      elements_[row][col] = *(it++);
  return *this;
}

//----- Matrix manipulation ----------------------------------------------------------//

template <typename T, uint rows, uint cols>
Matrix<T, cols, rows> Matrix<T, rows, cols>::Transpose() const
{
  Matrix<T, cols, rows> transpose;
  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
      transpose(col + 1, row + 1) = elements_[row][col];
  return transpose;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SwapRow(const uint row1, const uint row2)
{
  T temp;
  for (uint i = 0; i < cols; ++i)
  {
    temp                   = elements_[row1 - 1][i];
    elements_[row1 - 1][i] = elements_[row2 - 1][i];
    elements_[row2 - 1][i] = temp;
  }
  return *this;
}

template <typename T, uint rows, uint cols>
Matrix<T, rows, cols>& Matrix<T, rows, cols>::SwapCol(const uint col1, const uint col2)
{
  T temp;
  for (uint i = 0; i < rows; ++i)
  {
    temp                   = elements_[i][col1 - 1];
    elements_[i][col1 - 1] = elements_[i][col2 - 1];
    elements_[i][col2 - 1] = temp;
  }
  return *this;
}

//----- Getters ----------------------------------------------------------------------//

template <typename T, uint rows, uint cols>
Matrix<T, 1, cols> Matrix<T, rows, cols>::GetRow(const uint row) const
{
  assert(row > 0 && row <= rows);

  Matrix<T, 1, cols> row_vect;
  for (uint col = 0; col < cols; ++col)
    row_vect(col + 1) = elements_[row - 1][col];
  return row_vect;
}

template <typename T, uint rows, uint cols>
template <uint num_rows>
Matrix<T, num_rows, cols> Matrix<T, rows, cols>::GetRows(const uint start_row) const
{
  assert(start_row > 0 && start_row + num_rows - 1 <= rows);

  Matrix<T, num_rows, cols> block_rows;
  for (uint i = start_row; i < start_row + num_rows; ++i)
    block_rows.SetRow(i, this->GetRow(i));
  return block_rows;
}

template <typename T, uint rows, uint cols>
template <uint num_rows>
Matrix<T, num_rows, cols> Matrix<T, rows, cols>::HeadRows() const
{
  return GetRows<num_rows>(1);
}

template <typename T, uint rows, uint cols>
template <uint num_rows>
Matrix<T, num_rows, cols> Matrix<T, rows, cols>::TailRows() const
{
  return GetRows<num_rows>(rows - num_rows + 1);
}

template <typename T, uint rows, uint cols>
VectorX<T, rows> Matrix<T, rows, cols>::GetCol(const uint col) const
{
  assert(col > 0 && col <= cols);

  VectorX<T, rows> col_vect;
  for (uint row = 0; row < rows; ++row)
    col_vect(row + 1) = elements_[row][col - 1];
  return col_vect;
}

template <typename T, uint rows, uint cols>
template <uint num_cols>
Matrix<T, rows, num_cols> Matrix<T, rows, cols>::GetCols(const uint start_col) const
{
  assert(start_col > 0 && start_col + num_cols - 1 <= cols);

  Matrix<T, rows, num_cols> block_cols;
  for (uint i = start_col; i < start_col + num_cols; ++i)
    block_cols.SetCol(i, this->GetCol(i));
  return block_cols;
}

template <typename T, uint rows, uint cols>
template <uint num_cols>
Matrix<T, rows, num_cols> Matrix<T, rows, cols>::HeadCols() const
{
  return GetCols<num_cols>(1);
}

template <typename T, uint rows, uint cols>
template <uint num_cols>
Matrix<T, rows, num_cols> Matrix<T, rows, cols>::TailCols() const
{
  return GetCols<num_cols>(cols - num_cols + 1);
}

template <typename T, uint rows, uint cols>
template <uint blk_rows, uint blk_cols>
Matrix<T, blk_rows, blk_cols> Matrix<T, rows, cols>::GetBlock(const uint start_row,
                                                              const uint start_col) const
{
  assert(start_row > 0 && start_col > 0);

  Matrix<T, blk_rows, blk_cols> block;
  block.SetFromBlock(start_row, start_col, *this);
  return block;
}

//----- Check functions --------------------------------------------------------------//

template <typename T, uint rows, uint cols> bool Matrix<T, rows, cols>::IsSquare() const
{
  return rows == cols;
}

template <typename T, uint rows, uint cols>
bool Matrix<T, rows, cols>::IsSymmetric() const
{
  if (!IsSquare())
    return false;
  return *this == Transpose();
}

template <typename T, uint rows, uint cols>
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

template <typename T, uint rows, uint cols>
bool Matrix<T, rows, cols>::IsApprox(const Matrix<T, rows, cols>& other,
                                     const double tol /* = epsilon*/) const
{
  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
    {
      if (!IsClose(elements_[row][col], other(row + 1, col + 1), tol))
        return false;
    }
  return true;
}

//----- Matrix internal utilities ----------------------------------------------------//

template <typename T, uint rows, uint cols> uint16_t Matrix<T, rows, cols>::MaxIdx() const
{
  uint16_t index;
  uint i_max = 0;
  uint j_max = 0;
  T max      = elements_[0][0];
  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
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

template <typename T, uint rows, uint cols> uint16_t Matrix<T, rows, cols>::MinIdx() const
{
  uint16_t index;
  uint i_min = 0;
  uint j_min = 0;
  T min      = elements_[0][0];
  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
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

template <typename T, uint rows, uint cols>
std::vector<std::vector<uint>> Matrix<T, rows, cols>::NonZeros() const
{
  std::vector<std::vector<uint>> indeces;
  for (uint row = 0; row < rows; ++row)
    for (uint col = 0; col < cols; ++col)
      if (elements_[row][col] != 0)
        indeces.push_back({row + 1, col + 1});
  return indeces;
}

} // end namespace grabnum
