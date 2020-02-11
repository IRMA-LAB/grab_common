/**
 * @file matrix.h
 * @author Edoardo Idà, Simone Comari
 * @date 11 Feb 2020
 * @brief File containing matrix class and utilities to be included in the GRAB numeric
 * library.
 *
 * In this file a simple implementation of a class object is provided, with its basic
 * functionalities. All elements of this file are templated to be compatible with all
 * different types and leave the user more flexibility.
 */

#ifndef GRABCOMMON_LIBNUMERIC_MATRIX_H_
#define GRABCOMMON_LIBNUMERIC_MATRIX_H_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <typeinfo>

#include "common.h"

#ifndef GRABNUM_MCU_TARGET
#include <iomanip>
#include <iostream>
#endif

/**
 * @brief Namespace for GRAB numeric library.
 */
namespace grabnum {

/**
 * A Matlab-alike implementation of a matrix class to simplify and speed up standard
 * operations and make it MCU-friendly.
 *
 * @note Indexing starts at 1 instead of 0 (like in Matlab)!
 */
template <typename T, uint rows, uint cols> class Matrix
{
 public:
  using Matrix_t = Matrix<T, rows, cols>; /**< practical typedef for local use */

  /**
   * Default empty constructor.
   *
   * @todo initialize to zero anyway
   */
  Matrix();
  /**
   * Constructor for empty or identity matrix.
   *
   * @param[in] scalar A scalar value to be duplicated on the diagonal of the matrix. Use
   * 0 for initializing an empty matrix.
   * @see SetZero()
   * @see SetIdentity()
   */
  Matrix(T scalar);
  /**
   * Full constructor.
   * Fills the matrix row-by-row with the elements of @a values.
   *
   * @param[in] values A constant pointer to a constant @a T vector.
   * @param[in] size The number of elements in the vector.
   * @see Fill()
   */
  Matrix(const T* values, const uint16_t size);
  /**
   * Full constructor.
   * Fills the matrix row-by-row with the elements of @a values.
   *
   * @param[in] values A standard @a T vector.
   * @see Fill()
   */
  Matrix(const std::vector<T>& values);
  /**
   * Full constructor.
   * Fills the matrix row-by-row with the elements of @a values.
   *
   * @param[in] it Iterator pointing at the beginning of the array data.
   * @param[in] end Iterator pointing at the end of the array data.
   * @see Fill()
   */
  template <class IteratorType> Matrix(IteratorType it, IteratorType end);
  /**
   * Parametrized constructor from another matrix with the same shape and size.
   * Makes a copy of the given matrix. It also handle automatic casting in case of
   * different types.
   *
   * @param[in] other The copied matrix.
   */
  template <typename T2> Matrix(const Matrix<T2, rows, cols>& other);

  /**
   * Returns numbers of rows.
   *
   * @return A size.
   */
  uint Rows() const { return rows; }
  /**
   * Returns numbers of rows.
   *
   * @return A size.
   */
  uint Cols() const { return cols; }
  /**
   * Returns the matrix size, i.e. @f$m\times n@f$.
   *
   * @return A size.
   */
  uint16_t Size() const { return rows * cols; }
  /**
   * Returns the maximum value inside the matrix.
   *
   * @return The maximum value.
   */
  inline T Max() const { return this(MaxIdx()); }
  /**
   * Returns the minimum value inside the matrix.
   *
   * @return The minimum value.
   */
  inline T Min() const { return this(MinIdx()); }
  /**
   * Returns the linear index of the maximum value inside the matrix.
   *
   * @return The linear index of the maximum value.
   */
  uint16_t MaxIdx() const;
  /**
   * Returns the linear index of the minimum value inside the matrix.
   *
   * @return The linear index of the minimum value.
   */
  uint16_t MinIdx() const;
  /**
   * Returns the linear index corresponding to a standard double index.
   *
   * @return The linear index corresponding to a standard double index.
   */
  inline uint16_t LinIdx(const uint row, const uint col) const
  {
    return (row - 1) * cols + col;
  }
  /**
   * Returns the matrix type.
   *
   * @return A constant reference to @c std::type_info.
   * @note This can be useful to compare different matrix types, but can't be used to to
   *declare a variable, for instance.
   */
  const std::type_info& Type() const { return typeid(T); }
  /**
   * Give full access to the matrix data.
   *
   * @return A pointer to the data of the matrix.
   */
  inline T* Data() { return *elements_; }
  /**
   * Give read-only access to the matrix data.
   *
   * @return A pointer to the data of the matrix.
   */
  inline const T* Data() const { return *elements_; }
  /**
   * Returns a pointer to beginning of the data.
   *
   * @return A pointer to beginning of the data.
   * @see Data
   */
  inline T* Begin() { return this->Data(); }
  /**
   * Returns a constant pointer to beginning of the data.
   *
   * @return A constant pointer to beginning of the data.
   * @see Data
   */
  inline const T* Begin() const { return this->Data(); }
  /**
   * Returns a pointer to beginning of the data.
   *
   * @return A pointer to beginning of the data.
   * @see Data
   */
  inline T* End() { return this->Data() + this->Size(); }
  /**
   * Returns a constant pointer to beginning of the data.
   *
   * @return A constant pointer to beginning of the data.
   * @see Data
   */
  inline const T* End() const { return this->Data() + this->Size(); }

  /**
   * Conversion operator.
   */
  template <typename NewT> operator Matrix<NewT, rows, cols>();
  /**
   * Assign operator with conversion.
   *
   * @param[in] other The matrix to be copied and type-casted.
   * @return A reference to @c *this.
   */
  template <typename NewT>
  Matrix<T, rows, cols>& operator=(const Matrix<NewT, rows, cols>& other);
  /**
   * Give read-only access to a single entry of the matrix.
   *
   * param[in] row The row index of the desired entry.
   * param[in] col The column index of the desired entry.
   * @return The (@a row , @a col ) entry of the matrix.
   * @note Matrix indexing starts from 1 like in Matlab.
   */
  inline const T& operator()(const uint row, const uint column) const
  {
    return elements_[row - 1][column - 1];
  }
  /**
   * Give access to a single entry of the matrix.
   *
   * param[in] row The row index of the desired entry.
   * param[in] col The column index of the desired entry.
   * @return The (@a row , @a col ) entry of the matrix.
   * @note Matrix indexing starts from 1 like in Matlab.
   */
  inline T& operator()(const uint row, const uint column)
  {
    return elements_[row - 1][column - 1];
  }
  /**
   * Give read-only access to a single entry of the unraveled matrix.
   * This is particularly useful when accessing elements of a vector (aka 1D matrix).
   *
   * param[in] lin_index The linear index of the desired entry.
   * @return The i-th entry of the matrix.
   * @note Matrix indexing starts from 1 like in Matlab and read row-by-row,
   * top-to-bottom.
   */
  inline const T& operator()(const uint lin_index) const
  {
    uint row    = (lin_index - 1) / cols;
    uint column = (lin_index - 1) % cols;
    return elements_[row][column];
  }
  /**
   * Give access to a single entry of the unraveled matrix.
   * This is particularly useful when accessing elements of a vector (aka 1D matrix).
   *
   * param[in] lin_index The linear index of the desired entry.
   * @return The i-th entry of the matrix.
   * @note Matrix indexing starts from 1 like in Matlab and read row-by-row,
   * top-to-bottom.
   */
  inline T& operator()(const uint lin_index)
  {
    uint row    = (lin_index - 1) / cols;
    uint column = (lin_index - 1) % cols;
    return elements_[row][column];
  }

  /**
   * Operator for element-wise comparison (equal).
   *
   * @param[in] other The matrix to be compared against.
   * @return true if each element of @c *this and @a other are all exactly equal.
   * @warning When using floating point scalar values you probably should rather use a
   * fuzzy comparison such as IsApprox().
   * @see operator!=
   * @see IsApprox()
   */
  bool operator==(const Matrix_t& other) const;
  /**
   * Operator for element-wise comparison (different).
   *
   * @param[in] other The matrix to be compared against.
   * @return true if at least one element of @c *this and @a other are not exactly equal.
   * @warning When using floating point scalar values you probably should rather use a
   * fuzzy comparison such as IsApprox().
   * @see operator==
   * @see IsApprox()
   */
  bool operator!=(const Matrix_t& other) const;
  /**
   * Replaces @c *this by @c *this + the scalar value @a scalar.
   *
   * @param[in] scalar The scalar to be added to each element of the matrix.
   * @return A reference to @c *this.
   */
  Matrix_t& operator+=(const T& scalar);
  /**
   * Replaces @c *this by @c *this + @a other.
   *
   * @param[in] other The matrix to be added to @c *this.
   * @return A reference to @c *this.
   */
  Matrix_t& operator+=(const Matrix_t& other);
  /**
   * Replaces @c *this by @c *this - the scalar value @a scalar.
   *
   * @param[in] scalar The scalar to be subtracted to each element of the matrix.
   * @return A reference to @c *this.
   */
  Matrix_t& operator-=(const T& scalar);
  /**
   * Replaces @c *this by @c *this - @a other.
   *
   * @param[in] other The matrix to be subtracted to @c *this.
   * @return A reference to @c *this.
   */
  Matrix_t& operator-=(const Matrix_t& other);
  /**
   * Replaces @c *this by @c *this * the scalar value @a scalar.
   *
   * @param[in] scalar The factor by which each element of the matrix is multiplied..
   * @return A reference to @c *this.
   */
  Matrix_t& operator*=(const T& scalar);
  /**
   * Replaces @c *this by @c *this divided by the scalar value @a scalar.
   *
   * @param[in] scalar The factor by which each element of the matrix is divided.
   * @return A reference to @c *this.
   */
  Matrix_t& operator/=(const T& scalar);

  /**
   * Replaces a block of @c *this with the elements of @a other.
   *
   * @param[in] start_row The first row in the block.
   * @param[in] start_col The first column in the block.
   * @param[in] other The submatrix to be used to replace the block of  @c *this.
   * @return A reference to @c *this.
   * @note @a block_rows must be <= m - @a start_row, likewise @a block_cols <= n -
   * @a start_col, where @f$m\times n@f$ are the dimensions of @c *this. In other words,
   * @a other becomes a submatrix of @c *this.
   * @see SetFromBlock()
   * @todo example
   */
  template <uint block_rows, uint block_cols>
  Matrix<T, rows, cols>& SetBlock(const uint start_row, const uint start_col,
                                  const Matrix<T, block_rows, block_cols>& other);
  /**
   * Sets a column of @c *this with the elements of a 1D matrix.
   *
   * @param[in] col The index of the column to be replaced.
   * @param[in] matrix1d The 1D vertical matrix to be used to replace the column of  @c
   **this.
   * @return A reference to @c *this.
   */
  Matrix_t& SetCol(const uint col, const Matrix<T, rows, 1>& matrix1d);
  /**
   * Sets a column of @c *this with the elements of a standard vector.
   *
   * @param[in] col The index of the column to be replaced.
   * @param[in] vect The standard vector to be used to replace the column of  @c *this.
   * @param[in] size The length of the column vector. It needs to be the same as the
   *number of rows in the original matrix.
   * @return A reference to @c *this.
   */
  Matrix_t& SetCol(const uint col, const T* vect, const uint size);
  /**
   * Sets a column of @c *this with the elements of a standard vector.
   *
   * @param[in] col The index of the column to be replaced.
   * @param[in] vect The standard vector to be used to replace the column of  @c *this.
   * @return A reference to @c *this.
   */
  Matrix_t& SetCol(const uint col, const std::vector<T>& vect);
  /**
   * Replaces @c *this with a block of @a other.
   *
   * @param[in] start_row The first row in the block.
   * @param[in] start_col The first column in the block.
   * @param[in] other The matrix whose block is used to replace @c *this.
   * @return A reference to @c *this.
   * @note @a m must be <= @a _rows - @a start_row, likewise @a n <= @a _rows -
   * @a start_col, where @f$m\times n@f$ are the dimensions of @c *this. In other words,
   * @c *this becomes a submatrix of @a other.
   * @see SetBlock()
   * @todo example
   */
  template <uint _rows, uint _cols>
  Matrix<T, rows, cols>& SetFromBlock(const uint start_row, const uint start_col,
                                      const Matrix<T, _rows, _cols>& other);
  /**
   * Set the matrix to an identity matrix.
   *
   * @return A reference to @c *this.
   */
  Matrix_t& SetIdentity();
  /**
   * Sets a row of @c *this with the elements of a 1D matrix.
   *
   * @param[in] row The index of the row to be replaced.
   * @param[in] matrix1d The 1D horizontal matrix to be used to replace the row of  @c
   **this.
   * @return A reference to @c *this.
   */
  Matrix_t& SetRow(const uint row, const Matrix<T, 1, cols>& matrix1d);
  /**
   * Sets a row of @c *this with the elements of a standard vector.
   *
   * @param[in] row The index of the row to be replaced.
   * @param[in] vect A pointer to the vector to be used to replace the row of  @c *this.
   * @param[in] size The length of the row vector. It needs to be the same as the number
   *of rows in the original matrix.
   * @return A reference to @c *this.
   */
  Matrix_t& SetRow(const uint row, const T* vect, const uint size);
  /**
   * Sets a row of @c *this with the elements of a standard vector.
   *
   * @param[in] row The index of the row to be replaced.
   * @param[in] vect The standard vector to be used to replace the row of  @c *this.
   * @return A reference to @c *this.
   */
  Matrix_t& SetRow(const uint row, const std::vector<T>& vect);
  /**
   * Sets the matrix to an empty matrix.
   *
   * @return A reference to @c *this.
   */
  Matrix_t& SetZero();
  /**
   * Fills the matrix row-by-row with the elements of @a values.
   *
   * @param[in] values A constant pointer to a constant @a T vector.
   * @param[in] size The number of elements in the vector.
   * @return A reference to @c *this.
   */
  Matrix_t& Fill(const T* values, const uint16_t size);
  /**
   * Fills the matrix row-by-row with the elements of @a values.
   *
   * @param[in] values A reference to a standard @a T vector.
   * @return A reference to @c *this.
   */
  Matrix_t& Fill(const std::vector<T>& values);
  /**
   * Fills the matrix row-by-row with the elements of @a values.
   *
   * @param[in] it Iterator pointing at the beginning of the array data.
   * @param[in] end Iterator pointing at the end of the array data.
   * @return A reference to @c *this.
   */
  template <class IteratorType>
  Matrix<T, rows, cols>& Fill(IteratorType it, IteratorType end);

  /**
   * Returns the transposed matrix.
   *
   * @return The transposed matrix.
   */
  Matrix<T, cols, rows> Transpose() const;
  /**
   * Swaps two different rows of @c *this.
   *
   * @param[in] row1 The index of the first row to be swapped.
   * @param[in] row2 The index of the second row to be swapped.
   * @return A reference to @c *this.
   */
  Matrix_t& SwapRow(const uint row1, const uint row2);
  /**
   * Swaps two different columns of @c *this.
   *
   * @param[in] col1 The index of the first column to be swapped.
   * @param[in] col2 The index of the second column to be swapped.
   * @return A reference to @c *this.
   */
  Matrix_t& SwapCol(const uint col1, const uint col2);

  /**
   * Extracts a row from the matrix.
   *
   * @param[in] row The index of the row to be extracted.
   * @return A 1-dimensional matrix (aka an horizontal vector).
   */
  Matrix<T, 1, cols> GetRow(const uint row) const;
  /**
   * @brief Get multiple consequent rows from *this.
   * @param start_row The initial row to start counting from.
   * @return A num_rows x cols sub-matrix.
   */
  template <uint num_rows> Matrix<T, num_rows, cols> GetRows(const uint start_row) const;
  /**
   * @brief Get top num_rows rows from *this.
   * @return A num_rows x cols sub-matrix.
   */
  template <uint num_rows> Matrix<T, num_rows, cols> HeadRows() const;
  /**
   * @brief Get bottom num_rows rows from *this.
   * @return A num_rows x cols sub-matrix.
   */
  template <uint num_rows> Matrix<T, num_rows, cols> TailRows() const;
  /**
   * Extracts a column from the matrix.
   *
   * @param[in] col The index of the column to be extracted.
   * @return A 1-dimensional matrix (aka a vertical vector).
   */
  Matrix<T, rows, 1> GetCol(const uint col) const;
  /**
   * @brief Get multiple consequent columns from *this.
   * @param start_col The initial column to start counting from.
   * @return A rows x num_cols sub-matrix.
   */
  template <uint num_cols> Matrix<T, rows, num_cols> GetCols(const uint start_col) const;
  /**
   * @brief Get left num_cols columns from *this.
   * @return A rows x num_cols sub-matrix.
   */
  template <uint num_cols> Matrix<T, rows, num_cols> HeadCols() const;
  /**
   * @brief Get right num_cols columns from *this.
   * @return A rows x num_cols sub-matrix.
   */
  template <uint num_cols> Matrix<T, rows, num_cols> TailCols() const;
  /**
   * Extracts a block from the matrix.
   *
   * The number of rows and columns of the block are specified in the template arguments.
   * @param[in] start_row The starting row of the block.
   * @param[in] start_col The starting column of the block.
   * @return A block of the original matrix.
   */
  template <uint blk_rows, uint blk_cols>
  Matrix<T, blk_rows, blk_cols> GetBlock(const uint start_row,
                                         const uint start_col) const;

  /**
   * Check if the matrix is square, i.e. @a m = @a n.
   *
   * @return true if matrix is square.
   */
  bool IsSquare() const;
  /**
   * Check if the matrix is symmetric, i.e. @f$\mathbf{A} = \mathbf{A}^T@f$.
   *
   * @return true if matrix is symmetric.
   */
  bool IsSymmetric() const;
  /**
   * Check if the matrix is positive-definite using _Cholesky decomposition_.
   *
   * A generic symmetric matrix @f$\mathbf{A}@f$ is positive-definite matrix if and only
   * if it can be decomposed into a product of a unique lower triangular matrix
   * @f$\mathbf{L}@f$ and its transpose:
   * @f[
   * \mathbf{A} = \mathbf{L}\mathbf{L}^T
   * @f]
   * where @f$\mathbf{L}@f$ is called the _Cholesky factor_ of @f$\mathbf{A}@f$, and can
   * be interpreted as a generalized square root of @f$\mathbf{A}@f$.
   * @return true if matrix is positive-definite.
   * @see Cholesky()
   */
  bool IsPositiveDefinite() const;
  /**
   * Operator for fuzzy element-wise comparison (equal).
   *
   * @param[in] other The matrix to be compared against.
   * @param[in] tol (optional) The difference below which two single entries are
   * considered equal.
   * @return true if each element of @c *this and @a other are all equal up to a certain
   * tolerance.
   * @see operator!=
   */
  bool IsApprox(const Matrix<T, rows, cols>& other, const double tol = EPSILON) const;

  /**
   * Return a vector of 2D vectors containing the 2D index of non-zero values of @c *this.
   * @return A vector of 2D vectors containing the 2D index of non-zero values of @c
   * *this.
   */
  std::vector<std::vector<uint>> NonZeros() const;

 private:
  T elements_[rows][cols]; /**< for easy internal access to matrix elements. */
};

//----- Common typedef ---------------------------------------------------------------//

template <typename T> using Vector2 = Matrix<T, 2, 1>; /**< generic 2D vector */
using Vector2u                      = Vector2<uint>;   /**< 2D vector of unsigned int */
using Vector2i                      = Vector2<int>;    /**< 2D vector of int */
using Vector2l                      = Vector2<long>;   /**< 2D vector of long */
using Vector2f                      = Vector2<float>;  /**< 2D vector of float */
using Vector2d                      = Vector2<double>; /**< 2D vector of double */

template <typename T> using Vector3 = Matrix<T, 3, 1>; /**< generic 3D vector */
using Vector3u                      = Vector3<uint>;   /**< 3D vector of unsigned int */
using Vector3i                      = Vector3<int>;    /**< 3D vector of int */
using Vector3l                      = Vector3<long>;   /**< 3D vector of long */
using Vector3f                      = Vector3<float>;  /**< 3D vector of float */
using Vector3d                      = Vector3<double>; /**< 3D vector of double */

template <typename T> using Vector4 = Matrix<T, 4, 1>; /**< generic 4D vector */
using Vector4u                      = Vector4<uint>;   /**< 4D vector of unsigned int */
using Vector4i                      = Vector4<int>;    /**< 4D vector of int */
using Vector4l                      = Vector4<long>;   /**< 4D vector of long */
using Vector4f                      = Vector4<float>;  /**< 4D vector of float */
using Vector4d                      = Vector4<double>; /**< 4D vector of double */

template <typename T> using Vector6 = Matrix<T, 6, 1>; /**< generic 6D vector */
using Vector6u                      = Vector6<uint>;   /**< 6D vector of unsigned int */
using Vector6i                      = Vector6<int>;    /**< 6D vector of int */
using Vector6l                      = Vector6<long>;   /**< 6D vector of long */
using Vector6f                      = Vector6<float>;  /**< 6D vector of float */
using Vector6d                      = Vector6<double>; /**< 6D vector of double */

template <typename T> using Matrix2 = Matrix<T, 2, 2>; /**< generic 2x2 matrix */
using Matrix2u                      = Matrix2<uint>;   /**< 2x2 matrix of unsigned int */
using Matrix2i                      = Matrix2<int>;    /**< 2x2 matrix of int */
using Matrix2l                      = Matrix2<long>;   /**< 2x2 matrix of long */
using Matrix2f                      = Matrix2<float>;  /**< 2x2 matrix of float */
using Matrix2d                      = Matrix2<double>; /**< 2x2 matrix of double */

template <typename T> using Matrix3 = Matrix<T, 3, 3>; /**< generic 3x3 matrix */
using Matrix3u                      = Matrix3<uint>;   /**< 3x3 matrix of unsigned int */
using Matrix3i                      = Matrix3<int>;    /**< 3x3 matrix of int */
using Matrix3l                      = Matrix3<long>;   /**< 3x3 matrix of long */
using Matrix3f                      = Matrix3<float>;  /**< 3x3 matrix of float */
using Matrix3d                      = Matrix3<double>; /**< 3x3 matrix of double */

template <typename T> using Matrix4 = Matrix<T, 4, 4>; /**< generic 4x4 matrix */
using Matrix4u                      = Matrix4<uint>;   /**< 4x4 matrix of unsigned int */
using Matrix4i                      = Matrix4<int>;    /**< 4x4 matrix of int */
using Matrix4l                      = Matrix4<long>;   /**< 4x4 matrix of long */
using Matrix4f                      = Matrix4<float>;  /**< 4x4 matrix of float */
using Matrix4d                      = Matrix4<double>; /**< 4x4 matrix of double */

template <typename T> using Matrix6 = Matrix<T, 6, 6>; /**< generic 6x6 matrix */
using Matrix6u                      = Matrix6<uint>;   /**< 6x6 matrix of unsigned int */
using Matrix6i                      = Matrix6<int>;    /**< 6x6 matrix of int */
using Matrix6l                      = Matrix6<long>;   /**< 6x6 matrix of long */
using Matrix6f                      = Matrix6<float>;  /**< 6x6 matrix of float */
using Matrix6d                      = Matrix6<double>; /**< 6x6 matrix of double */

template <uint rows, uint cols>
using MatrixXu = Matrix<uint, rows, cols>; /**< generic mxn matrix of unsigned int */
template <uint rows, uint cols>
using MatrixXi = Matrix<int, rows, cols>; /**< generic mxn matrix of int */
template <uint rows, uint cols>
using MatrixXl = Matrix<long, rows, cols>; /**< generic mxn matrix of long */
template <uint rows, uint cols>
using MatrixXf = Matrix<float, rows, cols>; /**< generic mxn matrix of float */
template <uint rows, uint cols>
using MatrixXd = Matrix<double, rows, cols>; /**< generic mxn matrix of double */

template <typename T, uint dim>
using VectorX = Matrix<T, dim, 1>; /**< generic vertical vector */
template <uint dim>
using VectorXu = Matrix<uint, dim, 1>; /**< generic vertical vector of unsigned int */
template <uint dim>
using VectorXi = Matrix<int, dim, 1>; /**< generic vertical vector of int */
template <uint dim>
using VectorXl = Matrix<long, dim, 1>; /**< generic vertical vector of long */
template <uint dim>
using VectorXf = Matrix<float, dim, 1>; /**< generic vertical vector of float */
template <uint dim>
using VectorXd = Matrix<double, dim, 1>; /**< generic vertical vector of double */

template <typename T, uint dim>
using RowVectorX = Matrix<T, 1, dim>; /**< generic horizontal vector */
template <uint dim>
using RowVectorXu = Matrix<uint, 1, dim>; /**< generic horizontal vector of uint */
template <uint dim>
using RowVectorXi = Matrix<int, 1, dim>; /**< generic horizontal vector of int */
template <uint dim>
using RowVectorXl = Matrix<long, 1, dim>; /**< generic horizontal vector of long */
template <uint dim>
using RowVectorXf = Matrix<float, 1, dim>; /**< generic horizontal vector of float */
template <uint dim>
using RowVectorXd = Matrix<double, 1, dim>; /**< generic horizontal vector of double */

} //  end namespace grabnum

// This is a trick to define templated functions in a source file.
#include "../src/matrix.cpp"

#endif /* GRABCOMMON_LIBNUMERIC_MATRIX_H_ */
