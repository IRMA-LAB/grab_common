#include <QtTest/QtTest>

#include <cmath>

#include "common.h"
#include "matrix.h"
#include "matrix_utilities.h"

#define DIM1 2
#define DIM2 3

class TestMatrix: public QObject
{
  Q_OBJECT

 private:
  const double DValues_[6]        = {0.1, 1.1, 2.1, 3.1, 4.1, 5.1};
  const int IValues_[6]           = {0, 1, 2, 3, 4, 5};

  grabnum::Matrix2d mat22d_;
  grabnum::Matrix2i mat22i_;
  grabnum::MatrixXd<DIM1, DIM2> mat23d_;
  grabnum::MatrixXi<DIM1, DIM2> mat23i_;
  grabnum::MatrixXi<DIM2, DIM1> mat32i_;
  grabnum::Matrix2d identity2d_;
  grabnum::Matrix2i identity2i_;
  grabnum::Matrix2d zeros22d_;
  grabnum::Matrix2i zeros22i_;

 private slots:
  void init();

  void Constructors();

  void ClassOperators();

  void Setters();

  void Getters();

  void Manipulations();

  void Operations();

  void Utilities();
};

void TestMatrix::init()
{
  // Reset data.
  mat22d_.Fill(DValues_, mat22d_.Size());
  mat22i_.Fill(IValues_, mat22i_.Size());
  mat23d_.Fill(DValues_, mat23d_.Size());
  mat23i_.Fill(IValues_, mat23i_.Size());
  mat32i_.Fill(IValues_, mat32i_.Size());
  identity2d_.SetIdentity();
  identity2i_.SetIdentity();
  zeros22d_.SetZero();
  zeros22i_.SetZero();
}

void TestMatrix::Constructors()
{
  const uint8_t m = 3;
  const uint8_t n = 2;

  // Default constructor.
  grabnum::MatrixXi<m, n> mat_mxn_i;
  for (uint8_t i = 1; i <= m; ++i)
    for (uint8_t j = 1; j <= n; ++j)
      QCOMPARE(mat_mxn_i(i, j), 0);

  // Identity constructor.
  double diag_value = 4.3;
  grabnum::Matrix3d mat_mxm_d(diag_value);
  for (uint8_t i = 1; i <= m; ++i)
    for (uint8_t j = 1; j <= n; ++j)
      if (i == j)
        QCOMPARE(mat_mxm_d(i, j), diag_value);
      else
        QCOMPARE(mat_mxm_d(i, j), 0.);

  // Constructor from array.
  float vec[n * m] = {3.45f, 3.1f, 6.7f, 54.3f, 5.74f, 0.45f};
  grabnum::MatrixXf<n, m> mat_nxm_f(vec, ARRAY_SIZE(vec));
  for (uint8_t i = 1; i <= n; ++i)
    for (uint8_t j = 1; j <= m; ++j)
      QCOMPARE(mat_nxm_f(i, j), vec[(i - 1) * m + (j - 1)]);

  // Constructor from standard array (hard way).
  std::vector<float> std_vec(vec, vec + sizeof(vec) / sizeof(float));
  grabnum::MatrixXf<n, m> mat_nxm_f2(&std_vec.front(),
                                     static_cast<uint16_t>(std_vec.size()));
  for (uint8_t i = 1; i <= n; ++i)
    for (uint8_t j = 1; j <= m; ++j)
      QCOMPARE(mat_nxm_f2(i, j), std_vec[(i - 1) * m + (j - 1)]);

  // Constructor from standard array (easy way).
  grabnum::MatrixXf<n, m> mat_nxm_f4(std_vec);
  for (uint8_t i = 1; i <= n; ++i)
    for (uint8_t j = 1; j <= m; ++j)
      QCOMPARE(mat_nxm_f4(i, j), std_vec[(i - 1) * m + (j - 1)]);

  // Constructor from copy.
  grabnum::MatrixXf<n, m> mat_nxm_f3 = mat_nxm_f2;
  QVERIFY(mat_nxm_f2 == mat_nxm_f3);

  // Assignment with explicit casting.
  grabnum::MatrixXd<n, m> mat_nxm_d;
  mat_nxm_d = static_cast<grabnum::Matrix<double, n, m>>(mat_nxm_f2);
  QVERIFY(mat_nxm_d == mat_nxm_f2);

  // Constructor from copy with implicit casting.
  grabnum::MatrixXi<n, m> mat_nxm_i(mat_nxm_f2);
  QVERIFY(mat_nxm_i != mat_nxm_f2);
  QVERIFY(mat_nxm_i.Type() != mat_nxm_f2.Type());

  // Constructor from direct assignment with implicit casting.
  grabnum::MatrixXl<n, m> mat_nxm_i2 = mat_nxm_f2;
  QVERIFY(mat_nxm_i2 != mat_nxm_f2);
  QVERIFY(mat_nxm_i2.Type() != mat_nxm_f2.Type());

  // Double vector constructor.
  grabnum::VectorXd<ARRAY_SIZE(DValues_)> vvect(DValues_, ARRAY_SIZE(DValues_));
  grabnum::MatrixXd<1, ARRAY_SIZE(DValues_)> hvect(vvect.Transpose());
  for (uint8_t i = 1; i <= vvect.Size(); i++)
  {
    QVERIFY(vvect(i) == DValues_[i - 1]);
    QVERIFY(hvect(i) == DValues_[i - 1]);
  }
}

void TestMatrix::ClassOperators()
{
  double scalar = 0.1;
  double values[ARRAY_SIZE(DValues_)];
  for (uint16_t i = 0; i < ARRAY_SIZE(DValues_); ++i)
    values[i] = DValues_[i] + scalar;
  grabnum::MatrixXd<DIM1, DIM2> other(values, mat23d_.Size());
  grabnum::MatrixXd<DIM1, DIM2> mat23d_copy(mat23d_);
  grabnum::MatrixXd<DIM1, DIM2> other_copy(other);

  // Test equal operator
  bool res = true;
  for (uint8_t i = 0; i < DIM1; ++i)
  {
    if (!res)
      break;
    for (uint8_t j = 0; j < DIM2; ++j)
      if (!grabnum::IsClose(DValues_[i * DIM2 + j], mat23d_copy(i + 1, j + 1)))
      {
        res = false;
        break;
      }
  }
  QVERIFY(res == (mat23d_.IsApprox(mat23d_copy)));

  // Test not equal operator
  res = true;
  for (uint8_t i = 1; i <= DIM1; ++i)
  {
    if (!res)
      break;
    for (uint8_t j = 1; j <= DIM2; ++j)
      if (std::abs(mat23d_(i, j) - other(i, j)) <= std::numeric_limits<double>::epsilon())
      {
        res = false;
        break;
      }
  }
  QVERIFY(res == (!mat23d_.IsApprox(other)));

  // Test difference operator with scalar
  other -= scalar;
  QVERIFY(other.IsApprox(mat23d_));

  // Test addition operator with scalar
  other += scalar;
  QVERIFY(other.IsApprox(other_copy));

  // Test scalar multiplication
  for (uint16_t i = 0; i < ARRAY_SIZE(DValues_); ++i)
    values[i] = DValues_[i] * scalar;
  grabnum::MatrixXd<DIM1, DIM2> mat23d_scaled(values, mat23d_.Size());
  mat23d_copy *= scalar;
  QVERIFY(mat23d_copy.IsApprox(mat23d_scaled));

  // Test division by scalar
  mat23d_copy /= scalar;
  QVERIFY(mat23d_copy.IsApprox(mat23d_));

  // Test matrix addition
  double dvalues[6] = {-2.3, 0.0, 4.5, 6.77, -11.2, 0.1};
  grabnum::MatrixXd<DIM1, DIM2> addendum(dvalues, mat23d_.Size());
  addendum += mat23d_;
  double sum;
  for (uint8_t i = 0; i < DIM1; ++i)
    for (uint8_t j = 0; j < DIM2; ++j)
    {
      sum = DValues_[i * DIM2 + j] + dvalues[i * DIM2 + j];
      QVERIFY(std::abs(sum - addendum(i + 1, j + 1)) <= grabnum::EPSILON);
    }

  // Test matrix subtraction
  grabnum::MatrixXd<DIM1, DIM2> mat(dvalues, mat23d_.Size());
  mat -= mat23d_;
  double diff;
  for (uint8_t i = 0; i < DIM1; ++i)
    for (uint8_t j = 0; j < DIM2; ++j)
    {
      diff = dvalues[i * DIM2 + j] - DValues_[i * DIM2 + j];
      QVERIFY(std::abs(diff - mat(i + 1, j + 1)) <= grabnum::EPSILON);
    }
}

void TestMatrix::Setters()
{
  const uint8_t dim1 = 5;
  const uint8_t dim2 = 4;
  grabnum::MatrixXi<dim1, dim2> mat;

  // Test SetZero
  mat.SetZero();
  for (uint8_t i = 1; i <= dim1; ++i)
    for (uint8_t j = 1; j <= dim2; ++j)
      QVERIFY(mat(i, j) == 0);

  // Test SetIdentity
  mat.SetIdentity();
  for (uint8_t i = 1; i <= dim1; ++i)
    for (uint8_t j = 1; j <= dim2; ++j)
      if (i == j)
        QVERIFY(mat(i, j) == 1);
      else
        QVERIFY(mat(i, j) == 0);

  // Test SetBlock
  mat.SetZero();
  const uint8_t start_row = 2;
  const uint8_t start_col = 3;
  mat.SetBlock(start_row, start_col, mat22i_);
  for (uint8_t i = 1; i <= dim1; ++i)
    for (uint8_t j = 1; j <= dim2; ++j)
      if ((i >= start_row && i < mat22i_.Rows() + start_row) &&
          (j >= start_col && j < mat22i_.Cols() + start_col))
        QVERIFY(mat(i, j) == mat22i_(i - start_row + 1, j - start_col + 1));
      else
        QVERIFY(mat(i, j) == 0);

  // Test SetCol with 1D matrix
  mat.SetZero();
  grabnum::VectorXi<dim1> col(IValues_, dim1);
  mat.SetCol(start_col, col);
  for (uint8_t i = 1; i <= dim1; ++i)
    for (uint8_t j = 1; j <= dim2; ++j)
      if (j == start_col)
        QVERIFY(mat(i, j) == IValues_[i - 1]);
      else
        QVERIFY(mat(i, j) == 0);

  // Test SetCol with vector
  mat.SetZero();
  int values[dim1] = {1, 2, 3, 4, 5};
  mat.SetCol(start_col, values, ARRAY_SIZE(values));
  for (uint8_t i = 1; i <= dim1; ++i)
    for (uint8_t j = 1; j <= dim2; ++j)
      if (j == start_col)
        QVERIFY(mat(i, j) == values[i - 1]);
      else
        QVERIFY(mat(i, j) == 0);

  // Test SetRow with 1D matrix
  mat.SetZero();
  grabnum::MatrixXi<1, dim2> row(IValues_, dim2);
  mat.SetRow(start_row, row);
  for (uint8_t i = 1; i <= dim1; ++i)
    for (uint8_t j = 1; j <= dim2; ++j)
      if (i == start_row)
        QVERIFY(mat(i, j) == IValues_[j - 1]);
      else
        QVERIFY(mat(i, j) == 0);

  // Test SetRow with vector
  mat.SetZero();
  int values2[dim2] = {1, 2, 3, 4};
  mat.SetRow(start_row, values2, ARRAY_SIZE(values2));
  for (uint8_t i = 1; i <= dim1; ++i)
    for (uint8_t j = 1; j <= dim2; ++j)
      if (i == start_row)
        QVERIFY(mat(i, j) == values[j - 1]);
      else
        QVERIFY(mat(i, j) == 0);

  // Test Fill
  mat22i_.Fill(values2, ARRAY_SIZE(values2));
  QVERIFY(mat22i_(1, 1) == 1);
  QVERIFY(mat22i_(1, 2) == 2);
  QVERIFY(mat22i_(2, 1) == 3);
  QVERIFY(mat22i_(2, 2) == 4);

  // Test SetFrom Block
  mat22i_.SetFromBlock(1, 2, mat23i_);
  QVERIFY(mat22i_(1, 1) == mat23i_(1, 2));
  QVERIFY(mat22i_(1, 2) == mat23i_(1, 3));
  QVERIFY(mat22i_(2, 1) == mat23i_(2, 2));
  QVERIFY(mat22i_(2, 2) == mat23i_(2, 3));
}

void TestMatrix::Getters()
{
  // Test GetRow
  grabnum::MatrixXi<1, DIM2> row = mat23i_.GetRow(2);
  QVERIFY(row(1, 1) == 3);
  QVERIFY(row(1, 2) == 4);
  QVERIFY(row(1, 3) == 5);

  // Test GetRows
  grabnum::MatrixXi<2, DIM1> rows = mat32i_.GetRows<2>(2);
  QVERIFY(rows(1, 1) == 2);
  QVERIFY(rows(1, 2) == 3);
  QVERIFY(rows(2, 1) == 4);
  QVERIFY(rows(2, 2) == 5);

  // Test HeadRows
  grabnum::MatrixXi<2, DIM1> head_rows = mat32i_.HeadRows<2>();
  QVERIFY(head_rows(1, 1) == 0);
  QVERIFY(head_rows(1, 2) == 1);
  QVERIFY(head_rows(2, 1) == 2);
  QVERIFY(head_rows(2, 2) == 3);

  // Test GetRows
  grabnum::MatrixXi<2, DIM1> tail_rows = mat32i_.TailRows<2>();
  QVERIFY(rows == tail_rows);

  // Test GetCol
  grabnum::VectorXi<DIM1> col = mat23i_.GetCol(2);
  QVERIFY(col(1, 1) == 1);
  QVERIFY(col(2, 1) == 4);

  // Test GetCols
  grabnum::MatrixXi<DIM1, 2> cols = mat23i_.GetCols<2>(2);
  QVERIFY(cols(1, 1) == 1);
  QVERIFY(cols(1, 2) == 2);
  QVERIFY(cols(2, 1) == 4);
  QVERIFY(cols(2, 2) == 5);

  // Test HeadCols
  grabnum::MatrixXi<DIM1, 2> head_cols = mat23i_.HeadCols<2>();
  QVERIFY(head_cols(1, 1) == 0);
  QVERIFY(head_cols(1, 2) == 1);
  QVERIFY(head_cols(2, 1) == 3);
  QVERIFY(head_cols(2, 2) == 4);

  // Test GetCols
  grabnum::MatrixXi<DIM1, 2> tail_cols = mat23i_.TailCols<2>();
  QVERIFY(cols == tail_cols);

  // Test GetBlock
  grabnum::MatrixXi<2, 2> block = mat23i_.GetBlock<2, 2>(1, 2);
  QVERIFY(block(1, 1) == 1);
  QVERIFY(block(1, 2) == 2);
  QVERIFY(block(2, 1) == 4);
  QVERIFY(block(2, 2) == 5);
}

void TestMatrix::Manipulations()
{
  // Test Transpose
  grabnum::MatrixXi<DIM2, DIM1> mat = mat23i_.Transpose();
  for (uint8_t i = 1; i <= DIM1; ++i)
    for (uint8_t j = 1; j <= DIM2; ++j)
      QVERIFY(mat23i_(i, j) == mat(j, i));

  // Test SwapRow
  grabnum::MatrixXi<DIM1, DIM2> mat23i_copy(mat23i_);
  mat23i_copy.SwapRow(1, 2);
  for (uint8_t j = 1; j <= DIM2; ++j)
  {
    QVERIFY(mat23i_copy(1, j) == mat23i_(2, j));
    QVERIFY(mat23i_copy(2, j) == mat23i_(1, j));
  }

  // Test SwapRow
  mat23i_copy = mat23i_;
  mat23i_copy.SwapCol(1, 2);
  for (uint8_t i = 1; i <= DIM1; ++i)
  {
    QVERIFY(mat23i_copy(i, 1) == mat23i_(i, 2));
    QVERIFY(mat23i_copy(i, 2) == mat23i_(i, 1));
  }
}

void TestMatrix::Operations()
{
  // Test row-column multiplication.
  int result[6] = {3, 4, 5, 9, 14, 19};
  grabnum::MatrixXi<DIM1, DIM2> res(result, ARRAY_SIZE(result));
  QVERIFY((mat22i_ * mat23i_) == res);
  QVERIFY((identity2d_ * mat23d_) == mat23d_);
  QVERIFY((zeros22d_ * mat22d_) == zeros22d_);

  // Test outer product
  grabnum::VectorXi<DIM1> col(IValues_, DIM1);
  grabnum::MatrixXi<1, 3> row(IValues_, 3);
  int result2[6] = {0, 0, 0, 0, 1, 2};
  res.Fill(result2, ARRAY_SIZE(result2));
  QVERIFY((col * row) == res);

  // Test element-wise multiplication
  int result3[3] = {0, 1, 4};
  grabnum::MatrixXi<1, 3> prod(result3, ARRAY_SIZE(result3));
  QVERIFY((row.Transpose() * row.Transpose()) == prod.Transpose());
  QVERIFY((row * row) == prod);

  // Test horizontal concatenation
  int results4[10] = {0, 1, 2, 0, 1, 3, 4, 5, 2, 3};
  grabnum::MatrixXi<DIM1, 5> hcat(results4, ARRAY_SIZE(results4));
  QVERIFY(grabnum::HorzCat(mat23i_, mat22i_) == hcat);

  // Test vertical concatenation
  int results5[10] = {0, 3, 1, 4, 2, 5, 0, 1, 2, 3};
  grabnum::MatrixXi<5, 2> vcat(results5, ARRAY_SIZE(results5));
  QVERIFY(grabnum::VertCat(mat23i_.Transpose(), mat22i_) == vcat);

  // Test dot-product
  QVERIFY(grabnum::Dot(row.Transpose(), row.Transpose()) == 5);
  QVERIFY(grabnum::Dot(row, row.Transpose()) == 5);

  // Try cross-product
  QVERIFY(grabnum::Cross(row.Transpose(), row.Transpose()) == grabnum::Vector3i(0));
}

void TestMatrix::Utilities()
{
  // Test norm
  grabnum::MatrixXi<1, 3> row(IValues_, 3);
  QVERIFY(std::abs(grabnum::Norm(row) - 2.236067977499790) <= grabnum::EPSILON);
  QVERIFY(std::abs(grabnum::Norm(row.Transpose()) - 2.236067977499790) <=
          grabnum::EPSILON);

  // Test determinant
  int m_values[9] = {25, 15, -5, 15, 18, 0, -5, 0, 11};
  grabnum::Matrix3i mat(m_values, ARRAY_SIZE(m_values));
  QVERIFY(grabnum::Det(mat) == 2025);

  // Test Cholesky decomposition
  double l_values[9] = {5, 0, 0, 3, 3, 0, -1, 1, 3};
  grabnum::Matrix3d l_mat(l_values, ARRAY_SIZE(l_values));
  QVERIFY(grabnum::Cholesky(mat).IsApprox(l_mat));

  // Test positive-definiteness
  QVERIFY(mat.IsPositiveDefinite());
  int m_values2[9] = {14, 84, 40, 18, 81, 53, 40, 7, 42};
  mat.Fill(m_values2, ARRAY_SIZE(m_values2));
  QVERIFY(!mat.IsPositiveDefinite());
}

QTEST_MAIN(TestMatrix)
#include "matrix_test.moc"
