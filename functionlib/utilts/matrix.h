#ifndef SFC_UTILTS_MATRIX_H_
#define SFC_UTILTS_MATRIX_H_

#include <array>
#include <cmath>
#include <cstddef>
#include <ostream>
#include <stdexcept>

#include "config.h"
#include "utilts/vector.h"

namespace sfc {

template <std::size_t Rows, std::size_t Cols>
struct Matrix {
  std::array<sfc::Real, Rows * Cols> data{};

  sfc::Real operator()(std::size_t row, std::size_t col) const {
    return data[row * Cols + col];
  }

  sfc::Real& operator()(std::size_t row, std::size_t col) {
    return data[row * Cols + col];
  }

  bool isFinite() const {
    for (sfc::Real value : data) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
    return true;
  }

  Matrix operator+(const Matrix& other) const {
    if (!isFinite() || !other.isFinite()) {
      throw std::runtime_error("Matrix contains non-finite value");
    }
    Matrix result{};
    for (std::size_t i = 0; i < data.size(); ++i) {
      result.data[i] = data[i] + other.data[i];
    }
    return result;
  }

  Matrix operator-(const Matrix& other) const {
    if (!isFinite() || !other.isFinite()) {
      throw std::runtime_error("Matrix contains non-finite value");
    }
    Matrix result{};
    for (std::size_t i = 0; i < data.size(); ++i) {
      result.data[i] = data[i] - other.data[i];
    }
    return result;
  }

  template <std::size_t OtherCols>
  Matrix<Rows, OtherCols> operator*(const Matrix<Cols, OtherCols>& other) const {
    if (!isFinite() || !other.isFinite()) {
      throw std::runtime_error("Matrix contains non-finite value");
    }
    Matrix<Rows, OtherCols> result{};
    for (std::size_t row = 0; row < Rows; ++row) {
      for (std::size_t col = 0; col < OtherCols; ++col) {
        sfc::Real sum = 0.0;
        for (std::size_t k = 0; k < Cols; ++k) {
          sum += (*this)(row, k) * other(k, col);
        }
        result(row, col) = sum;
      }
    }
    return result;
  }

  Vector<Rows> operator*(const Vector<Cols>& v) const {
    if (!isFinite() || !v.isFinite()) {
      throw std::runtime_error("Matrix contains non-finite value");
    }
    Vector<Rows> out{};
    for (std::size_t row = 0; row < Rows; ++row) {
      sfc::Real sum = 0.0;
      for (std::size_t col = 0; col < Cols; ++col) {
        sum += (*this)(row, col) * v(col);
      }
      out(row) = sum;
    }
    return out;
  }

  Matrix<Cols, Rows> transpose() const {
    if (!isFinite()) {
      throw std::runtime_error("Matrix contains non-finite value");
    }
    Matrix<Cols, Rows> result{};
    for (std::size_t row = 0; row < Rows; ++row) {
      for (std::size_t col = 0; col < Cols; ++col) {
        result(col, row) = (*this)(row, col);
      }
    }
    return result;
  }
};

using Matrix3 = Matrix<3, 3>;
using Matrix4 = Matrix<4, 4>;

template <std::size_t RowsTop, std::size_t RowsBottom, std::size_t Cols>
inline Matrix<RowsTop + RowsBottom, Cols> vstack(
    const Matrix<RowsTop, Cols>& top,
    const Matrix<RowsBottom, Cols>& bottom) {
  Matrix<RowsTop + RowsBottom, Cols> out{};
  for (std::size_t row = 0; row < RowsTop; ++row) {
    for (std::size_t col = 0; col < Cols; ++col) {
      out(row, col) = top(row, col);
    }
  }
  for (std::size_t row = 0; row < RowsBottom; ++row) {
    for (std::size_t col = 0; col < Cols; ++col) {
      out(row + RowsTop, col) = bottom(row, col);
    }
  }
  return out;
}

template <std::size_t Rows, std::size_t ColsLeft, std::size_t ColsRight>
inline Matrix<Rows, ColsLeft + ColsRight> hstack(
    const Matrix<Rows, ColsLeft>& left,
    const Matrix<Rows, ColsRight>& right) {
  Matrix<Rows, ColsLeft + ColsRight> out{};
  for (std::size_t row = 0; row < Rows; ++row) {
    for (std::size_t col = 0; col < ColsLeft; ++col) {
      out(row, col) = left(row, col);
    }
    for (std::size_t col = 0; col < ColsRight; ++col) {
      out(row, col + ColsLeft) = right(row, col);
    }
  }
  return out;
}

template <std::size_t Rows, std::size_t Cols>
inline Matrix<Rows, Cols> zeros() {
  return Matrix<Rows, Cols>{};
}

template <std::size_t N>
inline Matrix<N, N> identity() {
  Matrix<N, N> m{};
  for (std::size_t i = 0; i < N; ++i) {
    m(i, i) = static_cast<sfc::Real>(1.0);
  }
  return m;
}

inline Matrix3 skew(const Vector3& p) {
  Matrix3 s{};
  s(0, 0) = 0.0;
  s(0, 1) = -p(2);
  s(0, 2) = p(1);
  s(1, 0) = p(2);
  s(1, 1) = 0.0;
  s(1, 2) = -p(0);
  s(2, 0) = -p(1);
  s(2, 1) = p(0);
  s(2, 2) = 0.0;
  return s;
}

inline Matrix<6, 6> blockDiag(const Matrix3& r1,const Matrix3& r2) {
  Matrix<6, 6> m{};
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      m(i, j) = r1(i, j);
      m(i + 3, j + 3) = r2(i, j);
    }
  }
  return m;
}

template <std::size_t Rows, std::size_t Cols>
inline void printMatrix(const Matrix<Rows, Cols>& m,
                        std::ostream& os,
                        const char* name) {
  os << name << " (" << Rows << "x" << Cols << ")\n";
  for (std::size_t row = 0; row < Rows; ++row) {
    for (std::size_t col = 0; col < Cols; ++col) {
      os << m(row, col);
      if (col + 1 < Cols) {
        os << " ";
      }
    }
    os << "\n";
  }
}

}  // namespace sfc

#endif  // SFC_UTILTS_MATRIX_H_
