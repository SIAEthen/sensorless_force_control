#ifndef SFC_UTILTS_MATRIX_H_
#define SFC_UTILTS_MATRIX_H_

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "config.h"
#include "utilts/vector.h"

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

inline Matrix3 identity3() {
  Matrix3 i{};
  i(0, 0) = 1.0;
  i(1, 1) = 1.0;
  i(2, 2) = 1.0;
  return i;
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

#endif  // SFC_UTILTS_MATRIX_H_
