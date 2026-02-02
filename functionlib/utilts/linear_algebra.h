#ifndef SFC_UTILTS_LINEAR_ALGEBRA_H_
#define SFC_UTILTS_LINEAR_ALGEBRA_H_

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>

#include "config.h"
#include "utilts/matrix.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

template <std::size_t N>
Matrix<N, N> addScaledIdentity(const Matrix<N, N>& a, Real lambda) {
  if (!a.isFinite() || !isFinite(lambda)) {
    throw std::runtime_error("addScaledIdentity: non-finite value");
  }
  Matrix<N, N> out = a;
  for (std::size_t i = 0; i < N; ++i) {
    out(i, i) += lambda;
  }
  return out;
}

template <std::size_t N>
Matrix<N, N> inverse(const Matrix<N, N>& a) {
  if (!a.isFinite()) {
    throw std::runtime_error("inverse: non-finite value");
  }

  Matrix<N, N> aug{};
  Matrix<N, N> inv = identity<N>();

  for (std::size_t row = 0; row < N; ++row) {
    for (std::size_t col = 0; col < N; ++col) {
      aug(row, col) = a(row, col);
    }
  }

  for (std::size_t i = 0; i < N; ++i) {
    std::size_t pivot = i;
    Real max_val = std::fabs(aug(i, i));
    for (std::size_t row = i + 1; row < N; ++row) {
      const Real val = std::fabs(aug(row, i));
      if (val > max_val) {
        max_val = val;
        pivot = row;
      }
    }

    if (max_val == static_cast<Real>(0.0)) {
      throw std::runtime_error("inverse: singular matrix");
    }

    if (pivot != i) {
      for (std::size_t col = 0; col < N; ++col) {
        std::swap(aug(i, col), aug(pivot, col));
        std::swap(inv(i, col), inv(pivot, col));
      }
    }

    const Real diag = aug(i, i);
    for (std::size_t col = 0; col < N; ++col) {
      aug(i, col) /= diag;
      inv(i, col) /= diag;
    }

    for (std::size_t row = 0; row < N; ++row) {
      if (row == i) {
        continue;
      }
      const Real factor = aug(row, i);
      for (std::size_t col = 0; col < N; ++col) {
        aug(row, col) -= factor * aug(i, col);
        inv(row, col) -= factor * inv(i, col);
      }
    }
  }

  return inv;
}

template <std::size_t Rows, std::size_t Cols>
Matrix<Cols, Rows> pseudoInverseDls(const Matrix<Rows, Cols>& j, Real lambda) {
  if (!j.isFinite() || !isFinite(lambda)) {
    throw std::runtime_error("pseudoInverseDls: non-finite value");
  }
  const Matrix<Cols, Rows> jt = j.transpose();
  Matrix<Rows, Rows> jj = j * jt;
  jj = addScaledIdentity(jj, lambda);
  const Matrix<Rows, Rows> inv = inverse(jj);
  return jt * inv;
}

template <std::size_t Size>
Vector<Size> elementWiseDivide(const Vector<Size>& a, const Vector<Size>& b) {
  if (!a.isFinite() || !b.isFinite()) {
    throw std::runtime_error("elementWiseDivide: non-finite value");
  }
  Vector<Size> out{};
  for (std::size_t i = 0; i < Size; ++i) {
    if (b(i) == static_cast<Real>(0.0)) {
      throw std::runtime_error("elementWiseDivide: division by zero");
    }
    out(i) = a(i) / b(i);
  }
  return out;
}

template <std::size_t Size>
Vector<Size> elementWiseMultiply(const Vector<Size>& a, const Vector<Size>& b) {
  if (!a.isFinite() || !b.isFinite()) {
    throw std::runtime_error("elementWiseMultiply: non-finite value");
  }
  Vector<Size> out{};
  for (std::size_t i = 0; i < Size; ++i) {
    out(i) = a(i) * b(i);
  }
  return out;
}

template <std::size_t Size>
Vector<Size> clampVector(const Vector<Size>& value,
                         const Vector<Size>& min_value,
                         const Vector<Size>& max_value) {
  if (!value.isFinite() || !min_value.isFinite() || !max_value.isFinite()) {
    throw std::runtime_error("clampVector: non-finite value");
  }
  Vector<Size> out{};
  for (std::size_t i = 0; i < Size; ++i) {
    if (value(i) < min_value(i)) {
      out(i) = min_value(i);
    } else if (value(i) > max_value(i)) {
      out(i) = max_value(i);
    } else {
      out(i) = value(i);
    }
  }
  return out;
}

template <std::size_t Rows, std::size_t Cols>
Real frobeniusNorm(const Matrix<Rows, Cols>& m) {
  if (!m.isFinite()) {
    throw std::runtime_error("frobeniusNorm: non-finite value");
  }
  Real sum = 0.0;
  for (std::size_t row = 0; row < Rows; ++row) {
    for (std::size_t col = 0; col < Cols; ++col) {
      const Real value = m(row, col);
      sum += value * value;
    }
  }
  return std::sqrt(sum);
}

}  // namespace sfc

#endif  // SFC_UTILTS_LINEAR_ALGEBRA_H_
