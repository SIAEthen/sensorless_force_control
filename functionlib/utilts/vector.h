#ifndef SFC_UTILTS_VECTOR_H_
#define SFC_UTILTS_VECTOR_H_

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "config.h"

template <std::size_t Size>
struct Vector {
  std::array<sfc::Real, Size> data{};

  sfc::Real operator()(std::size_t index) const { return data[index]; }
  sfc::Real& operator()(std::size_t index) { return data[index]; }

  bool isFinite() const {
    for (sfc::Real value : data) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
    return true;
  }

  Vector operator+(const Vector& other) const {
    if (!isFinite() || !other.isFinite()) {
      throw std::runtime_error("Vector contains non-finite value");
    }
    Vector result{};
    for (std::size_t i = 0; i < data.size(); ++i) {
      result.data[i] = data[i] + other.data[i];
    }
    return result;
  }

  Vector operator-(const Vector& other) const {
    if (!isFinite() || !other.isFinite()) {
      throw std::runtime_error("Vector contains non-finite value");
    }
    Vector result{};
    for (std::size_t i = 0; i < data.size(); ++i) {
      result.data[i] = data[i] - other.data[i];
    }
    return result;
  }

  Vector operator*(sfc::Real scalar) const {
    if (!isFinite() || !std::isfinite(scalar)) {
      throw std::runtime_error("Vector contains non-finite value");
    }
    Vector result{};
    for (std::size_t i = 0; i < data.size(); ++i) {
      result.data[i] = data[i] * scalar;
    }
    return result;
  }
};

using Vector3 = Vector<3>;
using Vector6 = Vector<6>;

inline Vector3 cross(const Vector3& a, const Vector3& b) {
  Vector3 r{};
  r(0) = a(1) * b(2) - a(2) * b(1);
  r(1) = a(2) * b(0) - a(0) * b(2);
  r(2) = a(0) * b(1) - a(1) * b(0);
  return r;
}

#endif  // SFC_UTILTS_VECTOR_H_
