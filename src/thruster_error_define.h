#ifndef SFC_THRUSTER_ERROR_DEFINE_H_
#define SFC_THRUSTER_ERROR_DEFINE_H_

#include "functionlib/utilts/vector.h"
#include <stdexcept>

namespace sfc {

// Multiplicative drift for each thruster in:
// f_measured = (1 + drift_i) * f_commanded + offset_i
inline const Vector6& thrusterForceDrift() {
  static const Vector6 value{0.0459, 0.0156, -0.0464, 0.0349, 0.0434, 0.0179};
  return value;
}

// Additive bias [N] for each thruster.
inline const Vector6& thrusterForceOffset() {
  static const Vector6 value{2.5774, 2.4313, -1.0777, 1.5548, -3.2881, 2.0605};
  return value;
}

inline Vector6 applyThrusterForceError(const Vector6& commanded_force) {
  Vector6 out{};
  for (std::size_t i = 0; i < 6; ++i) {
    out(i) = (static_cast<Real>(1.0) + thrusterForceDrift()(i)) * commanded_force(i) +
             thrusterForceOffset()(i);
  }
  return out;
}

inline Vector6 removeThrusterForceError(const Vector6& measured_force) {
  Vector6 out{};
  for (std::size_t i = 0; i < 6; ++i) {
    const Real scale = static_cast<Real>(1.0) + thrusterForceDrift()(i);
    if (std::abs(scale) < static_cast<Real>(1e-9)) {
      throw std::runtime_error("removeThrusterForceError: invalid drift scale");
    }
    out(i) = (measured_force(i) - thrusterForceOffset()(i)) / scale;
  }
  return out;
}

}  // namespace sfc

#endif  // SFC_THRUSTER_ERROR_DEFINE_H_
