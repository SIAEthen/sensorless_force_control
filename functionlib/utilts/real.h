#ifndef SFC_UTILTS_REAL_H_
#define SFC_UTILTS_REAL_H_

#include <cmath>
#include <limits>
#include <stdexcept>

#include "config.h"

namespace sfc {
inline Real zero() { return static_cast<Real>(0.0); }
inline Real one() { return static_cast<Real>(1.0); }

inline Real pi() {
  return static_cast<Real>(3.14159265358979323846264338327950288L);
}

inline Real deg2rad(Real degrees) { return degrees * (pi() / static_cast<Real>(180.0)); }
inline Real rad2deg(Real radians) { return radians * (static_cast<Real>(180.0) / pi()); }

inline bool isFinite(Real value) { return std::isfinite(value); }

inline bool epsilonEqual(Real a, Real b,
                         Real eps = static_cast<Real>(1e-9)) {
  return std::fabs(a - b) <= eps;
}

inline Real clampReal(Real value, Real low, Real high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

}  // namespace sfc

#endif  // SFC_UTILTS_REAL_H_
