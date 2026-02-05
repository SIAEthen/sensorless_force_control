// Quaternion error helpers.
#ifndef SFC_UTILTS_ERROR_REPRESENTATION_H_
#define SFC_UTILTS_ERROR_REPRESENTATION_H_

#include <stdexcept>

#include "config.h"
#include "utilts/real.h"
#include "utilts/rotation.h"

namespace sfc {

// Quaternion error: q_e = q_d * conj(q_now)
// Ensures a consistent hemisphere (w >= 0) and returns a unit quaternion.
inline Quaternion quatError(const Quaternion& q_d, const Quaternion& q_now) {
  if (!std::isfinite(q_d.w) || !std::isfinite(q_d.x) || !std::isfinite(q_d.y) ||
      !std::isfinite(q_d.z) || !std::isfinite(q_now.w) || !std::isfinite(q_now.x) ||
      !std::isfinite(q_now.y) || !std::isfinite(q_now.z)) {
    throw std::runtime_error("quatError: non-finite value");
  }

  Quaternion q_e = q_d * q_now.conjugate();
  if (q_e.w < zero()) {
    q_e.w = -q_e.w;
    q_e.x = -q_e.x;
    q_e.y = -q_e.y;
    q_e.z = -q_e.z;
  }
  return q_e.normalized();
}

}  // namespace sfc

#endif  // SFC_UTILTS_ERROR_REPRESENTATION_H_
