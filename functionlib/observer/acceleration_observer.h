#ifndef SFC_OBSERVER_ACCELERATION_OBSERVER_H_
#define SFC_OBSERVER_ACCELERATION_OBSERVER_H_

#include <cstddef>
#include <stdexcept>

#include "config.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

template <std::size_t Size>
class AccelerationObserver {
 public:
  explicit AccelerationObserver(Real cutoff_hz = static_cast<Real>(1.0))
      : cutoff_hz_(cutoff_hz) {
    validateCutoffHz(cutoff_hz_);
  }

  void setCutoffHz(Real cutoff_hz) {
    validateCutoffHz(cutoff_hz);
    cutoff_hz_ = cutoff_hz;
  }

  void reset() {
    has_last_velocity_ = false;
    last_velocity_ = Vector<Size>{};
    acceleration_ = Vector<Size>{};
  }

  Vector<Size> update(const Vector<Size>& velocity, Real dt) {
    if (!velocity.isFinite() || !isFinite(dt)) {
      throw std::runtime_error("AccelerationObserver::update: non-finite value");
    }
    if (dt <= zero()) {
      throw std::runtime_error("AccelerationObserver::update: non-positive dt");
    }

    if (!has_last_velocity_) {
      last_velocity_ = velocity;
      has_last_velocity_ = true;
      return acceleration_;
    }

    Vector<Size> raw_acceleration{};
    const Real tau = one() / (static_cast<Real>(2.0) * kPi * cutoff_hz_);
    const Real alpha = dt / (tau + dt);
    for (std::size_t i = 0; i < Size; ++i) {
      raw_acceleration(i) = (velocity(i) - last_velocity_(i)) / dt;
      acceleration_(i) = alpha * raw_acceleration(i) + (one() - alpha) * acceleration_(i);
    }

    last_velocity_ = velocity;
    return acceleration_;
  }

 private:
  static void validateCutoffHz(Real cutoff_hz) {
    if (!isFinite(cutoff_hz) || cutoff_hz <= zero()) {
      throw std::runtime_error("AccelerationObserver: cutoff_hz must be finite and > 0");
    }
  }

  Real cutoff_hz_{static_cast<Real>(1.0)};
  bool has_last_velocity_{false};
  Vector<Size> last_velocity_{};
  Vector<Size> acceleration_{};
};

using AccelerationObserver3 = AccelerationObserver<3>;
using AccelerationObserver6 = AccelerationObserver<6>;

}  // namespace sfc

#endif  // SFC_OBSERVER_ACCELERATION_OBSERVER_H_
