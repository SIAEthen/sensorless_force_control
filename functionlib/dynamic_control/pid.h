#ifndef SFC_DYNAMIC_CONTROL_PID_H_
#define SFC_DYNAMIC_CONTROL_PID_H_

#include <cstddef>
#include <limits>
#include <stdexcept>

#include "config.h"
#include "utilts/linear_algebra.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

template <std::size_t Size>
class Pid {
public:
  Pid() {
    for (std::size_t i = 0; i < Size; ++i) {
      integrator_min_(i) = -std::numeric_limits<Real>::max();
      integrator_max_(i) = std::numeric_limits<Real>::max();
    }
  }
  void setGains(const Vector<Size>& kp,
                const Vector<Size>& ki,
                const Vector<Size>& kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void setIntegratorLimits(const Vector<Size>& min_value,
                           const Vector<Size>& max_value) {
    integrator_min_ = min_value;
    integrator_max_ = max_value;
  }

  void reset() {
    integral_ = Vector<Size>{};
    last_error_ = Vector<Size>{};
    has_last_error_ = false;
  }

  Vector<Size> update(const Vector<Size>& error, Real dt) {
    if (!error.isFinite() || !isFinite(dt)) {
      throw std::runtime_error("Pid::update: non-finite value");
    }
    if (dt <= zero()) {
      throw std::runtime_error("Pid::update: non-positive dt");
    }

    if (has_last_error_) {
      for (std::size_t i = 0; i < Size; ++i) {
        integral_(i) += error(i) * dt;
      }
      integral_ = clampVector(integral_, integrator_min_, integrator_max_);
    }

    Vector<Size> derivative{};
    if (has_last_error_) {
      for (std::size_t i = 0; i < Size; ++i) {
        derivative(i) = (error(i) - last_error_(i)) / dt;
      }
    }

    Vector<Size> output{};
    for (std::size_t i = 0; i < Size; ++i) {
      output(i) = kp_(i) * error(i) + ki_(i) * integral_(i) + kd_(i) * derivative(i);
    }

    last_error_ = error;
    has_last_error_ = true;
    return output;
  }

private:
  Vector<Size> kp_{};
  Vector<Size> ki_{};
  Vector<Size> kd_{};
  Vector<Size> integral_{};
  Vector<Size> last_error_{};
  Vector<Size> integrator_min_{};
  Vector<Size> integrator_max_{};
  bool has_last_error_{false};
};

}  // namespace sfc

#endif  // SFC_DYNAMIC_CONTROL_PID_H_
