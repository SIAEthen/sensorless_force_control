#ifndef SFC_FORCE_CONTROL_FORCE_CONTROL_H_
#define SFC_FORCE_CONTROL_FORCE_CONTROL_H_

#include <cstddef>
#include <limits>
#include <stdexcept>

#include "config.h"
#include "utilts/linear_algebra.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

class ForceControl {
public:
  ForceControl() {
    for (std::size_t i = 0; i < 6; ++i) {
      integrator_min_(i) = -std::numeric_limits<Real>::max();
      integrator_max_(i) = std::numeric_limits<Real>::max();
    }
  }
  void setGains(const Vector6& kp, const Vector6& ki, const Vector6& kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void setIntegratorLimits(const Vector6& min_value, const Vector6& max_value) {
    integrator_min_ = min_value;
    integrator_max_ = max_value;
  }

  void reset() {
    integral_ = Vector6{};
    last_error_ = Vector6{};
    has_last_error_ = false;
  }

  Vector6 update(const Vector6& desired_wrench,
                 const Vector6& measured_wrench,
                 Real dt) {
    if (!desired_wrench.isFinite() || !measured_wrench.isFinite() || !isFinite(dt)) {
      throw std::runtime_error("ForceControl::update: non-finite value");
    }
    if (dt <= zero()) {
      throw std::runtime_error("ForceControl::update: non-positive dt");
    }

    Vector6 error{};
    for (std::size_t i = 0; i < 6; ++i) {
      error(i) = desired_wrench(i) - measured_wrench(i);
    }

    if (has_last_error_) {
      for (std::size_t i = 0; i < 6; ++i) {
        integral_(i) += error(i) * dt;
      }
      integral_ = clampVector(integral_, integrator_min_, integrator_max_);
    }

    Vector6 derivative{};
    if (has_last_error_) {
      for (std::size_t i = 0; i < 6; ++i) {
        derivative(i) = (error(i) - last_error_(i)) / dt;
      }
    }

    Vector6 output{};
    for (std::size_t i = 0; i < 6; ++i) {
      output(i) = kp_(i) * error(i) + ki_(i) * integral_(i) + kd_(i) * derivative(i);
    }

    last_error_ = error;
    has_last_error_ = true;
    return output;
  }

private:
  Vector6 kp_{};
  Vector6 ki_{};
  Vector6 kd_{};
  Vector6 integral_{};
  Vector6 last_error_{};
  Vector6 integrator_min_{};
  Vector6 integrator_max_{};
  bool has_last_error_{false};
};

}  // namespace sfc


#endif  // SFC_FORCE_CONTROL_FORCE_CONTROL_H_
