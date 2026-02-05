#ifndef SFC_PID_PID_H_
#define SFC_PID_PID_H_

#include <cstddef>
#include <limits>
#include <stdexcept>

#include "config.h"
#include "pid/p.h"
#include "utilts/linear_algebra.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

// Basic PID controller operating on Vector<Size> errors.
template <std::size_t Size>
class PidController : public PController<Size> {
public:
  PidController() {
    for (std::size_t i = 0; i < Size; ++i) {
      integrator_sat_(i) = std::numeric_limits<Real>::max();
    }
  }

  void setGains(const Vector<Size>& kp,
                const Vector<Size>& ki,
                const Vector<Size>& kd) {
    this->kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void setIntegratorLimits(const Vector<Size>& max_value) {
    if (!max_value.isFinite()) {
      throw std::runtime_error("PidController::setIntegratorLimits: non-finite value");
    }
    for (std::size_t i = 0; i < Size; ++i) {
      if (max_value(i) < zero()) {
        throw std::runtime_error("PidController::setIntegratorLimits: negative limit");
      }
      integrator_sat_(i) = max_value(i);
    }
  }

  void reset() override {
    integral_ = Vector<Size>{};
    last_error_ = Vector<Size>{};
    has_last_error_ = false;
  }

  Vector<Size> update(const Vector<Size>& error, Real dt) override {
    if (!error.isFinite() || !isFinite(dt)) {
      throw std::runtime_error("PidController::update: non-finite value");
    }
    if (dt <= zero()) {
      throw std::runtime_error("PidController::update: non-positive dt");
    }

    if (has_last_error_) {
      for (std::size_t i = 0; i < Size; ++i) {
        integral_(i) += error(i) * dt;
      }
      for (std::size_t i = 0; i < Size; ++i) {
        integral_(i) = clampReal(integral_(i), -integrator_sat_(i), integrator_sat_(i));
      }
    }

    Vector<Size> derivative{};
    if (has_last_error_) {
      for (std::size_t i = 0; i < Size; ++i) {
        derivative(i) = (error(i) - last_error_(i)) / dt;
      }
    }

    Vector<Size> output{};
    for (std::size_t i = 0; i < Size; ++i) {
      output(i) = this->kp_(i) * error(i) + ki_(i) * integral_(i) + kd_(i) * derivative(i);
    }

    last_error_ = error;
    has_last_error_ = true;
    return output;
  }

private:
  Vector<Size> ki_{};
  Vector<Size> kd_{};
  Vector<Size> integral_{};
  Vector<Size> last_error_{};
  Vector<Size> integrator_sat_{};
  bool has_last_error_{false};
};

}  // namespace sfc

#endif  // SFC_PID_PID_H_
