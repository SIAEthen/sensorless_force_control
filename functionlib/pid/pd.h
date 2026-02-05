#ifndef SFC_PID_PD_H_
#define SFC_PID_PD_H_

#include <cstddef>
#include <stdexcept>

#include "config.h"
#include "pid/p.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

// Basic PD controller operating on Vector<Size> errors.
template <std::size_t Size>
class PdController : public PController<Size> {
public:
  void setGains(const Vector<Size>& kp, const Vector<Size>& kd) {
    this->kp_ = kp;
    kd_ = kd;
  }

  void reset() override {
    last_error_ = Vector<Size>{};
    has_last_error_ = false;
  }

  Vector<Size> update(const Vector<Size>& error, Real dt) override {
    if (!error.isFinite() || !isFinite(dt)) {
      throw std::runtime_error("PdController::update: non-finite value");
    }
    if (dt <= zero()) {
      throw std::runtime_error("PdController::update: non-positive dt");
    }

    Vector<Size> derivative{};
    if (has_last_error_) {
      for (std::size_t i = 0; i < Size; ++i) {
        derivative(i) = (error(i) - last_error_(i)) / dt;
      }
    }

    Vector<Size> output{};
    for (std::size_t i = 0; i < Size; ++i) {
      output(i) = this->kp_(i) * error(i) + kd_(i) * derivative(i);
    }

    last_error_ = error;
    has_last_error_ = true;
    return output;
  }

private:
  Vector<Size> kd_{};
  Vector<Size> last_error_{};
  bool has_last_error_{false};
};

}  // namespace sfc

#endif  // SFC_PID_PD_H_
