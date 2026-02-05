#ifndef SFC_PID_P_H_
#define SFC_PID_P_H_

#include <cstddef>
#include <stdexcept>

#include "config.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

// Basic P controller operating on Vector<Size> errors.
template <std::size_t Size>
class PController {
public:
  virtual ~PController() = default;

  void setGains(const Vector<Size>& kp) {
    kp_ = kp;
  }

  virtual void reset() {}

  virtual Vector<Size> update(const Vector<Size>& error, Real dt) {
    if (!error.isFinite() || !isFinite(dt)) {
      throw std::runtime_error("PController::update: non-finite value");
    }
    if (dt <= zero()) {
      throw std::runtime_error("PController::update: non-positive dt");
    }

    Vector<Size> output{};
    for (std::size_t i = 0; i < Size; ++i) {
      output(i) = kp_(i) * error(i);
    }

    return output;
  }

protected:
  Vector<Size> kp_{};
};

}  // namespace sfc

#endif  // SFC_PID_P_H_
