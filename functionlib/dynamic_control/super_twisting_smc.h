#ifndef SFC_DYNAMIC_CONTROL_SUPER_TWISTING_SMC_H_
#define SFC_DYNAMIC_CONTROL_SUPER_TWISTING_SMC_H_

#include <cstddef>
#include <cmath>
#include <stdexcept>

#include "config.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

template <std::size_t Size>
class SuperTwistingSmc {
public:
  void setGains(const Vector<Size>& k1, const Vector<Size>& k2) {
    k1_ = k1;
    k2_ = k2;
  }

  void reset() { integral_ = Vector<Size>{}; }

  Vector<Size> update(const Vector<Size>& s, Real dt) {
    if (!s.isFinite() || !isFinite(dt)) {
      throw std::runtime_error("SuperTwistingSmc::update: non-finite value");
    }
    if (dt <= zero()) {
      throw std::runtime_error("SuperTwistingSmc::update: non-positive dt");
    }

    Vector<Size> u{};
    for (std::size_t i = 0; i < Size; ++i) {
      const Real sign_s = (s(i) >= zero()) ? one() : -one();
      const Real abs_s = std::fabs(s(i));
      integral_(i) += sign_s * dt;
      u(i) = -k1_(i) * std::sqrt(abs_s) * sign_s - k2_(i) * integral_(i);
    }
    return u;
  }

private:
  Vector<Size> k1_{};
  Vector<Size> k2_{};
  Vector<Size> integral_{};
};

}  // namespace sfc

#endif  // SFC_DYNAMIC_CONTROL_SUPER_TWISTING_SMC_H_
