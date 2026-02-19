#ifndef SFC_STSM_CONTROL_SUPER_TWISTING_SMC_H_
#define SFC_STSM_CONTROL_SUPER_TWISTING_SMC_H_

#include <cstddef>
#include <stdexcept>

#include "functionlib/config.h"
#include "functionlib/utilts/real.h"
#include "functionlib/utilts/vector.h"

namespace sfc {

// Super-Twisting Sliding Mode Controller (vector form, Size=6).
//
// Per-channel dynamics:
//   u_i = -k1_i * sqrt(|s_i|) * sat(s_i/eps_i) + z_i
//   dz_i = -k2_i * sat(s_i/eps_i) * dt
class SuperTwistingSmc {
 public:
  SuperTwistingSmc() = default;

  // k1, k2: super-twisting gains; lambda: sliding surface gain (s = d_error + lambda*error).
  void setGains(const Vector6& k1, const Vector6& k2, const Vector6& lambda);
  // epsilon, if it is zero, then we get sign(s), 
  // if it is larger than 0, we have a saturation function.
  // default it is zero.
  void setBoundaryLayer(const Vector6& epsilon);
  void reset();

  // Default update path: s = d_error + lambda * error.
  Vector6 update(const Vector6& error,
                 const Vector6& d_error,
                 const Vector6& feedforward,
                 Real dt);
  // Overload: estimate d_error internally from the previous error.
  Vector6 update(const Vector6& error,
                 const Vector6& feedforward,
                 Real dt);
  // Overload: zero feedforward.
  Vector6 update(const Vector6& error, Real dt);

 private:
  static Real sat(Real s, Real eps);

  Vector6 k1_{};
  Vector6 k2_{};
  Vector6 lambda_{};
  Vector6 epsilon_{};
  Vector6 z_{};
  Vector6 prev_error_{};
  bool has_prev_error_ = false;
};

}  // namespace sfc

#endif  // SFC_STSM_CONTROL_SUPER_TWISTING_SMC_H_
