#ifndef SFC_OBSERVER_CONTACT_WRENCH_OBSERVER_H_
#define SFC_OBSERVER_CONTACT_WRENCH_OBSERVER_H_

#include <cstddef>
#include <stdexcept>

#include "config.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

// Contact wrench observer with decoupled force and torque channels.
//
// Force observer (element-wise):
//   d/dt(tau_f_hat) = -K1 * tau_f_hat + m * K1 * a + K1 * F1
//
// Torque observer:
//   tau_t_hat = K2 * [ P(T) - P(0) + integral_0^T(F2 - tau_t_hat)dt ]
//   P = I .* omega
class ContactWrenchObserver {
 public:
  ContactWrenchObserver() {
    for (std::size_t i = 0; i < 3; ++i) {
      inertia_(i) = static_cast<Real>(1.0);
    }
  }

  void setMass(Real mass) {
    if (!isFinite(mass) || mass <= zero()) {
      throw std::runtime_error("ContactWrenchObserver::setMass: mass must be finite and > 0");
    }
    mass_ = mass;
  }

  void setInertia(const Vector3& inertia) {
    if (!inertia.isFinite()) {
      throw std::runtime_error("ContactWrenchObserver::setInertia: non-finite inertia");
    }
    for (std::size_t i = 0; i < 3; ++i) {
      if (inertia(i) <= zero()) {
        throw std::runtime_error("ContactWrenchObserver::setInertia: inertia must be > 0");
      }
    }
    inertia_ = inertia;
  }

  void setGains(const Vector3& k1, const Vector3& k2) {
    if (!k1.isFinite() || !k2.isFinite()) {
      throw std::runtime_error("ContactWrenchObserver::setGains: non-finite gains");
    }
    for (std::size_t i = 0; i < 3; ++i) {
      if (k1(i) < zero() || k2(i) < zero()) {
        throw std::runtime_error("ContactWrenchObserver::setGains: gains must be >= 0");
      }
    }
    k1_ = k1;
    k2_ = k2;
  }

  void reset() {
    tau_f_hat_ = Vector3{};
    tau_t_hat_ = Vector3{};
    torque_integral_ = Vector3{};
    p0_ = Vector3{};
    has_p0_ = false;
  }

  void reset(const Vector3& omega0) {
    if (!omega0.isFinite()) {
      throw std::runtime_error("ContactWrenchObserver::reset: non-finite omega0");
    }
    reset();
    p0_ = impulseFromOmega(omega0);
    has_p0_ = true;
  }

  // f      phi*theta - tau_v
  // a      acceleration of the vehicle
  // omega  angular velocity of the vehicle
  // dt     time
  Vector6 update(const Vector6& f,
                 const Vector3& a,
                 const Vector3& omega,
                 Real dt) {
    if (!f.isFinite() || !a.isFinite() || !omega.isFinite() || !isFinite(dt)) {
      throw std::runtime_error("ContactWrenchObserver::update: non-finite input");
    }
    if (dt <= zero()) {
      throw std::runtime_error("ContactWrenchObserver::update: non-positive dt");
    }

    if (!has_p0_) {
      p0_ = impulseFromOmega(omega);
      has_p0_ = true;
    }

    Vector3 f1{};
    Vector3 f2{};
    for (std::size_t i = 0; i < 3; ++i) {
      f1(i) = f(i);
      f2(i) = f(i + 3);
    }

    // Force estimate integration.
    for (std::size_t i = 0; i < 3; ++i) {
      const Real tau_f_dot =
          -k1_(i) * tau_f_hat_(i) + mass_ * k1_(i) * a(i) + k1_(i) * f1(i);
      tau_f_hat_(i) += tau_f_dot * dt;
    }

    // Torque estimate integration with previous tau_t_hat_ in the integral term.
    for (std::size_t i = 0; i < 3; ++i) {
      torque_integral_(i) += (f2(i) - tau_t_hat_(i)) * dt;
    }

    const Vector3 p = impulseFromOmega(omega);
    for (std::size_t i = 0; i < 3; ++i) {
      tau_t_hat_(i) = k2_(i) * ((p(i) - p0_(i)) + torque_integral_(i));
    }

    Vector6 tau_hat{};
    for (std::size_t i = 0; i < 3; ++i) {
      tau_hat(i) = tau_f_hat_(i);
      tau_hat(i + 3) = tau_t_hat_(i);
    }
    return tau_hat;
  }

  Vector3 forceEstimate() const { return tau_f_hat_; }
  Vector3 torqueEstimate() const { return tau_t_hat_; }

 private:
  Vector3 impulseFromOmega(const Vector3& omega) const {
    Vector3 p{};
    for (std::size_t i = 0; i < 3; ++i) {
      p(i) = inertia_(i) * omega(i);
    }
    return p;
  }

  Real mass_{static_cast<Real>(1.0)};
  Vector3 inertia_{};
  Vector3 k1_{};
  Vector3 k2_{};

  Vector3 tau_f_hat_{};
  Vector3 tau_t_hat_{};
  Vector3 torque_integral_{};
  Vector3 p0_{};
  bool has_p0_{false};
};

}  // namespace sfc

#endif  // SFC_OBSERVER_CONTACT_WRENCH_OBSERVER_H_
