#ifndef SFC_CONTACT_CONTROL_VARIABLE_H_
#define SFC_CONTACT_CONTROL_VARIABLE_H_

#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "config.h"
#include "utilts/error_representation.h"
#include "utilts/linear_algebra.h"
#include "utilts/real.h"
#include "utilts/rotation.h"
#include "utilts/vector.h"

namespace sfc {

// Quaternion-based admittance controller with variable stiffness.
//
// Admittance dynamics (element-wise diagonal gains as vectors):
//   K_M * acc + K_D * delta_nu + K_P * e = w_hat
//
// where e = [p_d - p_r; eps_tilde] and eps_tilde is the vector part of
// the quaternion error Q_tilde = Q_d * Q_r^{-1}.
//
// Variable stiffness law:
//   dK_P/dt = K_S * (K(nu_E) - K_P)
//   K(nu_E) = K_min + (K_max - K_min) * |nu_E| / (|nu_E| + nu_0)   (element-wise)
class VariableAdmittanceController {
 public:
  struct Output {
    Vector3 pos_r{};
    Quaternion q_r{};
    Vector6 nu_r{};
    Vector6 acc_r{};
  };

  VariableAdmittanceController() {
    for (std::size_t i = 0; i < 6; ++i) {
      m_(i) = static_cast<Real>(1.0);
    }
  }

  void setGains(const Vector6& mass,
                const Vector6& damping,
                const Vector6& stiffness) {
    m_ = mass;
    d_ = damping;
    k_ = stiffness;
    validateMass();
  }
  const Vector6& getStiffness() const { return k_; }

  // Parameters for variable stiffness. nu_0 must be > 0 (velocity half-saturation point).
  void setVariableStiffnessParams(const Vector6& k_s,
                                  const Vector6& k_min,
                                  const Vector6& k_max,
                                  const Vector6& nu_0) {
    if (!nu_0.isFinite()) {
      throw std::runtime_error("VariableAdmittanceController: nu_0 non-finite");
    }
    for (std::size_t i = 0; i < 6; ++i) {
      if (nu_0(i) <= zero()) {
        throw std::runtime_error(
            "VariableAdmittanceController: nu_0 must be > 0");
      }
    }
    k_s_ = k_s;
    k_min_ = k_min;
    k_max_ = k_max;
    nu_0_ = nu_0;
  }

  void reset(const Vector3& pos_r, const Quaternion& q_r) {
    if (!pos_r.isFinite() || !std::isfinite(q_r.w) || !std::isfinite(q_r.x) ||
        !std::isfinite(q_r.y) || !std::isfinite(q_r.z)) {
      throw std::runtime_error("VariableAdmittanceController::reset: non-finite value");
    }
    delta_nu_ = Vector6{};
    nu_r_ = Vector6{};
    pos_r_ = pos_r;
    q_r_ = q_r.normalized();
  }

  Output update(const Vector3& p_d,
                const Quaternion& q_d,
                const Vector6& nu_d,
                const Vector6& w_hat,
                Real dt) {
    if (!p_d.isFinite() || !nu_d.isFinite() || !w_hat.isFinite()) {
      throw std::runtime_error("VariableAdmittanceController::update: non-finite vector");
    }
    if (!isFinite(dt) || dt <= zero()) {
      throw std::runtime_error("VariableAdmittanceController::update: non-positive dt");
    }
    if (!std::isfinite(q_d.w) || !std::isfinite(q_d.x) ||
        !std::isfinite(q_d.y) || !std::isfinite(q_d.z)) {
      throw std::runtime_error("VariableAdmittanceController::update: non-finite quaternion");
    }
    if (!m_.isFinite() || !d_.isFinite() || !k_.isFinite()) {
      throw std::runtime_error("VariableAdmittanceController::update: non-finite gains");
    }

    // Pose error: e = [p_d - p_r;  eps_tilde]
    Vector6 e{};
    e(0) = p_d(0) - pos_r_(0);
    e(1) = p_d(1) - pos_r_(1);
    e(2) = p_d(2) - pos_r_(2);
    const Quaternion e_q = quatError(q_d, q_r_);
    e(3) = e_q.x;
    e(4) = e_q.y;
    e(5) = e_q.z;

    // Variable stiffness update: K_P += K_S * (K(nu_E) - K_P) * dt
    updateStiffness(dt);

    // Admittance: acc = (w_hat - K_D * delta_nu - K_P * e) / K_M
    Vector6 acc_r{};
    for (std::size_t i = 0; i < 6; ++i) {
      acc_r(i) = (w_hat(i) - d_(i) * delta_nu_(i) - k_(i) * e(i)) / m_(i);
    }

    // Euler integration of velocity error and reference velocity
    delta_nu_ = delta_nu_ + acc_r * dt;
    nu_r_ = nu_d - delta_nu_;

    // Integrate reference position
    pos_r_(0) += nu_r_(0) * dt;
    pos_r_(1) += nu_r_(1) * dt;
    pos_r_(2) += nu_r_(2) * dt;

    // Integrate reference orientation via exponential map
    Vector3 omega_r{};
    omega_r(0) = nu_r_(3);
    omega_r(1) = nu_r_(4);
    omega_r(2) = nu_r_(5);
    const Real omega_norm = vectorNorm(omega_r);
    Quaternion dq{};
    if (omega_norm > static_cast<Real>(1e-6)) {
      const Real half_theta = static_cast<Real>(0.5) * omega_norm * dt;
      const Real vec_scale = std::sin(half_theta) / omega_norm;
      dq.w = std::cos(half_theta);
      dq.x = omega_r(0) * vec_scale;
      dq.y = omega_r(1) * vec_scale;
      dq.z = omega_r(2) * vec_scale;
    } else {
      const Real vec_scale = static_cast<Real>(0.5) * dt;
      dq.w = static_cast<Real>(1.0);
      dq.x = omega_r(0) * vec_scale;
      dq.y = omega_r(1) * vec_scale;
      dq.z = omega_r(2) * vec_scale;
    }
    q_r_ = (q_r_ * dq).normalized();

    return Output{pos_r_, q_r_, nu_r_, acc_r};
  }

 private:
  void validateMass() const {
    for (std::size_t i = 0; i < 6; ++i) {
      if (!isFinite(m_(i)) || m_(i) <= zero()) {
        throw std::runtime_error(
            "VariableAdmittanceController: mass must be finite and > 0");
      }
    }
  }

  // K(nu_E) = K_min + (K_max - K_min) * |nu_E| / (|nu_E| + nu_0)
  // dK_P/dt = K_S * (K(nu_E) - K_P)   →  Euler: K_P += K_S*(K(nu_E)-K_P)*dt
  void updateStiffness(Real dt) {
    for (std::size_t i = 0; i < 6; ++i) {
      if (k_s_(i) == zero()) {
        continue;
      }
      const Real nu_i = std::abs(delta_nu_(i));
      const Real k_target = k_min_(i) + (k_max_(i) - k_min_(i)) * nu_i / (nu_i + nu_0_(i));
      k_(i) += k_s_(i) * (k_target - k_(i)) * dt;
    }
  }

  Vector6 m_{};
  Vector6 d_{};
  Vector6 k_{};
  Vector6 k_s_{};
  Vector6 k_min_{};
  Vector6 k_max_{};
  Vector6 nu_0_{};
  Vector6 delta_nu_{};
  Vector6 nu_r_{};
  Vector3 pos_r_{};
  Quaternion q_r_{Quaternion::identity()};
};

}  // namespace sfc

#endif  // SFC_CONTACT_CONTROL_VARIABLE_H_
