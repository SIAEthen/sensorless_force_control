#ifndef SFC_FORCE_CONTROL_QUATERNION_ADMITTANCE_CONTROLLER_H_
#define SFC_FORCE_CONTROL_QUATERNION_ADMITTANCE_CONTROLLER_H_

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

class QuaternionAdmittanceController {
 public:
  struct Output {
    Vector3 pos_r{};
    Quaternion q_r{};
    Vector6 nu_r{};
    Vector6 acc_r{};
  };

  QuaternionAdmittanceController() {
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

  void reset(const Vector3& pos_r, const Quaternion& q_r) {
    if (!pos_r.isFinite() || !std::isfinite(q_r.w) || !std::isfinite(q_r.x) ||
        !std::isfinite(q_r.y) || !std::isfinite(q_r.z)) {
      throw std::runtime_error("QuaternionAdmittanceController::reset: non-finite value");
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
      throw std::runtime_error("QuaternionAdmittanceController::update: non-finite vector");
    }
    if (!isFinite(dt) || dt <= zero()) {
      throw std::runtime_error("QuaternionAdmittanceController::update: non-positive dt");
    }
    if (!std::isfinite(q_d.w) || !std::isfinite(q_d.x) || !std::isfinite(q_d.y) ||
        !std::isfinite(q_d.z)) {
      throw std::runtime_error("QuaternionAdmittanceController::update: non-finite quaternion");
    }
    if (!m_.isFinite() || !d_.isFinite() || !k_.isFinite()) {
      throw std::runtime_error("QuaternionAdmittanceController::update: non-finite gains");
    }

    Vector3 e_p{};
    e_p(0) = p_d(0) - pos_r_(0);
    e_p(1) = p_d(1) - pos_r_(1);
    e_p(2) = p_d(2) - pos_r_(2);

    const Quaternion e_q = quatError(q_d, q_r_);
    Vector6 e{};
    e(0) = e_p(0);
    e(1) = e_p(1);
    e(2) = e_p(2);
    e(3) = e_q.x;
    e(4) = e_q.y;
    e(5) = e_q.z;

    Vector6 acc_r{};
    for (std::size_t i = 0; i < 6; ++i) {
      const Real rhs = w_hat(i) - d_(i) * delta_nu_(i) - k_(i) * e(i);
      acc_r(i) = rhs / m_(i);
    }

    delta_nu_ = delta_nu_ + acc_r * dt;
    nu_r_ = nu_d - delta_nu_;

    pos_r_(0) += nu_r_(0) * dt;
    pos_r_(1) += nu_r_(1) * dt;
    pos_r_(2) += nu_r_(2) * dt;

    Vector3 omega_r{};
    omega_r(0) = nu_r_(3);
    omega_r(1) = nu_r_(4);
    omega_r(2) = nu_r_(5);
    const Real omega_norm = vectorNorm(omega_r);
    Quaternion dq{};
    if (omega_norm > static_cast<Real>(1e-6)) {
      const Real half_theta = static_cast<Real>(0.5) * omega_norm * dt;
      const Real sin_half = std::sin(half_theta);
      const Real cos_half = std::cos(half_theta);
      const Real vec_scale = sin_half / omega_norm;
      dq.w = cos_half;
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
            "QuaternionAdmittanceController: mass must be finite and > 0");
      }
    }
  }

  Vector6 m_{};
  Vector6 d_{};
  Vector6 k_{};
  Vector6 delta_nu_{};
  Vector6 nu_r_{};
  Vector3 pos_r_{};
  Quaternion q_r_{Quaternion::identity()};
};

}  // namespace sfc

#endif  // SFC_FORCE_CONTROL_QUATERNION_ADMITTANCE_CONTROLLER_H_
