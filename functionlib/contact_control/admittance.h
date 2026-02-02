#ifndef SFC_CONTACT_CONTROL_ADMITTANCE_H_
#define SFC_CONTACT_CONTROL_ADMITTANCE_H_

#include <cstddef>
#include <stdexcept>

#include "config.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

class AdmittanceController {
public:
  void setGains(const Vector6& mass,
                const Vector6& damping,
                const Vector6& stiffness) {
    m_ = mass;
    b_ = damping;
    k_ = stiffness;
  }

  void reset() {
    desired_velocity_ = Vector6{};
    desired_position_offset_ = Vector6{};
  }

  Vector6 update(const Vector6& desired_force,
                 const Vector6& measured_force,
                 Real dt) {
    if (!desired_force.isFinite() || !measured_force.isFinite() || !isFinite(dt)) {
      throw std::runtime_error("AdmittanceController::update: non-finite value");
    }
    if (dt <= zero()) {
      throw std::runtime_error("AdmittanceController::update: non-positive dt");
    }

    Vector6 acceleration{};
    for (std::size_t i = 0; i < 6; ++i) {
      const Real force_error = desired_force(i) - measured_force(i);
      acceleration(i) = (force_error - b_(i) * desired_velocity_(i) -
                         k_(i) * desired_position_offset_(i)) / m_(i);
    }

    for (std::size_t i = 0; i < 6; ++i) {
      desired_velocity_(i) += acceleration(i) * dt;
      desired_position_offset_(i) += desired_velocity_(i) * dt;
    }

    return desired_position_offset_;
  }

private:
  Vector6 m_{};
  Vector6 b_{};
  Vector6 k_{};
  Vector6 desired_velocity_{};
  Vector6 desired_position_offset_{};
};

}  // namespace sfc

#endif  // SFC_CONTACT_CONTROL_ADMITTANCE_H_
