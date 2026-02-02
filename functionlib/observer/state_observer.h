#ifndef SFC_OBSERVER_STATE_OBSERVER_H_
#define SFC_OBSERVER_STATE_OBSERVER_H_

#include <cstddef>
#include <stdexcept>

#include "config.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

struct StateEstimate {
  Vector6 position{};
  Vector6 velocity{};
  Vector6 acceleration{};
};

class StateObserver {
public:
  explicit StateObserver(Real velocity_alpha = static_cast<Real>(0.2),
                         Real acceleration_alpha = static_cast<Real>(0.2))
      : velocity_alpha_(velocity_alpha),
        acceleration_alpha_(acceleration_alpha) {}

  void reset() {
    has_last_position_ = false;
    estimate_ = StateEstimate{};
  }

  StateEstimate update(const Vector6& position, Real dt) {
    if (!position.isFinite() || !isFinite(dt)) {
      throw std::runtime_error("StateObserver::update: non-finite value");
    }
    if (dt <= zero()) {
      throw std::runtime_error("StateObserver::update: non-positive dt");
    }

    if (!has_last_position_) {
      estimate_.position = position;
      last_position_ = position;
      has_last_position_ = true;
      return estimate_;
    }

    Vector6 raw_velocity{};
    for (std::size_t i = 0; i < 6; ++i) {
      raw_velocity(i) = (position(i) - last_position_(i)) / dt;
    }

    for (std::size_t i = 0; i < 6; ++i) {
      estimate_.velocity(i) = velocity_alpha_ * raw_velocity(i) +
                              (one() - velocity_alpha_) * estimate_.velocity(i);
    }

    Vector6 raw_acceleration{};
    for (std::size_t i = 0; i < 6; ++i) {
      raw_acceleration(i) = (estimate_.velocity(i) - last_velocity_(i)) / dt;
    }

    for (std::size_t i = 0; i < 6; ++i) {
      estimate_.acceleration(i) = acceleration_alpha_ * raw_acceleration(i) +
                                  (one() - acceleration_alpha_) * estimate_.acceleration(i);
    }

    estimate_.position = position;
    last_position_ = position;
    last_velocity_ = estimate_.velocity;
    return estimate_;
  }

private:
  Real velocity_alpha_{};
  Real acceleration_alpha_{};
  bool has_last_position_{false};
  Vector6 last_position_{};
  Vector6 last_velocity_{};
  StateEstimate estimate_{};
};

}  // namespace sfc

#endif  // SFC_OBSERVER_STATE_OBSERVER_H_
