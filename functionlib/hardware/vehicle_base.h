#ifndef SFC_VEHICLE_BASE_H_
#define SFC_VEHICLE_BASE_H_

#include <array>
#include <cstddef>
#include <string>

#include "config.h"
#include "utilts/rotation.h"
#include "utilts/types.h"

namespace sfc {

class VehicleBase {
public:
  static constexpr std::size_t kDof = 6;

  struct State {
    std::array<sfc::Real, kDof> position; // x y z roll pitch yaw
    std::array<sfc::Real, kDof> velocity; // u v w p q r
    std::array<sfc::Real, kDof> acceleration; // du dv dw dp dq dr

    State() : position{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
              velocity{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
              acceleration{0.0, 0.0, 0.0, 0.0, 0.0, 0.0} {}
  };

  explicit VehicleBase(std::string name, std::size_t dof = kDof)
      : name_(std::move(name)), dof_(dof <= kDof ? dof : kDof) {}

  virtual ~VehicleBase() = default;

  std::size_t dof() const { return dof_; }
  const std::string& name() const { return name_; }

  void setState(const State& state) { state_ = state; }
  const State& state() const { return state_; }
  const State& getState() const { return state_; }

  void setPosition(const std::array<sfc::Real, kDof>& position) {
    state_.position = position;
  }

  void setVelocity(const std::array<sfc::Real, kDof>& velocity) {
    state_.velocity = velocity;
  }

  void setAcceleration(const std::array<sfc::Real, kDof>& acceleration) {
    state_.acceleration = acceleration;
  }

  const std::array<sfc::Real, kDof>& position() const { return state_.position; }
  const std::array<sfc::Real, kDof>& velocity() const { return state_.velocity; }
  const std::array<sfc::Real, kDof>& acceleration() const { return state_.acceleration; }

  RotationMatrix getRotationMatrixBaseToInertial() const {
    return RotationMatrix::fromRPY(state_.position[3],
                                   state_.position[4],
                                   state_.position[5]);
  }

  RotationMatrix getRotationMatrixInertialToBase() const {
    return getRotationMatrixBaseToInertial().transpose();
  }

  // Returns T_b_ned. For the vehicle, world frame is NED.
  HomogeneousMatrix forwardKinematics() const {
    const RotationMatrix r = getRotationMatrixBaseToInertial();
    const Vector3 p{state_.position[0],
                    state_.position[1],
                    state_.position[2]};
    return HomogeneousMatrix::fromRotationTranslation(r, p);
  }

protected:
  std::string name_;
  std::size_t dof_;
  State state_;
};

}  // namespace sfc

#endif  // SFC_VEHICLE_BASE_H_
