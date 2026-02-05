#ifndef SFC_VEHICLE_BASE_H_
#define SFC_VEHICLE_BASE_H_

#include <array>
#include <cstddef>
#include <string>
#include <stdexcept>

#include "config.h"
#include "utilts/matrix.h"
#include "utilts/real.h"
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

  Vector3 rpy() const {
    return Vector3{state_.position[3],
                   state_.position[4],
                   state_.position[5]};
  }

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

  Matrix3 J_ko() const {
    const Vector3 rpy_vec = rpy();
    if (!rpy_vec.isFinite()) {
      throw std::runtime_error("VehicleBase::J_ko: non-finite value");
    }
    const sfc::Real phi = rpy_vec(0);
    const sfc::Real theta = rpy_vec(1);
    const sfc::Real psi = rpy_vec(2);

    const sfc::Real cp = std::cos(psi);
    const sfc::Real sp = std::sin(psi);
    const sfc::Real ct = std::cos(theta);
    const sfc::Real st = std::sin(theta);
    const sfc::Real cf = std::cos(phi);
    const sfc::Real sf = std::sin(phi);

    Matrix3 j{};
    j(0, 0) = static_cast<sfc::Real>(1.0);
    j(0, 1) = static_cast<sfc::Real>(0.0);
    j(0, 2) = -st;
    j(1, 0) = static_cast<sfc::Real>(0.0);
    j(1, 1) = cf;
    j(1, 2) = ct * sf;
    j(2, 0) = static_cast<sfc::Real>(0.0);
    j(2, 1) = -sf;
    j(2, 2) = ct * cf;

    (void)cp;
    (void)sp;
    return j;
  }

  Matrix3 J_ko_inv() const {
    const Vector3 rpy_vec = rpy();
    if (!rpy_vec.isFinite()) {
      throw std::runtime_error("VehicleBase::J_ko_inv: non-finite value");
    }
    const sfc::Real phi = rpy_vec(0);
    const sfc::Real theta = rpy_vec(1);
    const sfc::Real psi = rpy_vec(2);

    const sfc::Real cp = std::cos(psi);
    const sfc::Real sp = std::sin(psi);
    const sfc::Real ct = std::cos(theta);
    const sfc::Real st = std::sin(theta);
    const sfc::Real cf = std::cos(phi);
    const sfc::Real sf = std::sin(phi);

    if (ct == zero()) {
      throw std::runtime_error("VehicleBase::J_ko_inv: cos(theta) is zero");
    }

    Matrix3 j{};
    const sfc::Real inv_ct = static_cast<sfc::Real>(1.0) / ct;
    j(0, 0) = ct * inv_ct;
    j(0, 1) = sf * st * inv_ct;
    j(0, 2) = cf * st * inv_ct;
    j(1, 0) = static_cast<sfc::Real>(0.0);
    j(1, 1) = cf * ct * inv_ct;
    j(1, 2) = -sf * ct * inv_ct;
    j(2, 0) = static_cast<sfc::Real>(0.0);
    j(2, 1) = sf * inv_ct;
    j(2, 2) = cf * inv_ct;

    (void)cp;
    (void)sp;
    return j;
  }

protected:
  std::string name_;
  std::size_t dof_;
  State state_;
};

}  // namespace sfc

#endif  // SFC_VEHICLE_BASE_H_
