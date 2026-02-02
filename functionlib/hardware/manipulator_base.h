#ifndef SFC_MANIPULATOR_H_
#define SFC_MANIPULATOR_H_

#include <array>
#include <cstddef>
#include <cmath>
#include <string>

#include "config.h"
#include "utilts/rotation.h"

namespace sfc {

template <std::size_t Dof>
class ManipulatorBase {
public:
  static constexpr std::size_t kDof = Dof;

  enum class DHConvention {
    kStandard,
    kModified
  };

  struct DHParam {
    sfc::Real a{0.0};
    sfc::Real alpha{0.0};
    sfc::Real d{0.0};
    sfc::Real theta{0.0};
  };

  struct State {
    std::array<sfc::Real, kDof> q{};
    std::array<sfc::Real, kDof> dq{};
    std::array<sfc::Real, kDof> ddq{};
  };

  using Jacobian = Matrix<6, kDof>;

  explicit ManipulatorBase(std::string name,
                           DHConvention convention = DHConvention::kStandard)
      : name_(std::move(name)), convention_(convention) {}

  virtual ~ManipulatorBase() = default;

  std::size_t dof() const { return kDof; }
  const std::string& name() const { return name_; }
  DHConvention convention() const { return convention_; }

  void setConvention(DHConvention convention) { convention_ = convention; }

  void setDHParameters(const std::array<DHParam, kDof>& params) { dh_params_ = params; }
  const std::array<DHParam, kDof>& dhParameters() const { return dh_params_; }

  void setState(const State& state) { state_ = state; }
  const State& state() const { return state_; }

  void setJointPosition(const std::array<sfc::Real, kDof>& q) { state_.q = q; }
  void setJointVelocity(const std::array<sfc::Real, kDof>& dq) { state_.dq = dq; }
  void setJointAcceleration(const std::array<sfc::Real, kDof>& ddq) { state_.ddq = ddq; }

  const std::array<sfc::Real, kDof>& jointPosition() const { return state_.q; }
  const std::array<sfc::Real, kDof>& jointVelocity() const { return state_.dq; }
  const std::array<sfc::Real, kDof>& jointAcceleration() const { return state_.ddq; }

  // Returns T_ee_world. For the manipulator, world frame is the base frame.
  virtual HomogeneousMatrix forwardKinematics() const;

  virtual Jacobian jacobian() const;

protected:
  std::string name_;
  DHConvention convention_;
  std::array<DHParam, kDof> dh_params_{};
  State state_{};

private:
  HomogeneousMatrix dhTransform(const DHParam& p, sfc::Real q) const;
};

template <std::size_t Dof>
HomogeneousMatrix ManipulatorBase<Dof>::forwardKinematics() const {
  HomogeneousMatrix t = HomogeneousMatrix::identity();
  for (std::size_t i = 0; i < kDof; ++i) {
    t = t * dhTransform(dh_params_[i], state_.q[i]);
  }
  return t;
}

template <std::size_t Dof>
typename ManipulatorBase<Dof>::Jacobian ManipulatorBase<Dof>::jacobian() const {
  std::array<Vector3, kDof + 1> origins{};
  std::array<Vector3, kDof + 1> z_axes{};

  HomogeneousMatrix t = HomogeneousMatrix::identity();
  origins[0] = t.translation();
  z_axes[0] = t.zAxis();

  for (std::size_t i = 0; i < kDof; ++i) {
    t = t * dhTransform(dh_params_[i], state_.q[i]);
    origins[i + 1] = t.translation();
    z_axes[i + 1] = t.zAxis();
  }

  const Vector3 o_n = origins[kDof];
  Jacobian j{};
  for (std::size_t i = 0; i < kDof; ++i) {
    const Vector3 p = o_n - origins[i];
    const Vector3 z = z_axes[i];
    const Vector3 jv = cross(z, p);
    j(0, i) = jv(0);
    j(1, i) = jv(1);
    j(2, i) = jv(2);
    j(3, i) = z(0);
    j(4, i) = z(1);
    j(5, i) = z(2);
  }
  return j;
}

template <std::size_t Dof>
HomogeneousMatrix ManipulatorBase<Dof>::dhTransform(const DHParam& p,
                                                    sfc::Real q) const {
  const sfc::Real theta = p.theta + q;
  const sfc::Real ct = std::cos(theta);
  const sfc::Real st = std::sin(theta);
  const sfc::Real ca = std::cos(p.alpha);
  const sfc::Real sa = std::sin(p.alpha);

  HomogeneousMatrix t{};
  if (convention_ == DHConvention::kModified) {
    t(0, 0) = ct;
    t(0, 1) = -st;
    t(0, 2) = 0.0;
    t(0, 3) = p.a;

    t(1, 0) = st * ca;
    t(1, 1) = ct * ca;
    t(1, 2) = -sa;
    t(1, 3) = -p.d * sa;

    t(2, 0) = st * sa;
    t(2, 1) = ct * sa;
    t(2, 2) = ca;
    t(2, 3) = p.d * ca;
  } else {
    t(0, 0) = ct;
    t(0, 1) = -st * ca;
    t(0, 2) = st * sa;
    t(0, 3) = p.a * ct;

    t(1, 0) = st;
    t(1, 1) = ct * ca;
    t(1, 2) = -ct * sa;
    t(1, 3) = p.a * st;

    t(2, 0) = 0.0;
    t(2, 1) = sa;
    t(2, 2) = ca;
    t(2, 3) = p.d;
  }

  t(3, 0) = 0.0;
  t(3, 1) = 0.0;
  t(3, 2) = 0.0;
  t(3, 3) = 1.0;
  return t;
}

}  // namespace sfc

#endif  // SFC_MANIPULATOR_H_
