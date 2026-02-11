#ifndef SFC_MANIPULATOR_H_
#define SFC_MANIPULATOR_H_

#include <array>
#include <cstddef>
#include <string>

#include "config.h"
#include "utilts/rotation.h"

namespace sfc {

template <std::size_t Dof>
class ManipulatorBase {
public:
  static constexpr std::size_t kDof = Dof;

  struct State {
    std::array<sfc::Real, kDof> q{};
    std::array<sfc::Real, kDof> dq{};
    std::array<sfc::Real, kDof> ddq{};
  };

  using Jacobian = Matrix<6, kDof>;

  explicit ManipulatorBase(std::string name)
      : name_(std::move(name)) {}

  virtual ~ManipulatorBase() = default;

  std::size_t dof() const { return kDof; }
  const std::string& name() const { return name_; }
  void setToolTransformationFromT(const HomogeneousMatrix& t_tool_linkend) {
    t_tool_linkend_ = t_tool_linkend;
  }
  const HomogeneousMatrix& toolTransformation() const { return t_tool_linkend_; }

  void setState(const State& state) { state_ = state; }
  const State& state() const { return state_; }

  void setJointPosition(const std::array<sfc::Real, kDof>& q) { state_.q = q; }
  void setJointVelocity(const std::array<sfc::Real, kDof>& dq) { state_.dq = dq; }
  void setJointAcceleration(const std::array<sfc::Real, kDof>& ddq) { state_.ddq = ddq; }

  const std::array<sfc::Real, kDof>& jointPosition() const { return state_.q; }
  const std::array<sfc::Real, kDof>& jointVelocity() const { return state_.dq; }
  const std::array<sfc::Real, kDof>& jointAcceleration() const { return state_.ddq; }

  // Returns T_ee_world. For the manipulator, world frame is the base frame.
  virtual HomogeneousMatrix forwardKinematics() const = 0;
  virtual Jacobian jacobian() const = 0;
  virtual std::array<HomogeneousMatrix, kDof + 1> jointTransforms() const = 0;

protected:
  std::string name_;
  HomogeneousMatrix t_tool_linkend_{HomogeneousMatrix::identity()};
  State state_{};
};

}  // namespace sfc

#endif  // SFC_MANIPULATOR_H_
