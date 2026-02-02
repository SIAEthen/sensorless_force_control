#ifndef SFC_MANIPULATOR_H_
#define SFC_MANIPULATOR_H_

#include <array>
#include <cstddef>
#include <string>

#include "config.h"
#include "utilts/rotation.h"

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

  virtual HomogeneousMatrix forwardKinematics(
      const std::array<sfc::Real, kDof>& q) const;

  virtual Jacobian jacobian(const std::array<sfc::Real, kDof>& q) const;

protected:
  std::string name_;
  DHConvention convention_;
  std::array<DHParam, kDof> dh_params_{};
  State state_{};

private:
  HomogeneousMatrix dhTransform(const DHParam& p, sfc::Real q) const;
};

#endif  // SFC_MANIPULATOR_H_
