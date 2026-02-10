#ifndef SFC_MANIPULATOR_FROM_DH_H_
#define SFC_MANIPULATOR_FROM_DH_H_

#include <array>
#include <cmath>
#include <string>

#include "manipulator_base.h"
#include "utilts/rotation.h"

namespace sfc {

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

template <std::size_t Dof>
class ManipulatorFromDH : public ManipulatorBase<Dof> {
 public:
  using Base = ManipulatorBase<Dof>;
  using Jacobian = typename Base::Jacobian;

  explicit ManipulatorFromDH(std::string name,
                             DHConvention convention = DHConvention::kStandard)
      : Base(std::move(name)), convention_(convention) {}

  DHConvention convention() const { return convention_; }
  void setConvention(DHConvention convention) { convention_ = convention; }

  void setDHParameters(const std::array<DHParam, Dof>& params) { dh_params_ = params; }
  const std::array<DHParam, Dof>& dhParameters() const { return dh_params_; }

  HomogeneousMatrix forwardKinematics() const override {
    HomogeneousMatrix t = HomogeneousMatrix::identity();
    for (std::size_t i = 0; i < Dof; ++i) {
      t = t * dhTransform(dh_params_[i], this->state_.q[i]);
    }
    return t * this->t_tool_linkend_;
  }

  Jacobian jacobian() const override {
    std::array<Vector3, Dof + 1> origins{};
    std::array<Vector3, Dof + 1> z_axes{};

    HomogeneousMatrix t = HomogeneousMatrix::identity();
    origins[0] = t.translation();
    z_axes[0] = t.zAxis();

    for (std::size_t i = 0; i < Dof; ++i) {
      t = t * dhTransform(dh_params_[i], this->state_.q[i]);
      origins[i + 1] = t.translation();
      z_axes[i + 1] = t.zAxis();
    }

    HomogeneousMatrix t_tip = t * this->t_tool_linkend_;
    const Vector3 o_tip = t_tip.translation();

    Jacobian j{};
    for (std::size_t i = 0; i < Dof; ++i) {
      const Vector3 p = o_tip - origins[i];
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

 private:
  HomogeneousMatrix dhTransform(const DHParam& p, sfc::Real q) const {
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

  DHConvention convention_;
  std::array<DHParam, Dof> dh_params_{};
};

}  // namespace sfc

#endif  // SFC_MANIPULATOR_FROM_DH_H_
