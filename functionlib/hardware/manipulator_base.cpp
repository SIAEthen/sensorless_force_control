#include "hardware/manipulator_base.h"

#include <cmath>

template <std::size_t Dof>
HomogeneousMatrix ManipulatorBase<Dof>::forwardKinematics(
    const std::array<sfc::Real, kDof>& q) const {
  HomogeneousMatrix t = HomogeneousMatrix::identity();
  for (std::size_t i = 0; i < kDof; ++i) {
    t = t * dhTransform(dh_params_[i], q[i]);
  }
  return t;
}

template <std::size_t Dof>
typename ManipulatorBase<Dof>::Jacobian ManipulatorBase<Dof>::jacobian(
    const std::array<sfc::Real, kDof>& q) const {
  std::array<Vector3, kDof + 1> origins{};
  std::array<Vector3, kDof + 1> z_axes{};

  HomogeneousMatrix t = HomogeneousMatrix::identity();
  origins[0] = t.translation();
  z_axes[0] = t.zAxis();

  for (std::size_t i = 0; i < kDof; ++i) {
    t = t * dhTransform(dh_params_[i], q[i]);
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
HomogeneousMatrix ManipulatorBase<Dof>::dhTransform(const DHParam& p, sfc::Real q) const {
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

template class ManipulatorBase<3>;
template class ManipulatorBase<5>;
template class ManipulatorBase<6>;
template class ManipulatorBase<7>;
