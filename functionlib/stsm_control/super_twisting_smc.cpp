#include "functionlib/stsm_control/super_twisting_smc.h"

#include <cmath>

namespace sfc {

void SuperTwistingSmc::setGains(const Vector6& k1,
                                const Vector6& k2,
                                const Vector6& lambda) {
  if (!k1.isFinite() || !k2.isFinite() || !lambda.isFinite()) {
    throw std::runtime_error("SuperTwistingSmc::setGains: non-finite value");
  }
  for (std::size_t i = 0; i < 6; ++i) {
    if (k1(i) < zero() || k2(i) < zero() || lambda(i) < zero()) {
      throw std::runtime_error("SuperTwistingSmc::setGains: gains must be >= 0");
    }
  }
  k1_ = k1;
  k2_ = k2;
  lambda_ = lambda;
}

void SuperTwistingSmc::setBoundaryLayer(const Vector6& epsilon) {
  if (!epsilon.isFinite()) {
    throw std::runtime_error("SuperTwistingSmc::setBoundaryLayer: non-finite value");
  }
  for (std::size_t i = 0; i < 6; ++i) {
    if (epsilon(i) < zero()) {
      throw std::runtime_error("SuperTwistingSmc::setBoundaryLayer: epsilon must be >= 0");
    }
  }
  epsilon_ = epsilon;
}

void SuperTwistingSmc::reset() {
  z_ = Vector6{};
  prev_error_ = Vector6{};
  has_prev_error_ = false;
}

Vector6 SuperTwistingSmc::update(const Vector6& error,
                                 const Vector6& d_error,
                                 const Vector6& feedforward,
                                 Real dt) {
  if (!error.isFinite() || !d_error.isFinite() || !feedforward.isFinite() || !isFinite(dt)) {
    throw std::runtime_error("SuperTwistingSmc::update: non-finite value");
  }
  if (dt <= zero()) {
    throw std::runtime_error("SuperTwistingSmc::update: non-positive dt");
  }

  Vector6 sliding{};
  for (std::size_t i = 0; i < 6; ++i) {
    sliding(i) = d_error(i) + lambda_(i) * error(i);
  }
  
  Vector6 u{};
  for (std::size_t i = 0; i < 6; ++i) {
    const Real s = sliding(i);
    const Real sigma = sat(s, epsilon_(i));
    const Real sqrt_abs_s = std::sqrt(std::fabs(s));

    u(i) = k1_(i) * sqrt_abs_s * sigma + z_(i);
    z_(i) += k2_(i) * sigma * dt;
  }
  prev_error_ = error;
  has_prev_error_ = true;
  return u + feedforward;

}

Vector6 SuperTwistingSmc::update(const Vector6& error,
                                 const Vector6& feedforward,
                                 Real dt) {
  if (!error.isFinite() || !feedforward.isFinite() || !isFinite(dt)) {
    throw std::runtime_error("SuperTwistingSmc::update overload: non-finite value");
  }
  if (dt <= zero()) {
    throw std::runtime_error("SuperTwistingSmc::update overload: non-positive dt");
  }

  Vector6 d_error{};
  if (has_prev_error_) {
    for (std::size_t i = 0; i < 6; ++i) {
      d_error(i) = (error(i) - prev_error_(i)) / dt;
    }
  } else {
    d_error = Vector6{};
  }

  return update(error, d_error, feedforward, dt);
}

Vector6 SuperTwistingSmc::update(const Vector6& error, Real dt) {
  return update(error, Vector6{}, dt);
}


Real SuperTwistingSmc::sat(Real s, Real eps) {
  if (eps <= zero()) {
    if (s > zero()) {
      return one();
    }
    if (s < zero()) {
      return -one();
    }
    return zero();
  }

  const Real ratio = s / eps;
  if (ratio > one()) {
    return one();
  }
  if (ratio < -one()) {
    return -one();
  }
  return ratio;
}

}  // namespace sfc
