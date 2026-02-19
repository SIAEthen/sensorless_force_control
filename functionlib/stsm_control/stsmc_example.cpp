#include <cmath>
#include <iostream>

#include "functionlib/stsm_control/super_twisting_smc.h"

namespace {

using sfc::Real;
using sfc::SuperTwistingSmc;
using sfc::Vector6;

bool near(Real a, Real b, Real tol = static_cast<Real>(1e-6)) {
  return std::fabs(a - b) <= tol;
}

bool testBasicSaturationBranch() {
  SuperTwistingSmc smc;
  smc.setGains(Vector6{2, 2, 2, 2, 2, 2},
               Vector6{1, 1, 1, 1, 1, 1},
               Vector6{3, 3, 3, 3, 3, 3});
  smc.setBoundaryLayer(Vector6{0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
  smc.reset();

  const Vector6 error{1, 1, 1, 1, 1, 1};
  const Vector6 d_error{0, 0, 0, 0, 0, 0};
  const Vector6 feedforward{0, 0, 0, 0, 0, 0};
  const Real dt = static_cast<Real>(0.1);

  const Vector6 u1 = smc.update(error, d_error, feedforward, dt);
  const Vector6 u2 = smc.update(error, d_error, feedforward, dt);

  const Real s = static_cast<Real>(3.0);
  const Real sigma = static_cast<Real>(1.0);
  const Real sqrt_abs_s = std::sqrt(std::fabs(s));
  const Real expected_u1 = -static_cast<Real>(2.0) * sqrt_abs_s * sigma;
  const Real expected_u2 = expected_u1 - static_cast<Real>(0.1);

  for (std::size_t i = 0; i < 6; ++i) {
    if (!near(u1(i), expected_u1) || !near(u2(i), expected_u2)) {
      std::cerr << "[FAIL] saturation branch at channel " << i
                << ", u1=" << u1(i) << ", u2=" << u2(i)
                << ", expected_u1=" << expected_u1
                << ", expected_u2=" << expected_u2 << std::endl;
      return false;
    }
  }
  return true;
}

bool testBoundaryLayerLinearBranch() {
  SuperTwistingSmc smc;
  smc.setGains(Vector6{1, 1, 1, 1, 1, 1},
               Vector6{2, 2, 2, 2, 2, 2},
               Vector6{1, 1, 1, 1, 1, 1});
  smc.setBoundaryLayer(Vector6{10, 10, 10, 10, 10, 10});
  smc.reset();

  const Vector6 error{1, 1, 1, 1, 1, 1};
  const Vector6 d_error{0, 0, 0, 0, 0, 0};
  const Vector6 feedforward{0, 0, 0, 0, 0, 0};
  const Real dt = static_cast<Real>(0.1);

  const Vector6 u = smc.update(error, d_error, feedforward, dt);
  const Real expected_u = static_cast<Real>(-0.1);  // -k1*sqrt(1)*sat(1/10)

  for (std::size_t i = 0; i < 6; ++i) {
    if (!near(u(i), expected_u)) {
      std::cerr << "[FAIL] boundary layer branch at channel " << i
                << ", u=" << u(i) << ", expected_u=" << expected_u
                << std::endl;
      return false;
    }
  }
  return true;
}

bool testFeedforwardAddition() {
  SuperTwistingSmc smc;
  smc.setGains(Vector6{1, 1, 1, 1, 1, 1},
               Vector6{0, 0, 0, 0, 0, 0},
               Vector6{0, 0, 0, 0, 0, 0});
  smc.setBoundaryLayer(Vector6{0, 0, 0, 0, 0, 0});
  smc.reset();

  const Vector6 error{0, 0, 0, 0, 0, 0};
  const Vector6 d_error{0, 0, 0, 0, 0, 0};
  const Vector6 feedforward{1, 2, 3, 4, 5, 6};
  const Real dt = static_cast<Real>(0.1);

  const Vector6 u = smc.update(error, d_error, feedforward, dt);
  for (std::size_t i = 0; i < 6; ++i) {
    if (!near(u(i), feedforward(i))) {
      std::cerr << "[FAIL] feedforward addition at channel " << i
                << ", u=" << u(i) << ", expected=" << feedforward(i)
                << std::endl;
      return false;
    }
  }
  return true;
}

}  // namespace

int main() {
  bool ok = true;

  ok = testBasicSaturationBranch() && ok;
  ok = testBoundaryLayerLinearBranch() && ok;
  ok = testFeedforwardAddition() && ok;

  if (!ok) {
    std::cerr << "stsmc_test: FAILED" << std::endl;
    return 1;
  }

  std::cout << "stsmc_test: PASSED" << std::endl;
  return 0;
}
