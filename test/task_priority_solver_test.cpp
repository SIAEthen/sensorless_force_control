#include <iostream>

#include "functionlib/task_priority_control/task_priority_solver.h"

int main() {
  using sfc::Matrix;
  using sfc::Real;
  using sfc::Vector;
  using sfc::taskPrioritySolveStep;

  constexpr std::size_t kDof = 7;

  Matrix<3, kDof> J1{};
  J1(0, 0) = 0.8;  J1(0, 1) = 0.0;  J1(0, 2) = 0.1;  J1(0, 3) = -0.2; J1(0, 4) = 0.0;  J1(0, 5) = 0.3;  J1(0, 6) = -0.1;
  J1(1, 0) = 0.0;  J1(1, 1) = 0.9;  J1(1, 2) = -0.1; J1(1, 3) = 0.0;  J1(1, 4) = 0.2;  J1(1, 5) = -0.2; J1(1, 6) = 0.0;
  J1(2, 0) = 0.1;  J1(2, 1) = -0.2; J1(2, 2) = 0.7;  J1(2, 3) = 0.0;  J1(2, 4) = -0.1; J1(2, 5) = 0.0;  J1(2, 6) = 0.4;

  Matrix<2, kDof> J2{};
  J2(0, 0) = 0.2;  J2(0, 1) = -0.1; J2(0, 2) = 0.0;  J2(0, 3) = 0.6;  J2(0, 4) = 0.0;  J2(0, 5) = -0.3; J2(0, 6) = 0.1;
  J2(1, 0) = 0.0;  J2(1, 1) = 0.3;  J2(1, 2) = -0.2; J2(1, 3) = 0.0;  J2(1, 4) = 0.5;  J2(1, 5) = 0.0;  J2(1, 6) = -0.4;

  Vector<3> sigma_dot1{};
  sigma_dot1(0) = 0.1;
  sigma_dot1(1) = 0.0;
  sigma_dot1(2) = -0.1;

  Vector<2> sigma_dot2{};
  sigma_dot2(0) = 0.05;
  sigma_dot2(1) = -0.02;

  Vector<kDof> zeta{};
  Matrix<kDof, kDof> N = sfc::identity<kDof>();

  const Real damping = static_cast<Real>(1e-3);
  zeta = taskPrioritySolveStep<kDof, 3>(sigma_dot1, J1, N, zeta, damping);
  zeta = taskPrioritySolveStep<kDof, 2>(sigma_dot2, J2, N, zeta, damping);

  std::cout << "zeta: ";
  for (std::size_t i = 0; i < kDof; ++i) {
    std::cout << zeta(i);
    if (i + 1 < kDof) {
      std::cout << ", ";
    }
  }
  std::cout << "\n";

  return 0;
}
