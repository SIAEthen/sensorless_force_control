#include <iostream>

#include "functionlib/hardware/uvms_single_arm.h"
#include "functionlib/utilts/real.h"
#include "functionlib/utilts/linear_algebra.h"

int main() {
  constexpr std::size_t kDof = 7;
  sfc::UvmsSingleArm<kDof> uvms("vehicle", "arm");

  sfc::RotationMatrix r = sfc::RotationMatrix::fromRPY(0.1, 0.2, 0.3);
  sfc::Vector3 t = sfc::Vector3{0.1, 0.2, 0.5};
  sfc::HomogeneousMatrix t_0_b = sfc::HomogeneousMatrix::fromRotationTranslation(r, t);
  uvms.setManipulatorBaseToVehicleTransform(t_0_b);


  std::array<sfc::UvmsSingleArm<kDof>::Manipulator::DHParam, kDof> dh{};
  const std::array<sfc::Real, kDof> dh_a{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  const std::array<sfc::Real, kDof> dh_alpha{{-sfc::kPi2, sfc::kPi2,
                                              -sfc::kPi2, sfc::kPi2,
                                              -sfc::kPi2, sfc::kPi2,
                                              0.0}};
  const std::array<sfc::Real, kDof> dh_d{{0.213, 0.0, 0.427, 0.0, 0.42, 0.0, 0.153}};
  const std::array<sfc::Real, kDof> dh_theta{{0.0, sfc::deg2rad(40.0), 0.0,
                                              sfc::deg2rad(-10.0), 0.0,
                                              sfc::deg2rad(-15.0), 0.0}};
  for (std::size_t i = 0; i < kDof; ++i) {
    dh[i].a = dh_a[i];
    dh[i].alpha = dh_alpha[i];
    dh[i].d = dh_d[i];
    dh[i].theta = dh_theta[i];
  }
  uvms.manipulator().setDHParameters(dh);



  sfc::VehicleBase::State vehicle_state{};
  vehicle_state.position = {1.0, 2.0, -0.5,
                            sfc::deg2rad(5.0), sfc::deg2rad(-2.0), sfc::deg2rad(15.0)};
  vehicle_state.velocity = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0};
  uvms.setVehicleState(vehicle_state);

  sfc::UvmsSingleArm<kDof>::Manipulator::State manip_state{};
  manip_state.q = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  uvms.setManipulatorState(manip_state);


  const sfc::HomogeneousMatrix t_ee_ned = uvms.forwardKinematics(); 
  const auto j = uvms.jacobian();

  const auto j_inv = sfc::pseudoInverseDls(j,0.00001);

  sfc::printMatrix(t_ee_ned.m, std::cout, "t_ee_ned"); // pass test
  sfc::printMatrix(j, std::cout, "uvms_jacobian"); // pass test
  sfc::printMatrix(j_inv, std::cout, "uvms_jacobian_inv"); // pass test

  return 0;
}
