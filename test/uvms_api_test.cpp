#include <iostream>

#include "functionlib/robot_model/uvms_single_arm.h"
#include "functionlib/pid/pd.h"
#include "functionlib/task_priority_control/possible_tasks.h"
#include "functionlib/task_priority_control/task_priority_solver.h"
#include "functionlib/utilts/real.h"
#include "functionlib/utilts/linear_algebra.h"

int main() {
  constexpr std::size_t kArmDof = 7;
  sfc::UvmsSingleArm<kArmDof> uvms;

  sfc::RotationMatrix r = sfc::RotationMatrix::fromRPY(0.1, 0.2, 0.3);
  sfc::Vector3 t = sfc::Vector3{0.1, 0.2, 0.5};
  sfc::HomogeneousMatrix t_0_b = sfc::HomogeneousMatrix::fromRotationTranslation(r, t);
  uvms.setManipulatorBaseToVehicleTransform(t_0_b);


  std::array<sfc::DHParam, kArmDof> dh{};
  const std::array<sfc::Real, kArmDof> dh_a{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  const std::array<sfc::Real, kArmDof> dh_alpha{{-sfc::kPi2, sfc::kPi2,
                                              -sfc::kPi2, sfc::kPi2,
                                              -sfc::kPi2, sfc::kPi2,
                                              0.0}};
  const std::array<sfc::Real, kArmDof> dh_d{{0.213, 0.0, 0.427, 0.0, 0.42, 0.0, 0.153}};
  const std::array<sfc::Real, kArmDof> dh_theta{{0.0, sfc::deg2rad(40.0), 0.0,
                                              sfc::deg2rad(-10.0), 0.0,
                                              sfc::deg2rad(-15.0), 0.0}};
  for (std::size_t i = 0; i < kArmDof; ++i) {
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

  sfc::UvmsSingleArm<kArmDof>::Manipulator::State manip_state{};
  manip_state.q = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  uvms.setManipulatorState(manip_state);


  const sfc::HomogeneousMatrix t_ee_ned = uvms.forwardKinematics(); 
  const auto j = uvms.jacobian();

  const auto j_inv = sfc::pseudoInverseDls(j,0.00001);

  sfc::printMatrix(t_ee_ned.m, std::cout, "t_ee_ned"); // pass test
  sfc::printMatrix(j, std::cout, "uvms_jacobian"); // pass test
  sfc::printMatrix(j_inv, std::cout, "uvms_jacobian_inv"); // pass test

  // ---------------- task-priority solver demo ----------------
  // Define your tasks here.
  // Example usage:
  //   1) build task Jacobian + dsigma
  //   2) call taskPrioritySolveStep sequentially
  constexpr std::size_t kSysDof = 6 + kArmDof;
  sfc::Matrix<kSysDof, kSysDof> N = sfc::identity<kSysDof>();
  sfc::Vector<kSysDof> zeta{};
  const sfc::Real damping = static_cast<sfc::Real>(1e-6);

  // Task 1: roll/pitch stabilization
  sfc::Matrix<2, kSysDof> J_rp{};
  sfc::Vector<2> sigma_rp{};
  const sfc::Vector<2> rp_ref{};
  sfc::buildRollPitchTask(uvms, rp_ref, J_rp, sigma_rp);
  zeta = sfc::taskPrioritySolveStep<kSysDof, 2>(sigma_rp, J_rp, N, zeta, damping);

  // Task 2: end-effector task (set your references)
  sfc::Matrix<6, kSysDof> J_ee{};
  sfc::Vector<6> sigma_ee{};
  const sfc::Vector3 pos_ref = uvms.endEffectorPositionNed();
  const sfc::Quaternion q_ref = uvms.endEffectorQuaternionNed();
  sfc::buildEeTask(uvms, pos_ref, q_ref, J_ee, sigma_ee);
  zeta = sfc::taskPrioritySolveStep<kSysDof, 6>(sigma_ee, J_ee, N, zeta, damping);

  // Task 3: nominal joint configuration
  sfc::Matrix<kArmDof, kSysDof> J_nominal{};
  sfc::Vector<kArmDof> sigma_nominal{};
  const sfc::Vector<kArmDof> nominal_config = sfc::Vector<kArmDof>{};
  sfc::buildNominalConfigTask(uvms, nominal_config, J_nominal, sigma_nominal);
  zeta = sfc::taskPrioritySolveStep<kSysDof, kArmDof>(sigma_nominal, J_nominal, N, zeta, damping);

  sfc::printMatrix(J_rp, std::cout, "task_J_rp");
  sfc::printMatrix(J_ee, std::cout, "task_J_ee");
  sfc::printMatrix(J_nominal, std::cout, "task_J_nominal");
  std::cout << "zeta: ";
  for (std::size_t i = 0; i < kSysDof; ++i) {
    std::cout << zeta(i);
    if (i + 1 < kSysDof) {
      std::cout << ", ";
    }
  }
  std::cout << "\n";

  // ---------------- body velocity PD controller ----------------
  sfc::PdController<6> body_vel_pd{};
  sfc::Vector6 kp{};
  sfc::Vector6 kd{};
  for (std::size_t i = 0; i < 6; ++i) {
    kp(i) = static_cast<sfc::Real>(5.0);
    kd(i) = static_cast<sfc::Real>(1.0);
  }
  body_vel_pd.setGains(kp, kd);

  const sfc::Vector6 vel_now = uvms.vehicleVelocity();
  sfc::Vector6 vel_ref{};
  for (std::size_t i = 0; i < 6; ++i) {
    vel_ref(i) = zeta(i);
  }

  const sfc::Vector6 vel_err = vel_ref - vel_now;
  const sfc::Real dt = static_cast<sfc::Real>(0.02);
  const sfc::Vector6 tau = body_vel_pd.update(vel_err, dt);
  sfc::printVector(tau, std::cout, "body_velocity_tau");

  return 0;
}
