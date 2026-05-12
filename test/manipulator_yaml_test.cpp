#include <array>
#include <iostream>
#include <string>
#include <vector>

#include "functionlib/robot_model/manipulator_from_yaml.h"
#include "functionlib/utilts/linear_algebra.h"
#include "functionlib/utilts/print.h"
#include "functionlib/robot_model/uvms_single_arm.h"
#include "functionlib/hqp/hqp_cascaded_solver.h"
#include "functionlib/hqp/hqp_tasks.h"
#include <iomanip>
#include <cassert>
#include <cmath>

static void print(const std::string& label, const hqp::HQPCascadedResult& r) {
    std::cout << "\n[" << label << "]  success=" << r.success << "\n";
    std::cout << "  qdot = " << r.qdot.transpose() << "\n";
    for (int k = 0; k < (int)r.slacks.size(); ++k)
        std::cout << "  w" << k+1 << " = " << r.slacks[k].transpose()
                  << "  (|w|=" << std::fixed << std::setprecision(6)
                  << r.slacks[k].norm() << ")\n";
}


int main() {
  constexpr std::size_t kDof = 6;
  sfc::ManipulatorFromYAML<kDof> manip("bravo");

  const std::string yaml_path =
      "/home/sia/girona_ws/src/sensorless_force_control/config/control/bravo_joints.yaml";
  const std::array<std::string, kDof> joint_names = {{
      "bravo/joint1",
      "bravo/joint2",
      "bravo/joint3",
      "bravo/joint4",
      "bravo/joint5",
      "bravo/joint6",
  }};

  manip.setParametersFromFile(yaml_path, joint_names);
  sfc::HomogeneousMatrix t_tool_linkend = sfc::HomogeneousMatrix::fromRotationTranslation(sfc::RotationMatrix::fromRPY(0,0,-2.88),
                                                                                        sfc::Vector3{0.084, 0.022, 0.372});
  manip.setToolTransformationFromT(t_tool_linkend);

  sfc::ManipulatorBase<kDof>::State state{};
  state.q = {0.0,0.0,0.0,0.0,0.0,0.0};
  manip.setState(state);

  const sfc::HomogeneousMatrix t_ee = manip.forwardKinematics();
  const auto j = manip.jacobian();

  sfc::printMatrix(t_ee.m, std::cout, "T_ee");
  sfc::printMatrix(j, std::cout, "Jacobian");

  const sfc::Vector3 rpy_ee = sfc::RotationMatrix2RPY(t_ee.rotation());
  const sfc::Quaternion quat_ee = sfc::Quaternion::fromRotationMatrix(t_ee.rotation());
  // PASS TEST
  // rpy_ee: [-2.68268e-16, -5.49549e-17, 0.261593]
  // Quaternion_ee: [w=0.991458, x=-1.29405e-16, y=-4.4737e-17, z=0.130424]
  //   xyz ee (No tool) (3)
  // 0.2934 -0.022 0.55318
  std::cout << "rpy_ee: [" << rpy_ee(0) << ", " << rpy_ee(1) << ", " << rpy_ee(2) << "]\n";
  sfc::print(quat_ee,std::cout, "Quaternion_ee");
  sfc::print(t_ee.translation(),std::cout, "xyz ee (with tool)");



// PASS TEST
// At time 1770299967.670
// - Translation: [0.199, 0.254, 0.417]
// - Rotation: in Quaternion [-0.366, 0.351, 0.263, 0.821]
//         in RPY (radian) [-0.707, 0.876, 0.279]
//         in RPY (degree) [-40.480, 50.214, 15.981]
  state.q = {-1.027734158266263, -0.4482511677957936, -1.0333947045786023, -1.0279999999999194, -1.0291720713433727, -1.031403839752819};
  manip.setState(state);
  const sfc::HomogeneousMatrix t_ee_config2 = manip.forwardKinematics();
  const sfc::Vector3 rpy_ee_config2 = sfc::RotationMatrix2RPY(t_ee_config2.rotation());
  const sfc::Quaternion quat_ee_config2 = sfc::Quaternion::fromRotationMatrix(t_ee_config2.rotation());
  sfc::print(t_ee_config2.translation(),std::cout, "xyz_config2");
  sfc::print(rpy_ee_config2,std::cout, "rpy_config2");
  sfc::print(quat_ee_config2,std::cout, "quat_config2");

  constexpr std::size_t kArmDof = 6;
  sfc::UvmsSingleArm<kArmDof, sfc::ManipulatorFromYAML<kArmDof>> uvms;
  // read from rosrun tf tf_echo girona1000/base_link girona1000/bravo/base_link
  sfc::RotationMatrix r = sfc::RotationMatrix::fromRPY(3.142, 0.000, -0.175);
  sfc::Vector3 t = sfc::Vector3{0.732, -0.138, 0.271};
  sfc::HomogeneousMatrix t_0_b = sfc::HomogeneousMatrix::fromRotationTranslation(r, t);
  uvms.setManipulatorBaseToVehicleTransform(t_0_b);
  uvms.manipulator() = manip;

  sfc::VehicleBase::State vehicle_state{};
  vehicle_state.position = {};
  vehicle_state.velocity = {1,1,1,1,1,1};
  uvms.setVehicleState(vehicle_state);

  sfc::UvmsSingleArm<kArmDof>::Manipulator::State manip_state{};
  manip_state.q = {-1.027734158266263, -0.4482511677957936, -1.0333947045786023, -1.0279999999999194, -1.0291720713433727, -1.031403839752819};
  uvms.setManipulatorState(manip_state);

// PASS TEST 2026-02-05
// At time 1770301549.210
// - Translation: [0.884, -0.422, -0.146]
// - Rotation: in Quaternion [0.795, -0.334, 0.317, 0.395]
//             in RPY (radian) [2.435, -0.876, -0.454]
//             in RPY (degree) [139.520, -50.214, -26.008]
  const sfc::HomogeneousMatrix t_ee_ned = uvms.forwardKinematics(); 
  sfc::print(t_ee_ned.translation(),std::cout,"xyz");
  sfc::print(sfc::rpyFromRotationMatrix(t_ee_ned.rotation()),std::cout,"rpy");



  const int n = 12; // uvms 12 dof
  hqp::HQPCascadedSolver solver(n);
  solver.clearTasks();
//task: Joint Limits
  const sfc::Vector<6> q_min{-1.5*sfc::kPi4,-sfc::kPi2,-sfc::kPi2,-3*sfc::kPi4, -sfc::kPi, 0.0};
  const sfc::Vector<6> q_max{ 1.5*sfc::kPi4, sfc::kPi2, sfc::kPi2, 3*sfc::kPi4,  0.0,      3.5*sfc::kPi4};
  const sfc::Vector<6> dq_min{-0.30, -0.30, -0.30, -0.30, -0.30, -0.30};
  const sfc::Vector<6> dq_max{0.30, 0.30, 0.30, 0.30, 0.30, 0.30};
  const sfc::Vector<6> nu_min{-0.10,-0.10,-0.10,-0.10,-0.10,-0.10};
  const sfc::Vector<6> nu_max{0.10,0.10,0.10,0.10,0.10,0.10};
  sfc::Vector<12> Kq_jl = sfc::Vector<12>{1,1,1,1,1,1, 1,1,1,1,1,1};
  sfc::Vector<12> Kw_jl = sfc::Vector<12>{1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};

  solver.addTask(hqp::makeSystemConstraintsTask(uvms, nu_min,nu_max,
                                                  q_min, q_max,dq_min,dq_max, Kq_jl, Kw_jl));


  sfc::Vector<2> gain_rp = sfc::Vector<2>{1,1};
  sfc::Vector<2> rp_ref = sfc::Vector<2>{0,0.5};
  sfc::Vector<12> Kq_rp = sfc::Vector<12>{1,1,1,5,5,1, 1,1,1,1,1,1};
  sfc::Vector<2> Kw_rp = sfc::Vector<2>{1,1};
  solver.addTask(hqp::makeRollPitchTask(uvms, rp_ref, gain_rp, Kq_rp, Kw_rp));

  sfc::Vector<6> gain_ee = sfc::Vector<6>{1,1,1,1,1,1};
  const sfc::Vector3 pos_ref = uvms.endEffectorPositionNed()+sfc::Vector3{1,1,1};
  const sfc::Quaternion q_ref = uvms.endEffectorQuaternionNed();
  sfc::Vector<12> Kq_ee = sfc::Vector<12>{1,1,1,1,1,1, 1,1,1,1,1,1};
  sfc::Vector<6> Kw_ee = sfc::Vector<6>{1,1,1,1,1,1};
  solver.addTask(hqp::makeEeTask(uvms, pos_ref, q_ref, gain_ee,Kq_ee,Kw_ee));

  solver.addTask(hqp::makeZeroVelocityTask(uvms,1,10));

  auto res = solver.solve();
  Eigen::VectorXd zeta_eigen = res.qdot;
  sfc::print(hqp::toVector<12>(zeta_eigen),std::cout,"zeta_hqp");

  for (std::size_t lvl = 0; lvl < res.slacks.size(); ++lvl) {
    const Eigen::VectorXd& w = res.slacks[lvl];
    std::cout << "slack[" << lvl << "] (size=" << w.size() << "): " << w.transpose() << "\n";
  }


  return 0;
}
