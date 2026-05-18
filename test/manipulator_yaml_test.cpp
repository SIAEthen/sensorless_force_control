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


  // ── Beta-transition test: EE task (beta 1→0) + nominal config task ─────────
  // We simulate N steps. EE task starts fully active, then we call deactivate()
  // so beta ramps from 1 down to 0 over T_transition seconds.
  //
  // zeta_star for the EE task is always the pure nominal-config solution,
  // so at beta=0 the EE task collapses to "track zeta_star" and the two
  // tasks agree → no jump expected.
  //
  // Compare with the "naive" column where we simply drop the EE task at beta=0
  // (which CAN jump because the solver suddenly sees a different problem).

  std::cout << "\n\n=== Beta-transition test ===\n";
  std::cout << std::fixed << std::setprecision(4);

  // Task parameters (shared by both columns)
  const sfc::Vector3  pos_ref_t = uvms.endEffectorPositionNed() + sfc::Vector3{0.5, 0.3, -0.2};
  const sfc::Quaternion q_ref_t = uvms.endEffectorQuaternionNed();
  const sfc::Vector6  gain_ee_t{1,1,1,1,1,1};
  const sfc::Vector<12> Kq_ee_t{1,1,1,1,1,1, 1,1,1,1,1,1};
  const sfc::Vector<6>  Kw_ee_t{1,1,1,1,1,1};

  const sfc::Vector6  nominal_config_t{0,0,0,0,0,0.5};
  const sfc::Vector6  nc_gain_t{1,1,1,1,1,1};
  const sfc::Vector<12> Kq_nc_t{1,1,1,1,1,1, 1,1,1,1,1,1};
  const sfc::Vector<6>  Kw_nc_t{0.1,0.1,0.1,0.1,0.1,0.1};

  const double dt_t          = 0.1;    // simulated step [s]
  const double T_transition  = 2.0;   // time to ramp beta from 1→0 [s]
  const int    N_steps       = 21;    // total steps (covers transition + a bit after)

  hqp::TaskActivator ee_act_smooth;   // smooth blend (with zeta_star)
  hqp::TaskActivator ee_act_naive;    // naive: just remove task at beta=0
  ee_act_smooth.reset(1.0);
  ee_act_naive.reset(1.0);
  ee_act_smooth.deactivate();
  ee_act_naive.deactivate();

  ee_act_smooth.reset(0.0);
  ee_act_naive.reset(0.0);
  ee_act_smooth.activate();
  ee_act_naive.activate();


  for (int step = 0; step < N_steps; ++step) {
    ee_act_smooth.stepTime(dt_t, T_transition);
    ee_act_naive.stepTime(dt_t, T_transition);
    const double beta = ee_act_smooth.beta();

    // ── smooth column ────────────────────────────────────────────────────────
    // zeta_star = pure nominal-config solution (what we want to fall back to)
    sfc::Vector<12> zeta_star_t{};
    hqp::HQPCascadedResult res_zeta_star;
    {
      hqp::HQPCascadedSolver nc_solver(12);
      nc_solver.addTask(hqp::makeNominalConfigTask(uvms, nominal_config_t, nc_gain_t, Kq_nc_t, Kw_nc_t));
      res_zeta_star = nc_solver.solve();
      zeta_star_t = hqp::toVector<12>(res_zeta_star.qdot);
    }

    hqp::HQPCascadedSolver s_smooth(12);
    s_smooth.addTask(hqp::makeEeTask(uvms, pos_ref_t, q_ref_t, gain_ee_t,
                                     Kq_ee_t, Kw_ee_t, 0.01, 0.001, beta, zeta_star_t));
    s_smooth.addTask(hqp::makeNominalConfigTask(uvms, nominal_config_t, nc_gain_t, Kq_nc_t, Kw_nc_t));
    const auto res_smooth = s_smooth.solve();
    const sfc::Vector<12> zeta_smooth = hqp::toVector<12>(res_smooth.qdot);

    // ── naive column ─────────────────────────────────────────────────────────
    // When beta hits 0, simply don't add the EE task at all
    hqp::HQPCascadedSolver s_naive(12);
    if (!ee_act_naive.isFullyInactive() && !ee_act_naive.isFullyActive()) {
      s_naive.addTask(hqp::makeEeTask(uvms, pos_ref_t, q_ref_t, gain_ee_t,
                                      Kq_ee_t, Kw_ee_t, 0.01, 0.001, ee_act_naive.beta(),zeta_star_t));
    }
    if(ee_act_naive.isFullyActive()){
      s_naive.addTask(hqp::makeEeTask(uvms, pos_ref_t, q_ref_t, gain_ee_t,
                                      Kq_ee_t, Kw_ee_t, 0.01, 0.001));
    }
    // if(step ==19){
    //   s_naive.addTask(hqp::makeEeTask(uvms, pos_ref_t, q_ref_t, gain_ee_t,
    //                                   Kq_ee_t, Kw_ee_t, 0.01, 0.001, ee_act_naive.beta(),zeta_star_t));
    // }

    s_naive.addTask(hqp::makeNominalConfigTask(uvms, nominal_config_t, nc_gain_t, Kq_nc_t, Kw_nc_t));
    const auto res_naive = s_naive.solve();
    const sfc::Vector<12> zeta_naive = hqp::toVector<12>(res_naive.qdot);

    // ── print ────────────────────────────────────────────────────────────────
    std::cout << "step=" << std::setw(3) << step
              << "  beta=" << std::setw(6) << beta << "\n";

    // zeta_star
    std::cout << "  zeta_star dq: ";
    for (int i = 6; i < 12; ++i) std::cout << std::setw(8) << zeta_star_t(i);
    std::cout << "\n  slacks zeta_star:\n";
    for (std::size_t lvl = 0; lvl < res_zeta_star.slacks.size(); ++lvl) {
      const Eigen::VectorXd& w = res_zeta_star.slacks[lvl];
      std::cout << "    [lvl" << lvl << "] (|w|=" << std::setprecision(6) << w.norm() << ")  "
                << w.transpose() << "\n";
    }

    // joint velocities side-by-side
    std::cout << "  dq  smooth: ";
    for (int i = 6; i < 12; ++i) std::cout << std::setw(8) << zeta_smooth(i);
    std::cout << "\n  dq  naive:  ";
    for (int i = 6; i < 12; ++i) std::cout << std::setw(8) << zeta_naive(i);
    std::cout << "\n";

    // slacks for smooth solver
    std::cout << "  slacks smooth:\n";
    for (std::size_t lvl = 0; lvl < res_smooth.slacks.size(); ++lvl) {
      const Eigen::VectorXd& w = res_smooth.slacks[lvl];
      std::cout << "    [" << s_smooth.taskName(lvl) << "] (|w|="
                << std::setprecision(6) << w.norm() << ")  "
                << w.transpose() << "\n";
    }

    // slacks for naive solver
    std::cout << "  slacks naive:\n";
    for (std::size_t lvl = 0; lvl < res_naive.slacks.size(); ++lvl) {
      const Eigen::VectorXd& w = res_naive.slacks[lvl];
      std::cout << "    [" << s_naive.taskName(lvl) << "] (|w|="
                << std::setprecision(6) << w.norm() << ")  "
                << w.transpose() << "\n";
    }
    std::cout << "\n";
  }

  

  return 0;
}
