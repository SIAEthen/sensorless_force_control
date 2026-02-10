#include <array>
#include <iostream>
#include <string>
#include <vector>

#include "functionlib/robot_model/manipulator_from_yaml.h"
#include "functionlib/utilts/linear_algebra.h"
#include "functionlib/utilts/print.h"
#include "functionlib/robot_model/uvms_single_arm.h"


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

  std::cout << "rpy_ee: [" << rpy_ee(0) << ", " << rpy_ee(1) << ", " << rpy_ee(2) << "]\n";
  sfc::print(quat_ee,std::cout, "Quaternion_ee");

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
  vehicle_state.velocity = {};
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



  return 0;
}
