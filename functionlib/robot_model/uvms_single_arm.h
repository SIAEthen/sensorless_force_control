#ifndef SFC_UVMS_SINGLE_ARM_H_
#define SFC_UVMS_SINGLE_ARM_H_

#include <string>

#include "robot_model/manipulator_base.h"
#include "robot_model/manipulator_from_yaml.h"
#include "robot_model/vehicle_base.h"
#include "utilts/matrix.h"
#include "utilts/rotation.h"
#include "utilts/vector.h"

namespace sfc {

template <std::size_t kUvmsArmDof,
          typename ManipulatorT = ManipulatorBase<kUvmsArmDof>,
          typename VehicleT = VehicleBase>
class UvmsSingleArm {
public:
  using Manipulator = ManipulatorT;
  using Vehicle = VehicleT;
  using UvmsJacobian = Matrix<6, 6 + kUvmsArmDof>;

  // Frames: 
  // world is the inertial frame (ned)
  // vehicle frame is body FRD (b), 
  // manipulator frames are 0..kUvmsArmDof
  explicit UvmsSingleArm(const HomogeneousMatrix& t_0_b = HomogeneousMatrix::identity())
      : vehicle_("vehicle"),
        manipulator_("arm"),
        t_0_b_(t_0_b) {}

  Vehicle& vehicle() { return vehicle_; }
  const Vehicle& vehicle() const { return vehicle_; }

  Manipulator& manipulator() { return manipulator_; }
  const Manipulator& manipulator() const { return manipulator_; }

  void setManipulator(const Manipulator& manipulator) { manipulator_ = manipulator; }
  void setVehicle(const Vehicle& vehicle) { vehicle_ = vehicle; }

  Vector3 vehiclePosition() const {
    return Vector3{vehicle_.position()[0],
                   vehicle_.position()[1],
                   vehicle_.position()[2]};
  }

  Vector3 vehicleRpy() const { return vehicle_.rpy(); }

  Quaternion vehicleQuaternion() const { return Quaternion::fromRPY(vehicle_.rpy()); }

  Vector6 vehicleVelocity() const {
    Vector6 out{};
    const auto& nu = vehicle_.velocity();
    for (std::size_t i = 0; i < VehicleBase::kDof; ++i) {
      out(i) = nu[i];
    }
    return out;
  }

  Vector<kUvmsArmDof> manipulatorPosition() const {
    Vector<kUvmsArmDof> out{};
    const auto& q = manipulator_.jointPosition();
    for (std::size_t i = 0; i < kUvmsArmDof; ++i) {
      out(i) = q[i];
    }
    return out;
  }

  Vector<kUvmsArmDof> manipulatorVelocity() const {
    Vector<kUvmsArmDof> out{};
    const auto& dq = manipulator_.jointVelocity();
    for (std::size_t i = 0; i < kUvmsArmDof; ++i) {
      out(i) = dq[i];
    }
    return out;
  }

  void setVehicleState(const VehicleBase::State& state) { vehicle_.setState(state); }

  VehicleBase::State vehicleState() const { return vehicle_.state(); }

  void setManipulatorState(const typename Manipulator::State& state) { manipulator_.setState(state); }

  typename Manipulator::State manipulatorState() const { return manipulator_.state(); }

  void setManipulatorBaseToVehicleTransform(const HomogeneousMatrix& t_0_b) { t_0_b_ = t_0_b; }

  const HomogeneousMatrix& manipulatorBaseToVehicleTransform() const { return t_0_b_; }

  // Returns T_ee_ned. For UVMS, world frame is NED.
  HomogeneousMatrix forwardKinematics() const {
    return vehicle_.forwardKinematics() * t_0_b_ * manipulator_.forwardKinematics();
  }

  Vector3 endEffectorPositionNed() const { return forwardKinematics().translation(); }

  Quaternion endEffectorQuaternionNed() const {
    const RotationMatrix r = forwardKinematics().rotation();
    return Quaternion::fromRotationMatrix(r);
  }

  UvmsJacobian jacobian() const {
    const RotationMatrix r_b_ned =
        vehicle_.getRotationMatrixBaseToInertial();
    const Vector3 p_b_ned{vehicle_.position()[0],
                          vehicle_.position()[1],
                          vehicle_.position()[2]};

    const Vector3 p_b0_b =
        t_0_b_.translation();
    const RotationMatrix r_0_b =
        t_0_b_.rotation();

    const RotationMatrix r_0_ned = r_b_ned * r_0_b;

    const HomogeneousMatrix t_ee_ned = forwardKinematics();
    const Vector3 p_ee_ned =
        t_ee_ned.translation();
    const Vector3 p_b0_ned = r_b_ned.m * p_b0_b;
    const Vector3 p_0ee_ned =
        p_ee_ned - p_b_ned - p_b0_ned;

    const typename Manipulator::Jacobian j_man = manipulator_.jacobian();
    Matrix<3, kUvmsArmDof> j_man_pos{};
    Matrix<3, kUvmsArmDof> j_man_ori{};
    for (std::size_t col = 0; col < kUvmsArmDof; ++col) {
      for (std::size_t row = 0; row < 3; ++row) {
        j_man_pos(row, col) = j_man(row, col);
        j_man_ori(row, col) = j_man(row + 3, col);
      }
    }

    const Matrix3 r_b_ned_m = r_b_ned.m;
    const Matrix3 r_0_ned_m = r_0_ned.m;
    const Matrix3 S_p_b0_ned = skew(p_b0_ned);
    const Matrix3 S_p_0ee_ned = skew(p_0ee_ned);
    const Matrix3 jr_cross =
        (S_p_b0_ned + S_p_0ee_ned) * r_b_ned_m;
    const Matrix3 jr_linear = Matrix3{} - jr_cross;

    const Matrix<3, kUvmsArmDof> j_pos = r_0_ned_m * j_man_pos;
    const Matrix<3, kUvmsArmDof> j_ori = r_0_ned_m * j_man_ori;

    const auto j_linear =
        hstack(hstack(r_b_ned_m, jr_linear), j_pos);
    const auto j_angular =
        hstack(hstack(zeros<3, 3>(), r_b_ned_m), j_ori);
    return vstack(j_linear, j_angular);
  }

private:
  Vehicle vehicle_;
  Manipulator manipulator_;
  HomogeneousMatrix t_0_b_;

};

}  // namespace sfc

#endif  // SFC_UVMS_SINGLE_ARM_H_
