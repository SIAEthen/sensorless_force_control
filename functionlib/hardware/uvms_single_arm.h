#ifndef SFC_UVMS_SINGLE_ARM_H_
#define SFC_UVMS_SINGLE_ARM_H_

#include <string>

#include "hardware/manipulator_base.h"
#include "hardware/vehicle_base.h"
#include "utilts/rotation.h"

template <std::size_t Dof>
class UvmsSingleArm {
public:
  using Manipulator = ManipulatorBase<Dof>;
  using UvmsJacobian = Matrix<6, 6 + Dof>;

  explicit UvmsSingleArm(std::string vehicle_name,
                         std::string manipulator_name,
                         const HomogeneousMatrix& t_mb_to_v = HomogeneousMatrix::identity())
      : vehicle_(std::move(vehicle_name)),
        manipulator_(std::move(manipulator_name)),
        t_manipulator_base_to_vehicle_(t_mb_to_v) {}

  VehicleBase& vehicle() { return vehicle_; }
  const VehicleBase& vehicle() const { return vehicle_; }

  Manipulator& manipulator() { return manipulator_; }
  const Manipulator& manipulator() const { return manipulator_; }

  void setManipulatorBaseToVehicleTransform(const HomogeneousMatrix& t_mb_to_v) {
    t_manipulator_base_to_vehicle_ = t_mb_to_v;
  }

  const HomogeneousMatrix& manipulatorBaseToVehicleTransform() const {
    return t_manipulator_base_to_vehicle_;
  }

  HomogeneousMatrix directKinematics(
      const std::array<sfc::Real, Dof>& q) const {
    return vehicleToInertial() * t_manipulator_base_to_vehicle_ *
           manipulator_.forwardKinematics(q);
  }

  UvmsJacobian jacobian(const std::array<sfc::Real, Dof>& q) const {
    
  }

private:
  VehicleBase vehicle_;
  Manipulator manipulator_;
  HomogeneousMatrix t_manipulator_base_to_vehicle_;

  HomogeneousMatrix vehicleToInertial() const {
    const RotationMatrix r = vehicle_.getRotationMatrixBaseToInertial();
    const Vector3 p{vehicle_.position()[0],
                    vehicle_.position()[1],
                    vehicle_.position()[2]};
    return HomogeneousMatrix::fromRotationTranslation(r, p);
  }

};

#endif  // SFC_UVMS_SINGLE_ARM_H_
