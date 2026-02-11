// Common task builders for task-priority control.
#ifndef SFC_TASK_PRIORITY_CONTROL_POSSIBLE_TASKS_H_
#define SFC_TASK_PRIORITY_CONTROL_POSSIBLE_TASKS_H_

#include <cstddef>
#include <stdexcept>

#include "config.h"
#include "functionlib/robot_model/vehicle_base.h"
#include "functionlib/robot_model/uvms_single_arm.h"
#include "utilts/error_representation.h"
#include "utilts/matrix.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {
template<std::size_t taskDim>
inline void buildTaskVelocity(const Vector<taskDim>& ref_vel,
                              const Vector<taskDim>& dsigma,
                              const Vector<taskDim>& gain,
                              Vector<taskDim>& des_vel){
  for(std::size_t i=0;i<taskDim;i++){ 
    des_vel(i) = ref_vel(i) + gain(i)*dsigma(i);}
}




// Roll/pitch stabilization task (2xDof).
template <std::size_t ArmDof,
          typename ManipulatorT = ManipulatorFromYAML<ArmDof>,
          typename VehicleT = VehicleBase>
inline void buildRollPitchTask(const UvmsSingleArm<ArmDof,ManipulatorT,VehicleT>& uvms,
                               const Vector<2>& rp_ref,
                               Matrix<2, 6 + ArmDof>& J_out,
                               Vector<2>& dsigma) {
  const Vector3 rpy = uvms.vehicleRpy();
  if (!rpy.isFinite() || !rp_ref.isFinite()) {
    throw std::runtime_error("buildRollPitchTask: non-finite value");
  }

  dsigma(0) = rp_ref(0) - rpy(0);
  dsigma(1) = rp_ref(1) - rpy(1);

  J_out = Matrix<2, 6 + ArmDof>{};
  const Matrix3 j_inv = uvms.vehicle().J_ko_inv();
  const Matrix<2,3> map{1,0,0,0,1,0};
  Matrix<2,3> j_rp_temp = map * j_inv;
  for (std::size_t row = 0; row < 2; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      J_out(row, col + 3) = j_rp_temp(row, col);
    }
  }
}

// Roll/pitch/yaw stabilization task (3xDof).
template <std::size_t ArmDof,
          typename ManipulatorT = ManipulatorFromYAML<ArmDof>,
          typename VehicleT = VehicleBase>
inline void buildRollPitchYawTask(const UvmsSingleArm<ArmDof,ManipulatorT,VehicleT>& uvms,
                                  const Vector3& rpy_ref,
                                  Matrix<3, 6 + ArmDof>& J_out,
                                  Vector<3>& dsigma) {
  const Vector3 rpy = uvms.vehicleRpy();
  if (!rpy.isFinite() || !rpy_ref.isFinite()) {
    throw std::runtime_error("buildRollPitchYawTask: non-finite value");
  }

  dsigma(0) = (rpy_ref(0) - rpy(0));
  dsigma(1) = (rpy_ref(1) - rpy(1));
  dsigma(2) = (rpy_ref(2) - rpy(2));

  J_out = Matrix<3, 6 + ArmDof>{};
  const Matrix3 j_inv = uvms.vehicle().J_ko_inv();
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      J_out(row, col + 3) = j_inv(row, col);
    }
  }
}


// End-effector pose task (6x(6+n)) using UVMS model.
template <std::size_t ArmDof,
          typename ManipulatorT = ManipulatorFromYAML<ArmDof>,
          typename VehicleT = VehicleBase>
inline void buildEeTask(const UvmsSingleArm<ArmDof,ManipulatorT,VehicleT>& uvms,
                        const Vector3& pos_ref,
                        const Quaternion& q_ref,
                        Matrix<6, 6 + ArmDof>& J_out,
                        Vector<6>& dsigma) {

  Vector3 pos_now = uvms.endEffectorPositionNed();
  Quaternion q_now = uvms.endEffectorQuaternionNed();
  
  if (!pos_ref.isFinite() || !pos_now.isFinite()) {
    throw std::runtime_error("buildEeTask: non-finite value");
  }

  const Matrix<6, 6 + ArmDof> J_current = uvms.jacobian();
  if (!J_current.isFinite()) {
    throw std::runtime_error("buildEeTask: non-finite jacobian");
  }

  J_out = J_current;
  for (std::size_t row = 0; row < 6; ++row) {
    J_out(row, 3) = static_cast<Real>(0.0);
  }

  const Quaternion q_err = quatError(q_ref, q_now);
  dsigma(0) = pos_ref(0) - pos_now(0);
  dsigma(1) = pos_ref(1) - pos_now(1);
  dsigma(2) = pos_ref(2) - pos_now(2);
  dsigma(3) = q_err.x;
  dsigma(4) = q_err.y;
  dsigma(5) = q_err.z;
}

// Nominal joint configuration task (n x (6+n)).
template <std::size_t ArmDof,
          typename ManipulatorT = ManipulatorFromYAML<ArmDof>,
          typename VehicleT = VehicleBase>
inline void buildNominalConfigTask(const UvmsSingleArm<ArmDof,ManipulatorT,VehicleT>& uvms,
                                   const Vector<ArmDof>& nominal_config,
                                   Matrix<ArmDof, 6 + ArmDof>& J_out,
                                   Vector<ArmDof>& dsigma) {
  const Vector<ArmDof> current_config = uvms.manipulatorPosition();
  if (!current_config.isFinite()) {
    throw std::runtime_error("buildNominalConfigTask: non-finite value");
  }

  J_out = Matrix<ArmDof, 6 + ArmDof>{};
  for (std::size_t i = 0; i < ArmDof; ++i) {
    J_out(i, 6 + i) = static_cast<Real>(1.0);
    dsigma(i) = nominal_config(i)-current_config(i);
  }
}

// Nominal joint configuration task (n x (6+n)).
template <std::size_t ArmDof,
          typename ManipulatorT = ManipulatorFromYAML<ArmDof>,
          typename VehicleT = VehicleBase>
inline void buildNominal3ConfigTask(const UvmsSingleArm<ArmDof,ManipulatorT,VehicleT>& uvms,
                                   const Vector<3>& nominal_config,
                                   Matrix<3, 6 + ArmDof>& J_out,
                                   Vector<3>& dsigma) {
  const Vector<ArmDof> current_config = uvms.manipulatorPosition();
  if (!current_config.isFinite()) {
    throw std::runtime_error("buildNominalConfigTask: non-finite value");
  }

  J_out = Matrix<3, 6 + ArmDof>{};
  for (std::size_t i = 0; i < ArmDof; ++i) {
    J_out(i, 6 + i) = static_cast<Real>(1.0);
    dsigma(i) = nominal_config(i)-current_config(i);
  }
}

// vehicle configuration task (6 x (6+n)).
template <std::size_t ArmDof,
          typename ManipulatorT = ManipulatorFromYAML<ArmDof>,
          typename VehicleT = VehicleBase>
inline void buildVehiclePositionTask(const UvmsSingleArm<ArmDof,ManipulatorT,VehicleT>& uvms,
                                   const Vector<3>& pos_ref,
                                   Matrix<3, 6 + ArmDof>& J_out,
                                   Vector<3>& dsigma) {
  const Vector<3> current_pos = uvms.vehiclePosition();
  if (!current_pos.isFinite()) {
    throw std::runtime_error("buildNominalConfigTask: non-finite value");
  }

  RotationMatrix r_body_inertial = uvms.vehicleRotationMatrixBaseToInertial();
  Matrix<3,3+ArmDof> zero_matrix{};
  J_out = sfc::hstack<3,3,3+ArmDof>(r_body_inertial.m,zero_matrix);
  for (std::size_t i = 0; i < ArmDof; ++i) {
    dsigma(i) = (pos_ref(i)-current_pos(i));
  }
}

}  // namespace sfc

#endif  // SFC_TASK_PRIORITY_CONTROL_POSSIBLE_TASKS_H_
