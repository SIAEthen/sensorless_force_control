#ifndef SFC_UVMS_UVMS_CONTROLLER_H_
#define SFC_UVMS_UVMS_CONTROLLER_H_

#include <cstddef>

#include "config.h"
#include "dynamic_control/dynamic_controller.h"
#include "force_control/force_control.h"
#include "kinematic_control/pose_controller.h"
#include "observer/state_observer.h"
#include "thrust_allocation/thruster_allocator.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

template <std::size_t Thrusters>
class UvmsController {
public:
  void setPoseController(const PoseController& controller) { pose_controller_ = controller; }
  void setDynamicController(const DynamicController& controller) { dynamic_controller_ = controller; }
  void setForceController(const ForceControl& controller) { force_controller_ = controller; }
  void setObserver(const StateObserver& observer) { observer_ = observer; }
  void setAllocator(const ThrusterAllocator<Thrusters>& allocator) { allocator_ = allocator; }

  void setDesiredWrench(const Vector6& wrench) { desired_wrench_ = wrench; }
  void setUseForceControl(bool enable) { use_force_control_ = enable; }
  void setAllocationDamping(Real damping) { allocation_damping_ = damping; }

  Vector<Thrusters> update(const Vector6& desired_pose,
                           const Vector6& current_pose,
                           const Vector6& measured_wrench,
                           Real dt) {
    StateEstimate estimate = observer_.update(current_pose, dt);
    const Vector6 desired_twist = pose_controller_.update(desired_pose, estimate.position);
    Vector6 desired_wrench = dynamic_controller_.update(desired_twist, estimate.velocity, dt);

    if (use_force_control_) {
      const Vector6 wrench_correction =
          force_controller_.update(desired_wrench_, measured_wrench, dt);
      desired_wrench = desired_wrench + wrench_correction;
    }

    return allocator_.allocate(desired_wrench, allocation_damping_);
  }

private:
  PoseController pose_controller_{};
  DynamicController dynamic_controller_{};
  ForceControl force_controller_{};
  StateObserver observer_{};
  ThrusterAllocator<Thrusters> allocator_{};
  Vector6 desired_wrench_{};
  bool use_force_control_{false};
  Real allocation_damping_{static_cast<Real>(1e-3)};
};

}  // namespace sfc

#endif  // SFC_UVMS_UVMS_CONTROLLER_H_
