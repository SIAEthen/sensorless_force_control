#ifndef SFC_KINEMATIC_CONTROL_POSE_CONTROLLER_H_
#define SFC_KINEMATIC_CONTROL_POSE_CONTROLLER_H_

#include <cstddef>
#include <stdexcept>

#include "config.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

class PoseController {
public:
  void setGains(const Vector6& kp) { kp_ = kp; }

  Vector6 update(const Vector6& desired_pose,
                 const Vector6& current_pose) const {
    if (!desired_pose.isFinite() || !current_pose.isFinite()) {
      throw std::runtime_error("PoseController::update: non-finite value");
    }
    Vector6 error{};
    for (std::size_t i = 0; i < 6; ++i) {
      error(i) = desired_pose(i) - current_pose(i);
    }
    Vector6 desired_twist{};
    for (std::size_t i = 0; i < 6; ++i) {
      desired_twist(i) = kp_(i) * error(i);
    }
    return desired_twist;
  }

private:
  Vector6 kp_{};
};

}  // namespace sfc

#endif  // SFC_KINEMATIC_CONTROL_POSE_CONTROLLER_H_
