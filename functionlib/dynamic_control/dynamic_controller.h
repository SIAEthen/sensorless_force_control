#ifndef SFC_DYNAMIC_CONTROL_DYNAMIC_CONTROLLER_H_
#define SFC_DYNAMIC_CONTROL_DYNAMIC_CONTROLLER_H_

#include <stdexcept>

#include "config.h"
#include "dynamic_control/pid.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

class DynamicController {
public:
  void setPid(const Pid<6>& pid) { pid_ = pid; }

  Vector6 update(const Vector6& desired_twist,
                 const Vector6& estimated_twist,
                 Real dt) {
    if (!desired_twist.isFinite() || !estimated_twist.isFinite() || !isFinite(dt)) {
      throw std::runtime_error("DynamicController::update: non-finite value");
    }
    if (dt <= zero()) {
      throw std::runtime_error("DynamicController::update: non-positive dt");
    }

    Vector6 error{};
    for (std::size_t i = 0; i < 6; ++i) {
      error(i) = desired_twist(i) - estimated_twist(i);
    }
    return pid_.update(error, dt);
  }

private:
  Pid<6> pid_{};
};

}  // namespace sfc

#endif  // SFC_DYNAMIC_CONTROL_DYNAMIC_CONTROLLER_H_
