#ifndef SFC_GIRONA_CONTROLLER_H_
#define SFC_GIRONA_CONTROLLER_H_

#include <atomic>
#include <thread>
#include <vector>

#include <ros/ros.h>

#include "functionlib/hardware/uvms_single_arm.h"
#include "girona_interface.h"

namespace sfc {

class GironaController {
 public:
  GironaController(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~GironaController();

  void start();
 void stop();

  GironaInterface& interface() { return interface_; }
  const GironaInterface& interface() const { return interface_; }

  sfc::UvmsSingleArm<GironaInterface::kArmDof>& uvms() { return uvms_; }
  const sfc::UvmsSingleArm<GironaInterface::kArmDof>& uvms() const { return uvms_; }

 private:
 void interfaceThread();
  void controlThread();
  void initUvms();

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  GironaInterface interface_;
  sfc::UvmsSingleArm<GironaInterface::kArmDof> uvms_;

  ros::AsyncSpinner spinner_;
  std::thread interface_thread_;
  std::thread control_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> control_running_{false};
  std::size_t n_thrusters_{6};
};

}  // namespace sfc

#endif  // SFC_GIRONA_CONTROLLER_H_
