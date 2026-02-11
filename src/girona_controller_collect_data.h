#ifndef SFC_GIRONA_CONTROLLER_H_
#define SFC_GIRONA_CONTROLLER_H_

#include <atomic>
#include <thread>
#include <vector>
#include <cstdlib> // rand()
#include "logger.h"
#include <ros/ros.h>

#include "functionlib/robot_model/uvms_single_arm.h"
#include "girona_interface.h"
#include "functionlib/utilts/print.h"
#include "functionlib/thrust_allocation/thruster_allocator_dls.h"
#include "functionlib/task_priority_control/possible_tasks.h"
#include "functionlib/task_priority_control/task_priority_solver.h"
#include "functionlib/pid/pid.h"

#include <cmath>

inline double thrust2setpoint(double f) {
  const double max_rpm = 1000.0;
  const double KT = 0.48;
  const double D = 0.18;
  const double rho = 1000.0;

  const double n2 = f / (KT * rho * std::pow(D, 4));
  const double n = std::sqrt(std::abs(n2));
  double s = 60.0 * n / max_rpm;
  if(f>0.0) return s;
  else return -s;
}
inline sfc::Vector<6> convertForceToSetpoints(const sfc::Vector<6> force){
  sfc::Vector<6> setpoints{};
  for(int8_t i=0; i<6;i++){
    setpoints(i) = thrust2setpoint(force(i));
  }
  return setpoints;
}
inline sfc::Vector<6> getRandomJointPosition(){
  sfc::Vector<6> joint_position{};
  sfc::Vector<6> min{-sfc::kPi4,-sfc::kPi4,-sfc::kPi4,-sfc::kPi4,-sfc::kPi2,-sfc::kPi4};
  sfc::Vector<6> max{sfc::kPi4,sfc::kPi4,sfc::kPi4,sfc::kPi4,0,sfc::kPi4};
  for(int8_t i=0; i<6;i++){
    double r = static_cast<double>(rand()%100)/100.0;
    joint_position(i) = r*(max(i)-min(i)) + min(i);
  }
  return joint_position;
}

namespace sfc {

class GironaController {
  using UvmsType = sfc::UvmsSingleArm<GironaInterface::kArmDof,ManipulatorFromYAML<GironaInterface::kArmDof>>;
 public:
  GironaController(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~GironaController();

  void start();
  void stop();

  GironaInterface& interface() { return interface_; }
  const GironaInterface& interface() const { return interface_; }

  UvmsType& uvms() { return uvms_; }
  const UvmsType& uvms() const { return uvms_; }

 private:
  void interfaceThread();
  void controlThread();
 void initializeController();
  void logFrame(double stamp_sec,
                const sfc::Vector3& vehicle_xyz,
                const sfc::Vector3& vehicle_rpy,
                const sfc::Vector6& current_joint,
                const sfc::Vector6& setpoints,
                const sfc::Vector<12>& zeta,
                const sfc::Vector3& xyz_err,
                const sfc::Vector3& rpy_err,
                const sfc::Vector6& nominal_err);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  GironaInterface interface_;
  UvmsType uvms_;
  sfc::ThrusterAllocatorDls<6> allocator_;
  sfc::PidController<6> pid_;
  sfc::PController<6> p_;

  ros::AsyncSpinner spinner_;
  std::thread interface_thread_;
  std::thread control_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> control_running_{false};
  std::size_t n_thrusters_{6};
  sfc::Logger logger_{""};
  bool logger_open_{false};
};

}  // namespace sfc

#endif  // SFC_GIRONA_CONTROLLER_H_
