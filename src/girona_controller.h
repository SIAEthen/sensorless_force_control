#ifndef SFC_GIRONA_CONTROLLER_H_
#define SFC_GIRONA_CONTROLLER_H_

#include <atomic>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include "functionlib/robot_model/uvms_single_arm.h"
#include "girona_interface.h"
#include "functionlib/utilts/print.h"
#include "functionlib/thrust_allocation/thruster_allocator_dls.h"
#include "functionlib/task_priority_control/possible_tasks.h"
#include "functionlib/task_priority_control/task_priority_solver.h"
#include "functionlib/pid/pid.h"
#include "functionlib/observer/contact_wrench_observer.h"
#include "functionlib/observer/acceleration_observer.h"
#include "functionlib/robot_model/uvms_regressor.h"

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

inline double setpoint2thrust(double s) {
  const double max_rpm = 1000.0;
  const double KT = 0.48;
  const double D = 0.18;
  const double rho = 1000.0;
  const double n = s * max_rpm / 60;
  return KT*rho*std::pow(D, 4) * n *std::abs(n);
}

inline sfc::Vector<6> convertForceToSetpoints(const sfc::Vector<6> force){
  sfc::Vector<6> setpoints{};
  for(int8_t i=0; i<6;i++){
    setpoints(i) = thrust2setpoint(force(i));
  }
  return setpoints;
}

inline sfc::Vector<6> convertSetpointsToThrusts(const sfc::Vector<6> setpoints){
  sfc::Vector<6> thrusts{};
  for(int8_t i=0; i<6;i++){
    thrusts(i) = setpoint2thrust(setpoints(i));
  }
  return thrusts;
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

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  GironaInterface interface_;
  UvmsType uvms_;
  sfc::ThrusterAllocatorDls<6> allocator_;
  sfc::PidController<6> pid_;
  sfc::ContactWrenchObserver wrench_observer_;
  sfc::AccelerationObserver<3> linear_acc_observer_;
  sfc::Vector<28> dynamic_parameters_;
  ros::Publisher gravity_pub_;
  ros::Publisher thrusts_pub_;
  ros::Publisher computed_control_wrench_pub_;
  ros::Publisher gravity_minus_tau_v_pub_;
  ros::Publisher tau_e_pub_;
  ros::Publisher h_e_inertiaframe_pub_;
  ros::Publisher sensor_feedback_pub_;
  ros::Publisher control_wrench_array_pub_;
  ros::Publisher force_array_pub_;
  ros::Publisher setpoints_array_pub_;
  ros::Publisher nu_d_array_pub_;
  ros::Publisher joint_velocities_array_pub_;
  ros::Publisher error_array_pub_;

  ros::AsyncSpinner spinner_;
  std::thread interface_thread_;
  std::thread control_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> control_running_{false};
  std::size_t n_thrusters_{6};
};

}  // namespace sfc

#endif  // SFC_GIRONA_CONTROLLER_H_
