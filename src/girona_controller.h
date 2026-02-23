#ifndef SFC_GIRONA_CONTROLLER_H_
#define SFC_GIRONA_CONTROLLER_H_

#include <atomic>
#include <thread>
#include <vector>
#include <chrono>
#include <utility>
#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include "sensorless_force_control/AdmittanceConfig.h"


#include "girona_interface.h"
#include "functionlib/robot_model/uvms_single_arm.h"
#include "functionlib/utilts/print.h"
#include "functionlib/thrust_allocation/thruster_allocator_dls.h"
#include "functionlib/thrust_allocation/thruster_allocator_dls_offset.h"
#include "functionlib/task_priority_control/possible_tasks.h"
#include "functionlib/task_priority_control/task_priority_solver.h"
#include "functionlib/pid/pid.h"
#include "functionlib/observer/contact_wrench_observer.h"
#include "functionlib/observer/acceleration_observer.h"
#include "functionlib/robot_model/uvms_regressor.h"
#include "functionlib/filter/low_pass_filter.h"
#include "functionlib/contact_control/quaternion_admittance_controller.h"
#include "functionlib/stsm_control/super_twisting_smc.h"
#include "logger.h"

#include <cmath>
#include <yaml-cpp/yaml.h>

#define DEBUG_CONTROLLER
// #define DEBUG_OBSERVER
#define DEBUG_ROSTOPIC
// #define DEBUG_JOYSTICK
// #define DEBUG_ADMITTANCE

#define USE_LOG
#define USE_CONTROL
// choose one dynamic controller to define
#define STSMC
// #define PID

// choose one thrust allocation method to define
#define THRUST_DLS_OFFSET
// #define THRUST_DLS

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

inline void velcmd2configurations(const sfc::Vector6& vel_cmd,
                                  sfc::Vector3& x_ee_d,
                                  sfc::Quaternion& q_ee_d,
                                  sfc::Real dt) {
  if (!vel_cmd.isFinite() || !x_ee_d.isFinite() || !sfc::isFinite(dt) || dt <= sfc::zero()) {
    throw std::runtime_error("velcmd2configurations: invalid input");
  }
  if (!std::isfinite(q_ee_d.w) || !std::isfinite(q_ee_d.x) ||
      !std::isfinite(q_ee_d.y) || !std::isfinite(q_ee_d.z)) {
    throw std::runtime_error("velcmd2configurations: invalid quaternion");
  }

  // Integrate position.
  x_ee_d(0) += vel_cmd(0) * dt;
  x_ee_d(1) += vel_cmd(1) * dt;
  x_ee_d(2) += vel_cmd(2) * dt;

  // Integrate orientation from angular velocity [rx, ry, rz].
  const sfc::Vector3 omega{vel_cmd(3), vel_cmd(4), vel_cmd(5)};
  const sfc::Real omega_norm = sfc::vectorNorm(omega);

  sfc::Quaternion dq{};
  if (omega_norm > static_cast<sfc::Real>(1e-6)) {
    const sfc::Real half_theta = static_cast<sfc::Real>(0.5) * omega_norm * dt;
    const sfc::Real sin_half = std::sin(half_theta);
    const sfc::Real cos_half = std::cos(half_theta);
    const sfc::Real scale = sin_half / omega_norm;
    dq.w = cos_half;
    dq.x = omega(0) * scale;
    dq.y = omega(1) * scale;
    dq.z = omega(2) * scale;
  } else {
    const sfc::Real scale = static_cast<sfc::Real>(0.5) * dt;
    dq.w = static_cast<sfc::Real>(1.0);
    dq.x = omega(0) * scale;
    dq.y = omega(1) * scale;
    dq.z = omega(2) * scale;
  }

  // post multiply means rotate within local frame, not inertia frame.
  q_ee_d = (q_ee_d * dq).normalized();
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
  void admittanceReconfigCb(sensorless_force_control::AdmittanceConfig& config, uint32_t level);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  GironaInterface interface_;
  UvmsType uvms_;
  #ifdef THRUST_DLS
    sfc::ThrusterAllocatorDls<6> allocator_;
  #endif
  #ifdef THRUST_DLS_OFFSET
    sfc::ThrusterAllocatorDlsOffset<6> allocator_;
  #endif
  sfc::Real thrust_offset_{static_cast<sfc::Real>(0.0)};
  #ifdef PID 
    sfc::PidController<6> pid_;
  #endif
  #ifdef STSMC
    sfc::SuperTwistingSmc stsmc_;
  #endif
  sfc::ContactWrenchObserver wrench_observer_;
  sfc::AccelerationObserver<3> linear_acc_observer_;
  sfc::Vector<28> dynamic_parameters_;
  sfc::Vector<4> wrenchsensor_parameters_;
  sfc::FirstOrderLowPassFilter<6> wrench_filter_;
  sfc::QuaternionAdmittanceController admitance_controller_;
  dynamic_reconfigure::Server<sensorless_force_control::AdmittanceConfig> admittance_server_;
  #ifdef USE_LOG
    sfc::Logger logger_{""};
    bool logger_open_{false};
  #endif
  ros::Subscriber joycmd_sub_;
  void joyCmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
  geometry_msgs::Twist joy_cmd_{};

  ros::Publisher gravity_pub_;
  ros::Publisher thrusts_pub_;
  ros::Publisher computed_control_wrench_pub_;
  ros::Publisher gravity_minus_tau_v_pub_;
  ros::Publisher tau_e_pub_;
  ros::Publisher h_e_inertiaframe_pub_;
  ros::Publisher h_e_bodyframe_pub_;
  ros::Publisher h_e_tipframe_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_d_;
  tf2_ros::TransformBroadcaster tf_broadcaster_r_;

  ros::Publisher sensor_feedback_pub_;
  ros::Publisher control_wrench_array_pub_;
  ros::Publisher force_array_pub_;
  ros::Publisher setpoints_array_pub_;
  ros::Publisher nu_d_array_pub_;
  ros::Publisher joint_velocities_array_pub_;
  ros::Publisher error_array_pub_;
  ros::Publisher sensor_calibrated_pub_;
  ros::Publisher sensor_calibrated_tiplink_pub_;

  std::mutex kin_config_mutex_;
  sfc::Real jointlimit_rho_{static_cast<sfc::Real>(0.2)};
  sfc::Real jointlimit_ds_{static_cast<sfc::Real>(0.2)};
  sfc::Real jointlimit_gain_{static_cast<sfc::Real>(0.1)};
  sfc::Vector3 ref_rpy_{0.0, 0.0, sfc::kPi2 / 2.0};
  sfc::Vector3 gain_rpy_{0.0, 1.0, 2.0};
  sfc::Vector6 gain_ee_{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  sfc::Vector6 nominal_config_{0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  sfc::Real allocator_damping_{static_cast<sfc::Real>(1e-4)};

  bool enable_thruster_command_{true};
  bool enable_arm_command_{true};
  bool enable_logging_{true};
  bool enable_jointlimits_task_{true};
  bool enable_sigma_rpy_task_{true};
  bool enable_ee_task_{true};
  bool enable_nominalconfiguration_task_{true};

  ros::AsyncSpinner spinner_;
  std::thread interface_thread_;
  std::thread control_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> control_running_{false};
  std::size_t n_thrusters_{6};
};

}  // namespace sfc

#endif  // SFC_GIRONA_CONTROLLER_H_
