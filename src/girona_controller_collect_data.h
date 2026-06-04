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
#include "functionlib/thrust_allocation/thruster_allocator_dls_offset.h"
#include "functionlib/task_priority_control/possible_tasks.h"
#include "functionlib/task_priority_control/task_priority_solver.h"
#include "functionlib/pid/pid.h"
#include "thruster_error_define.h"

#include <mutex>
#include <cmath>
#include "exp_thruster_model.h"
#include "exp_utilts.h"
#include "src/thruster_allocator_qpoases.h"

#include <dynamic_reconfigure/server.h>
#include <sensorless_force_control/ControllerCollectDataConfig.h>

#define QP_ALLOCATOR
// #define LS_ALLOCATOR
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
  void reconfigCb(sensorless_force_control::ControllerCollectDataConfig& config, uint32_t level);
  void logFrame(double stamp_sec,
                const sfc::Vector3& vehicle_xyz,
                const sfc::Vector3& vehicle_rpy,
                const sfc::Vector6& current_joint,
                const sfc::Vector6& thrusts,
                const sfc::Vector6& setpoints,
                const sfc::Vector<12>& zeta,
                const sfc::Vector3& xyz_err,
                const sfc::Vector3& rpy_err,
                const sfc::Vector6& nominal_err,
                const sfc::Vector6& wrench_sensor);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  GironaInterface interface_;
  UvmsType uvms_;

  ThrusterAllocatorQpoases allocator_;
  sfc::Real thrust_offset_{static_cast<sfc::Real>(0.0)};

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

  // Dynamic reconfigure
  std::mutex                                                                    cfg_mutex_;
  dynamic_reconfigure::Server<sensorless_force_control::ControllerCollectDataConfig> reconfig_server_;

  // Tunable parameters (protected by cfg_mutex_)
  sfc::Vector3 xyz_ref_{0.0, 0.0, 2.0};
  sfc::Real    desired_pitch_{0.0};
  sfc::Real    ref_yaw_{0.5};
  sfc::Vector3 gain_rpy_{0.0, 1.0, 2.0};
  sfc::Real    gain_nc_{0.1};
  sfc::Real    dq_lim_{0.3};
  sfc::Real    allocator_desired_input_{0.55};
  sfc::Vector6 pid_kp_{50.0, 50.0, 50.0,  0.0, 50.0, 20.0};
  sfc::Vector6 pid_ki_{ 1.0,  1.0,  5.0,  0.0, 10.0,  1.0};
  sfc::Vector6 pid_kd_{ 8.0,  8.0,  8.0,  0.0,  4.0,  4.0};
  bool         enable_thruster_command_{false};
  bool         enable_arm_command_{false};
};

}  // namespace sfc

#endif  // SFC_GIRONA_CONTROLLER_H_
