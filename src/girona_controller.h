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
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include "sensorless_force_control/AdmittanceConfig.h"
#include "sensorless_force_control/SetAllocatorMu.h"


#include "girona_interface.h"
#include "functionlib/robot_model/uvms_single_arm.h"
#include "functionlib/utilts/print.h"
#include "functionlib/utilts/robot_math.h"
#include "src/thruster_allocator_qpoases.h"
#include "functionlib/task_priority_control/possible_tasks.h"
#include "functionlib/task_priority_control/task_priority_solver.h"
#include "functionlib/pid/pid.h"
#include "functionlib/observer/contact_wrench_observer.h"
#include "functionlib/observer/acceleration_observer.h"
#include "functionlib/robot_model/uvms_regressor.h"
#include "functionlib/filter/low_pass_filter.h"
#include "functionlib/contact_control/quaternion_admittance_controller.h"
#include "functionlib/contact_control/variable_admittance.h"
#include "functionlib/contact_control/girona_iros_force.h"

#include "functionlib/stsm_control/super_twisting_smc.h"
#include "logger.h"
#include "thruster_error_define.h"


#include <cmath>
#include <yaml-cpp/yaml.h>

#include "controller_define.h"

#ifdef USE_HQP
  #include "functionlib/hqp/hqp_cascaded_solver.h"
  #include "functionlib/hqp/hqp_tasks.h"
  #include <iomanip>
  #include <cassert>
  #include <cmath>
#endif

#include "exp_thruster_model.h"
#include "exp_utilts.h"



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
  bool setAllocatorMuCb(sensorless_force_control::SetAllocatorMu::Request& req,
                        sensorless_force_control::SetAllocatorMu::Response& res);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  GironaInterface interface_;
  UvmsType uvms_;
  ThrusterAllocatorQpoases allocator_;
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
  sfc::Vector6 dynamic_offset_;
  sfc::Vector<11> wrenchsensor_parameters_;
  sfc::FirstOrderLowPassFilter<6> wrench_filter_;
  
  #ifdef USE_ADMITTANCE
    #ifdef USE_VARIABLE_ADMITTANCE
      sfc::VariableAdmittanceController variable_admitance_controller_;
    #elif defined(USE_IROS_FORCE)
      sfc::IrosForceAdmittanceController iros_force_controller_;
    #else
      sfc::QuaternionAdmittanceController admitance_controller_;
    #endif
  #endif


  dynamic_reconfigure::Server<sensorless_force_control::AdmittanceConfig> admittance_server_;
  #ifdef USE_LOG
    sfc::Logger logger_{""};
    bool logger_open_{false};
  #endif
  ros::Subscriber joycmd_sub_;
  ros::Subscriber ee_pose_cmd_sub_;
  ros::Subscriber desired_wrench_sub_;
  ros::ServiceServer allocator_mu_srv_;
  void joyCmdCallback(const geometry_msgs::Twist::ConstPtr& msg);
  geometry_msgs::Twist joy_cmd_{};
  void eePoseCmdCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  geometry_msgs::PoseStamped ee_pose_cmd_{};
  bool ee_pose_cmd_received_{false};
  void desiredWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

  ros::Publisher gravity_pub_;
  ros::Publisher thrusts_pub_;
  ros::Publisher computed_control_wrench_pub_;
  ros::Publisher gravity_minus_tau_v_pub_;
  ros::Publisher tau_e_pub_;
  ros::Publisher tau_e_sensed_pub_;
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
  ros::Publisher ee_velocity_array_pub_;
  ros::Publisher x_ee_array_pub_;
  ros::Publisher q_ee_array_pub_;
  ros::Publisher x_ee_d_array_pub_;
  ros::Publisher x_ee_r_array_pub_;
  ros::Publisher q_ee_d_array_pub_;
  ros::Publisher q_ee_r_array_pub_;
  ros::Publisher v_ee_d_array_pub_;
  ros::Publisher v_ee_r_array_pub_;
  ros::Publisher a_ee_r_array_pub_;
  ros::Publisher joint_velocities_array_pub_;
  ros::Publisher error_array_pub_;
  ros::Publisher sensor_calibrated_pub_;
  ros::Publisher sensor_calibrated_tiplink_pub_;
  ros::Publisher k_stiff_pub_;
  ros::Publisher manipulability_pub_;
#ifdef USE_HQP
  ros::Publisher beta_ee_pub_;
  ros::Publisher beta_rpy_pub_;
  ros::Publisher beta_nominal_pub_;
  ros::Publisher beta_man_pub_;
  ros::Publisher beta_sc_pub_;
  ros::Publisher kin_ctrl_timing_pub_;
#endif

  std::mutex kin_config_mutex_;
  std::mutex allocator_config_mutex_;
  sfc::Real jointlimit_rho_{static_cast<sfc::Real>(0.2)};
  sfc::Real jointlimit_ds_{static_cast<sfc::Real>(0.2)};
  sfc::Real jointlimit_gain_{static_cast<sfc::Real>(0.1)};
  sfc::Real desired_depth_{static_cast<sfc::Real>(2.0)}; ///< desired vehicle depth [m]
  sfc::Real v1_lim_{static_cast<sfc::Real>(0.2)};        ///< vehicle linear  velocity limit [m/s]
  sfc::Real v2_lim_{static_cast<sfc::Real>(0.2)};        ///< vehicle angular velocity limit [rad/s]
  sfc::Real dq_lim_{static_cast<sfc::Real>(0.1)};        ///< manipulator joint velocity limit [rad/s]
  sfc::Vector3 ref_rpy_{0.0, 0.0, sfc::kPi2 / 2.0};
  sfc::Vector3 gain_rpy_{0.0, 1.0, 2.0};
  sfc::Vector6 gain_ee_{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  sfc::Vector6 admittance_deadzone_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  sfc::Vector6 nominal_config_{0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  sfc::Real allocator_damping_{static_cast<sfc::Real>(1e-4)};
  sfc::Real kw_jl_{1000.0};
  sfc::Real kw_sc_{1000.0};
  sfc::Real kw_man_{10.0};
  sfc::Real kw_rpy_{1.0};
  sfc::Real kw_ee_{1.0};
  sfc::Real kw_nc_{1.0};
  
  #ifdef QP_ALLOCATOR_055
    sfc::Vector6 allocator_desired_normalized_input_{0.55, 0.55, 0.55, 0.55, 0.0, 0.0};
  #endif
  #ifdef QP_ALLOCATOR_040
    sfc::Vector6 allocator_desired_normalized_input_{0.40, 0.40, 0.40, 0.40, 0.0, 0.0};
  #endif
  #ifdef QP_ALLOCATOR_030
    sfc::Vector6 allocator_desired_normalized_input_{0.30, 0.30, 0.30, 0.30, 0.0, 0.0};
  #endif
  #ifdef QP_ALLOCATOR_020
    sfc::Vector6 allocator_desired_normalized_input_{0.20, 0.20, 0.20, 0.20, 0.0, 0.0};
  #endif
  #ifdef QP_ALLOCATOR_010
    sfc::Vector6 allocator_desired_normalized_input_{0.10, 0.10, 0.10, 0.10, 0.0, 0.0};
  #endif
  #ifdef QP_ALLOCATOR_000
    sfc::Vector6 allocator_desired_normalized_input_{0.00, 0.00, 0.00, 0.00, 0.0, 0.0};
  #endif
  
  bool allocator_desired_input_pending_{false};

  bool enable_thruster_command_{true};
  bool enable_arm_command_{true};
  bool enable_logging_{true};
  bool enable_jointlimits_task_{true};
  bool enable_self_collision_task_{true};
  bool enable_manipulability_task_{true};
  bool enable_sigma_rpy_task_{true};
  bool enable_push_task_{false};
  bool enable_ee_task_{true};
  bool enable_nominalconfiguration_task_{true};
  bool enable_zero_velocity_task_{true};
  bool enable_vehicle_vision_task_{false};
  bool enable_admittance_{false};

  ros::AsyncSpinner spinner_;
  std::thread interface_thread_;
  std::thread control_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> control_running_{false};
  std::size_t n_thrusters_{6};
};

}  // namespace sfc

#endif  // SFC_GIRONA_CONTROLLER_H_
