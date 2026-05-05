#include "girona_controller.h"
#include <boost/bind.hpp>

namespace {

inline void publishWrench(ros::Publisher& pub,
                          const sfc::Vector6& v,
                          const ros::Time& stamp,
                          const char* frame_id = "girona1000/base_link") {
  geometry_msgs::WrenchStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.wrench.force.x = v(0);
  msg.wrench.force.y = v(1);
  msg.wrench.force.z = v(2);
  msg.wrench.torque.x = v(3);
  msg.wrench.torque.y = v(4);
  msg.wrench.torque.z = v(5);
  pub.publish(msg);
}

inline void publishArray6(ros::Publisher& pub, const sfc::Vector6& v) {
  std_msgs::Float64MultiArray msg;
  msg.data.resize(6);
  for (std::size_t i = 0; i < 6; ++i) {
    msg.data[i] = v(i);
  }
  pub.publish(msg);
}

inline void publishArray3(ros::Publisher& pub, const sfc::Vector3& v) {
  std_msgs::Float64MultiArray msg;
  msg.data.resize(3);
  for (std::size_t i = 0; i < 3; ++i) {
    msg.data[i] = v(i);
  }
  pub.publish(msg);
}

inline void publishQuaternion(ros::Publisher& pub, const sfc::Quaternion& q) {
  std_msgs::Float64MultiArray msg;
  msg.data.resize(4);
  msg.data[0] = q.w;
  msg.data[1] = q.x;
  msg.data[2] = q.y;
  msg.data[3] = q.z;
  pub.publish(msg);
}

inline sfc::Vector6 applyContinuousDeadzone(const sfc::Vector6& value,
                                            const sfc::Vector6& deadzone) {
  sfc::Vector6 out{};
  for (std::size_t i = 0; i < 6; ++i) {
    const sfc::Real dz = std::abs(deadzone(i));
    const sfc::Real v = value(i);
    const sfc::Real av = std::abs(v);
    if (av <= dz) {
      out(i) = static_cast<sfc::Real>(0.0);
    } else {
      out(i) = (v > static_cast<sfc::Real>(0.0)) ? (av - dz) : -(av - dz);
    }
  }
  return out;
}

}  // namespace

namespace sfc {

bool GironaController::setAllocatorMuCb(sensorless_force_control::SetAllocatorMu::Request& req,
                                        sensorless_force_control::SetAllocatorMu::Response& res) {
  std::lock_guard<std::mutex> lock(allocator_config_mutex_);
  allocator_desired_normalized_input_(0) = static_cast<sfc::Real>(req.mu_1);
  allocator_desired_normalized_input_(1) = static_cast<sfc::Real>(req.mu_2);
  allocator_desired_normalized_input_(2) = static_cast<sfc::Real>(req.mu_3);
  allocator_desired_normalized_input_(3) = static_cast<sfc::Real>(req.mu_4);
  allocator_desired_input_pending_ = true;

  res.success = true;
  res.message = "Updated allocator desired normalized input [0:4)";
  ROS_INFO("Updated allocator mu_d to [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
           allocator_desired_normalized_input_(0),
           allocator_desired_normalized_input_(1),
           allocator_desired_normalized_input_(2),
           allocator_desired_normalized_input_(3),
           allocator_desired_normalized_input_(4),
           allocator_desired_normalized_input_(5));
  return true;
}

void GironaController::admittanceReconfigCb(sensorless_force_control::AdmittanceConfig& config,
                                            uint32_t /*level*/) {
  sfc::Vector6 mass{};
  sfc::Vector6 damping{};
  sfc::Vector6 stiffness{};
  mass(0) = static_cast<sfc::Real>(config.mass_1);
  mass(1) = static_cast<sfc::Real>(config.mass_2);
  mass(2) = static_cast<sfc::Real>(config.mass_3);
  mass(3) = static_cast<sfc::Real>(config.mass_4);
  mass(4) = static_cast<sfc::Real>(config.mass_5);
  mass(5) = static_cast<sfc::Real>(config.mass_6);

  damping(0) = static_cast<sfc::Real>(config.damping_1);
  damping(1) = static_cast<sfc::Real>(config.damping_2);
  damping(2) = static_cast<sfc::Real>(config.damping_3);
  damping(3) = static_cast<sfc::Real>(config.damping_4);
  damping(4) = static_cast<sfc::Real>(config.damping_5);
  damping(5) = static_cast<sfc::Real>(config.damping_6);

  stiffness(0) = static_cast<sfc::Real>(config.stiffness_1);
  stiffness(1) = static_cast<sfc::Real>(config.stiffness_2);
  stiffness(2) = static_cast<sfc::Real>(config.stiffness_3);
  stiffness(3) = static_cast<sfc::Real>(config.stiffness_4);
  stiffness(4) = static_cast<sfc::Real>(config.stiffness_5);
  stiffness(5) = static_cast<sfc::Real>(config.stiffness_6);
  #ifdef USE_ADMITTANCE
    #ifdef USE_VARIABLE_ADMITTANCE
      variable_admitance_controller_.setGains(mass, damping, stiffness);       
    #elif defined(USE_IROS_FORCE)
    Vector6 stiff = iros_force_controller_.getStiffness();
      iros_force_controller_.setGains(mass, damping, stiff);                                       
    #else
      admitance_controller_.setGains(mass, damping, stiffness);
    #endif
  #endif
  

  wrench_observer_.setMass(static_cast<sfc::Real>(config.wrench_observer_mass));
  wrench_observer_.setInertia(sfc::Vector3{
      static_cast<sfc::Real>(config.wrench_observer_inertia_1),
      static_cast<sfc::Real>(config.wrench_observer_inertia_2),
      static_cast<sfc::Real>(config.wrench_observer_inertia_3)});
  wrench_observer_.setGains(
      sfc::Vector3{
          static_cast<sfc::Real>(config.wrench_observer_k1_1),
          static_cast<sfc::Real>(config.wrench_observer_k1_2),
          static_cast<sfc::Real>(config.wrench_observer_k1_3)},
      sfc::Vector3{
          static_cast<sfc::Real>(config.wrench_observer_k2_1),
          static_cast<sfc::Real>(config.wrench_observer_k2_2),
          static_cast<sfc::Real>(config.wrench_observer_k2_3)});

  linear_acc_observer_.setCutoffHz(static_cast<sfc::Real>(config.linear_acc_cutoff_hz));
  wrench_filter_.setCutoffHz(static_cast<sfc::Real>(config.wrench_filter_cutoff_hz));

#ifdef STSMC
  sfc::Vector6 stsmc_k1{};
  sfc::Vector6 stsmc_k2{};
  sfc::Vector6 stsmc_lambda{};
  sfc::Vector6 stsmc_eps{};

  stsmc_k1(0) = static_cast<sfc::Real>(config.stsmc_k1_1);
  stsmc_k1(1) = static_cast<sfc::Real>(config.stsmc_k1_2);
  stsmc_k1(2) = static_cast<sfc::Real>(config.stsmc_k1_3);
  stsmc_k1(3) = static_cast<sfc::Real>(config.stsmc_k1_4);
  stsmc_k1(4) = static_cast<sfc::Real>(config.stsmc_k1_5);
  stsmc_k1(5) = static_cast<sfc::Real>(config.stsmc_k1_6);

  stsmc_k2(0) = static_cast<sfc::Real>(config.stsmc_k2_1);
  stsmc_k2(1) = static_cast<sfc::Real>(config.stsmc_k2_2);
  stsmc_k2(2) = static_cast<sfc::Real>(config.stsmc_k2_3);
  stsmc_k2(3) = static_cast<sfc::Real>(config.stsmc_k2_4);
  stsmc_k2(4) = static_cast<sfc::Real>(config.stsmc_k2_5);
  stsmc_k2(5) = static_cast<sfc::Real>(config.stsmc_k2_6);

  stsmc_lambda(0) = static_cast<sfc::Real>(config.stsmc_lambda_1);
  stsmc_lambda(1) = static_cast<sfc::Real>(config.stsmc_lambda_2);
  stsmc_lambda(2) = static_cast<sfc::Real>(config.stsmc_lambda_3);
  stsmc_lambda(3) = static_cast<sfc::Real>(config.stsmc_lambda_4);
  stsmc_lambda(4) = static_cast<sfc::Real>(config.stsmc_lambda_5);
  stsmc_lambda(5) = static_cast<sfc::Real>(config.stsmc_lambda_6);

  stsmc_eps(0) = static_cast<sfc::Real>(config.stsmc_eps_1);
  stsmc_eps(1) = static_cast<sfc::Real>(config.stsmc_eps_2);
  stsmc_eps(2) = static_cast<sfc::Real>(config.stsmc_eps_3);
  stsmc_eps(3) = static_cast<sfc::Real>(config.stsmc_eps_4);
  stsmc_eps(4) = static_cast<sfc::Real>(config.stsmc_eps_5);
  stsmc_eps(5) = static_cast<sfc::Real>(config.stsmc_eps_6);

  stsmc_.setGains(stsmc_k1, stsmc_k2, stsmc_lambda);
  stsmc_.setBoundaryLayer(stsmc_eps);
#endif

  {
    std::lock_guard<std::mutex> lock(kin_config_mutex_);
    jointlimit_rho_ = static_cast<sfc::Real>(config.jointlimit_rho);
    jointlimit_ds_ = static_cast<sfc::Real>(config.jointlimit_ds);
    jointlimit_gain_ = static_cast<sfc::Real>(config.jointlimit_gain);

    ref_rpy_(0) = static_cast<sfc::Real>(config.ref_rpy_1);
    ref_rpy_(1) = static_cast<sfc::Real>(config.ref_rpy_2);
    ref_rpy_(2) = static_cast<sfc::Real>(config.ref_rpy_3);

    gain_rpy_(0) = static_cast<sfc::Real>(config.gain_rpy_1);
    gain_rpy_(1) = static_cast<sfc::Real>(config.gain_rpy_2);
    gain_rpy_(2) = static_cast<sfc::Real>(config.gain_rpy_3);

    gain_ee_(0) = static_cast<sfc::Real>(config.gain_ee_1);
    gain_ee_(1) = static_cast<sfc::Real>(config.gain_ee_2);
    gain_ee_(2) = static_cast<sfc::Real>(config.gain_ee_3);
    gain_ee_(3) = static_cast<sfc::Real>(config.gain_ee_4);
    gain_ee_(4) = static_cast<sfc::Real>(config.gain_ee_5);
    gain_ee_(5) = static_cast<sfc::Real>(config.gain_ee_6);

    admittance_deadzone_(0) = static_cast<sfc::Real>(config.admittance_deadzone_1);
    admittance_deadzone_(1) = static_cast<sfc::Real>(config.admittance_deadzone_2);
    admittance_deadzone_(2) = static_cast<sfc::Real>(config.admittance_deadzone_3);
    admittance_deadzone_(3) = static_cast<sfc::Real>(config.admittance_deadzone_4);
    admittance_deadzone_(4) = static_cast<sfc::Real>(config.admittance_deadzone_5);
    admittance_deadzone_(5) = static_cast<sfc::Real>(config.admittance_deadzone_6);

    nominal_config_(0) = static_cast<sfc::Real>(config.nominal_config_1);
    nominal_config_(1) = static_cast<sfc::Real>(config.nominal_config_2);
    nominal_config_(2) = static_cast<sfc::Real>(config.nominal_config_3);
    nominal_config_(3) = static_cast<sfc::Real>(config.nominal_config_4);
    nominal_config_(4) = static_cast<sfc::Real>(config.nominal_config_5);
    nominal_config_(5) = static_cast<sfc::Real>(config.nominal_config_6);

    allocator_damping_ = static_cast<sfc::Real>(config.allocator_damping);
    #ifdef THRUST_DLS_OFFSET
      thrust_offset_ = static_cast<sfc::Real>(config.allocator_offset);
      const Vector6 offset = Vector6{thrust_offset_,thrust_offset_,thrust_offset_,thrust_offset_,0.0,0.0};
      allocator_.setHorizontalOffset(offset);
    #endif
    enable_thruster_command_ = config.enable_thruster_command;
    enable_arm_command_ = config.enable_arm_command;
    enable_logging_ = config.enable_logging;
    enable_jointlimits_task_ = config.enable_jointlimits_task;
    enable_sigma_rpy_task_ = config.enable_sigma_rpy_task;
    enable_ee_task_ = config.enable_ee_task;
    enable_nominalconfiguration_task_ = config.enable_nominalconfiguration_task;
    enable_admittance_ = config.enable_admittance;
  }
}

GironaController::GironaController(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh)),
      pnh_(std::move(pnh)),
      interface_(nh_, pnh_),
      uvms_(),
      spinner_(1) {
  n_thrusters_ = static_cast<std::size_t>(pnh_.param<int>("n_thrusters", 6));
  initializeController();
}

GironaController::~GironaController() {
  stop();
}

void GironaController::start() {
  if (running_.exchange(true)) {
    return;
  }
  interface_thread_ = std::thread(&GironaController::interfaceThread, this);
  if (!control_running_.exchange(true)) {
    control_thread_ = std::thread(&GironaController::controlThread, this);
  }
}

void GironaController::stop() {
  if (!running_.exchange(false)) {
    return;
  }
  control_running_.store(false);
  spinner_.stop();
  if (interface_thread_.joinable()) {
    interface_thread_.join();
  }
  if (control_thread_.joinable()) {
    control_thread_.join();
  }
    if (logger_open_) {
    logger_.close();
    logger_open_ = false;
  }
  
}

void GironaController::interfaceThread() {
  spinner_.start();
  while (running_.load() && ros::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void GironaController::controlThread() {
    ros::Duration(1.0).sleep();
    ros::Rate rate(10.0);
    sfc::Vector6 setpoints{};
    sfc::Vector6 joint_velocity_desired{};
    ros::Time last = ros::Time::now();

    // variable for admittance control
    #ifdef S1
      sfc::Vector3 x_ee_d{0.0,3.6,2.0}; // for scenario push scenario 1
    #endif
    #ifdef S2
      sfc::Vector3 x_ee_d{0.3,3.6,2.0}; // for scenario push scenario 2
    #endif
    #ifdef FREE_FLOATING
      sfc::Vector3 x_ee_d{0,0,2.0};      // for free floating moving
    #endif
    #ifdef OBSERVER_TEST
      sfc::Vector3 x_ee_d{0.0,3.6,2.0};      // for observer test
    #endif
    // sfc::Quaternion q_ee_d = sfc::Quaternion::fromRPY(-sfc::kPi2,-sfc::kPi2,0.0);
    sfc::Quaternion q_ee_d = sfc::Quaternion{0,0,-0.707,-0.707}; // point the panel
    
    #ifdef OBSERVER_TEST
      // q_ee_d = sfc::Quaternion{0.0,0.0,0.0,1.0};      // for observer test
    #endif
    // test the accuracy, touch the sheep surface.
    // sfc::Vector3 x_ee_d{0.165781, -4.08583, 3.0};
    // sfc::Quaternion q_ee_d = sfc::Quaternion{-0.903388,0.0184309, -0.00873894, -0.428338};

    sfc::Vector6 v_ee_d{};

    sfc::Vector3 x_ee_r = x_ee_d;
    sfc::Quaternion q_ee_r = q_ee_d;
    sfc::Vector6 v_ee_r{};
    sfc::Vector6 a_ee_r{};



    #ifdef USE_ADMITTANCE
      #ifdef USE_VARIABLE_ADMITTANCE
        variable_admitance_controller_.reset(x_ee_d,q_ee_d);
      #elif defined(USE_IROS_FORCE)
        iros_force_controller_.reset(x_ee_d,q_ee_d);
      #else
        admitance_controller_.reset(x_ee_d,q_ee_d);
      #endif
    #endif
    #ifdef USE_HQP
      hqp::HQPCascadedSolver solver(12);
    #endif
    

    while (control_running_.load() && ros::ok()) {
        // get state from interface
        uvms_.setVehicleState(interface_.vehicleState());
        uvms_.setManipulatorState(interface_.manipulatorState());
        const sfc::Vector6 sensor_feedback = interface_.wrench();
        // prepare dt for control
        ros::Time now = ros::Time::now();
        double dt = (now - last).toSec();
        last = now;

        // update ee cmds
        v_ee_d = Vector6{};
        if (ee_pose_cmd_received_) {
          const sfc::Vector3 x_new{
              static_cast<sfc::Real>(ee_pose_cmd_.pose.position.x),
              static_cast<sfc::Real>(ee_pose_cmd_.pose.position.y),
              static_cast<sfc::Real>(ee_pose_cmd_.pose.position.z)};
          const sfc::Quaternion q_new = sfc::Quaternion{
              static_cast<sfc::Real>(ee_pose_cmd_.pose.orientation.w),
              static_cast<sfc::Real>(ee_pose_cmd_.pose.orientation.x),
              static_cast<sfc::Real>(ee_pose_cmd_.pose.orientation.y),
              static_cast<sfc::Real>(ee_pose_cmd_.pose.orientation.z)}.normalized();
          // linear velocity by finite difference
          v_ee_d(0) = (x_new(0) - x_ee_d(0)) / dt;
          v_ee_d(1) = (x_new(1) - x_ee_d(1)) / dt;
          v_ee_d(2) = (x_new(2) - x_ee_d(2)) / dt;
          // angular velocity from quaternion finite difference: omega ≈ 2*vec(dq)/dt
          const sfc::Quaternion dq = q_new * q_ee_d.conjugate();
          const sfc::Real sign = (dq.w >= sfc::zero()) ? sfc::one() : -sfc::one();
          v_ee_d(3) = sign * static_cast<sfc::Real>(2.0) * dq.x / dt;
          v_ee_d(4) = sign * static_cast<sfc::Real>(2.0) * dq.y / dt;
          v_ee_d(5) = sign * static_cast<sfc::Real>(2.0) * dq.z / dt;
          x_ee_d = x_new;
          q_ee_d = q_new;
        } else {
          // joystick fallback
          const Vector6 velcmd{joy_cmd_.linear.x,joy_cmd_.linear.y,joy_cmd_.linear.z,
                               joy_cmd_.angular.x,joy_cmd_.angular.y,joy_cmd_.angular.z};
          velcmd2configurations(velcmd,x_ee_d,q_ee_d,dt);
          v_ee_d = velcmd;
        }
        #ifdef DEBUG_JOYSTICK
            sfc::print(x_ee_d,std::cout,"x_ee_d");
            sfc::print(q_ee_d,std::cout,"q_ee_d");
        #endif
        geometry_msgs::TransformStamped t1;
        t1.header.stamp = ros::Time::now();
        t1.header.frame_id = "world_ned";   // 你的世界系
        t1.child_frame_id = "ee_d";         // 你想看的目标系

        t1.transform.translation.x = x_ee_d(0);
        t1.transform.translation.y = x_ee_d(1);
        t1.transform.translation.z = x_ee_d(2);

        // 注意：你的 Quaternion 是 w,x,y,z
        t1.transform.rotation.w = q_ee_d.w;
        t1.transform.rotation.x = q_ee_d.x;
        t1.transform.rotation.y = q_ee_d.y;
        t1.transform.rotation.z = q_ee_d.z;

        tf_broadcaster_d_.sendTransform(t1);

        geometry_msgs::TransformStamped t2;
        t2.header.stamp = ros::Time::now();
        t2.header.frame_id = "world_ned";   // 你的世界系
        t2.child_frame_id = "ee_r";         // 你想看的目标系

        t2.transform.translation.x = x_ee_r(0);
        t2.transform.translation.y = x_ee_r(1);
        t2.transform.translation.z = x_ee_r(2);

        // 注意：你的 Quaternion 是 w,x,y,z
        t2.transform.rotation.w = q_ee_d.w;
        t2.transform.rotation.x = q_ee_d.x;
        t2.transform.rotation.y = q_ee_d.y;
        t2.transform.rotation.z = q_ee_d.z;

        tf_broadcaster_r_.sendTransform(t2);

        
        
        
        // kinematics
        constexpr std::size_t kSysDof = 12;
        sfc::Matrix<kSysDof, kSysDof> N = sfc::identity<kSysDof>();
        sfc::Vector<kSysDof> zeta{};
        const sfc::Real damping = static_cast<sfc::Real>(1e-5);
        sfc::Real jointlimit_rho = static_cast<sfc::Real>(0.2);
        sfc::Real jointlimit_ds = static_cast<sfc::Real>(0.2);
        sfc::Real jointlimit_gain = static_cast<sfc::Real>(0.1);
        sfc::Vector3 ref_rpy{};
        sfc::Vector3 gain_rpy{};
        sfc::Vector6 gain_ee{};
        sfc::Vector6 admittance_deadzone{};
        sfc::Vector6 nominal_config{};
        sfc::Real allocator_damping = static_cast<sfc::Real>(1e-4);
        bool enable_thruster_command = true;
        bool enable_arm_command = true;
        bool enable_logging = true;
        bool enable_jointlimits_task = true;
        bool enable_sigma_rpy_task = true;
        bool enable_ee_task = true;
        bool enable_nominalconfiguration_task = true;
        bool enable_admittance = false;
        {
          std::lock_guard<std::mutex> lock(kin_config_mutex_);
          jointlimit_rho = jointlimit_rho_;
          jointlimit_ds = jointlimit_ds_;
          jointlimit_gain = jointlimit_gain_;
          ref_rpy = ref_rpy_;
          gain_rpy = gain_rpy_;
          gain_ee = gain_ee_;
          admittance_deadzone = admittance_deadzone_;
          nominal_config = nominal_config_;
          allocator_damping = allocator_damping_;
          enable_thruster_command = enable_thruster_command_;
          enable_arm_command = enable_arm_command_;
          enable_logging = enable_logging_;
          enable_jointlimits_task = enable_jointlimits_task_;
          enable_sigma_rpy_task = enable_sigma_rpy_task_;
          enable_ee_task = enable_ee_task_;
          enable_nominalconfiguration_task = enable_nominalconfiguration_task_;
          enable_admittance = enable_admittance_;
        }


        #ifdef USE_TPC
        // const sfc::Vector<6> q_min{-3*sfc::kPi4,-sfc::kPi2,-sfc::kPi2,-3*sfc::kPi4, -sfc::kPi, 0.0};
        // const sfc::Vector<6> q_max{ 3*sfc::kPi4, sfc::kPi2, sfc::kPi2, 3*sfc::kPi4,  0.0,      3.5*sfc::kPi4};
        const sfc::Vector<6> q_min{-1.5*sfc::kPi4,-sfc::kPi2,-sfc::kPi2,-3*sfc::kPi4, -sfc::kPi, 0.0};
        const sfc::Vector<6> q_max{ 1.5*sfc::kPi4, sfc::kPi2, sfc::kPi2, 3*sfc::kPi4,  0.0,      3.5*sfc::kPi4};
        sfc::Matrix<6, kSysDof> J_jointlimits{};
        sfc::Vector6 sigma_jointlimits{};
        if (enable_jointlimits_task) {
          sfc::buildJointLimitDamperTask<6>(uvms_,q_min,q_max,jointlimit_rho,jointlimit_ds,jointlimit_gain,J_jointlimits,sigma_jointlimits);
          zeta = sfc::taskPrioritySolveStep<kSysDof, 6>(sigma_jointlimits, J_jointlimits, N, zeta, damping);
        }
        #ifdef DEBUG_CONTROLLER
          sfc::print(sigma_jointlimits,std::cout,"sigma_jointlimits");
          sfc::print(zeta,std::cout,"zeta");
        #endif
        
        // // // Task 1: roll/pitch stabilization
        // const sfc::Vector<2> ref_rp{0.0,0.0};
        // const sfc::Vector<2> gain_rp{0.0,1.0};
        // sfc::Matrix<2, kSysDof> J_rp{};
        // sfc::Vector<2> sigma_rp{};
        // sfc::Vector<2> task_vel_rp{};
        // sfc::buildRollPitchTask(uvms_, ref_rp, J_rp, sigma_rp);
        // sfc::buildTaskVelocity<2>(sfc::Vector<2>{},sigma_rp,gain_rp,task_vel_rp);
        // zeta = sfc::taskPrioritySolveStep<kSysDof, 2>(task_vel_rp, J_rp, N, zeta, damping);
        // #ifdef DEBUG_CONTROLLER
        //   sfc::print(sigma_rp,std::cout,"sigma_rp");
        //   sfc::print(zeta,std::cout,"zeta");
        // #endif

        // // Task 1.5: roll/pitch/yaw stabilization
        sfc::Matrix<3, kSysDof> J_rpy{};
        sfc::Vector<3> sigma_rpy{};
        sfc::Vector<3> task_vel_rpy{};
        if (enable_sigma_rpy_task) {
          sfc::buildRollPitchYawTask(uvms_, ref_rpy, J_rpy, sigma_rpy);
          sfc::buildTaskVelocity<3>(sfc::Vector3{},sigma_rpy,gain_rpy,task_vel_rpy);
          zeta = sfc::taskPrioritySolveStep<kSysDof, 3>(task_vel_rpy, J_rpy, N, zeta, damping);
        }
        #ifdef DEBUG_CONTROLLER
          sfc::print(sigma_rpy,std::cout,"sigma_rpy");
          sfc::print(zeta,std::cout,"zeta");
        #endif

        // Task 2: end-effector task (set your references)
        // const sfc::Vector3 ref_ee_pos{0,0,2.5};
        // const sfc::Quaternion ref_ee_quat = uvms_.endEffectorQuaternionNed();
        // const sfc::Quaternion ref_ee_quat = sfc::Quaternion::fromRPY(-sfc::kPi2,-sfc::kPi2,0.0);
        sfc::Matrix<6, kSysDof> J_ee{};
        sfc::Vector<6> sigma_ee{};
        sfc::Vector<6> task_vel_ee{};
        // build ee task with desired quaternion or referenced quaternion
        if (enable_ee_task) {
          sfc::buildEeTask(uvms_, x_ee_r, q_ee_d, J_ee, sigma_ee);
          sfc::buildTaskVelocity<6>(v_ee_r,sigma_ee,gain_ee,task_vel_ee);
          // sfc::buildTaskVelocity<6>(Vector6{},sigma_ee,gain_ee,task_vel_ee);
          zeta = sfc::taskPrioritySolveStep<kSysDof, 6>(sigma_ee, J_ee, N, zeta, damping);
        }
        #ifdef DEBUG_CONTROLLER
          sfc::print(sigma_ee,std::cout,"sigma_ee");
          sfc::print(zeta,std::cout,"zeta");
        #endif

        // Task 3: nominal joint configuration
        const sfc::Vector<6> gain_nominal_config{10,10,10,10,10,10};
        sfc::Matrix<6, kSysDof> J_nominal{};
        sfc::Vector<6> sigma_nominal{};
        sfc::Vector<6> task_vel_nominal{};
        if (enable_nominalconfiguration_task) {
          sfc::buildNominalConfigTask(uvms_, nominal_config, J_nominal, sigma_nominal);
          sfc::buildTaskVelocity<6>(sfc::Vector6{},sigma_nominal,gain_nominal_config,task_vel_nominal);
          zeta = sfc::taskPrioritySolveStep<kSysDof, 6>(sigma_nominal, J_nominal, N, zeta, damping);
        }
        
        #ifdef DEBUG_CONTROLLER
          sfc::print(sigma_nominal,std::cout,"sigma_nominal");
          sfc::print(zeta,std::cout,"velocity");
        #endif

        const sfc::Vector<kSysDof> zeta_abs_limit{
            0.1, 0.1, 0.1, 0.1, 0.1, 0.1,  // vehicle velocity limits
            0.30, 0.30, 0.30, 0.30, 0.30, 0.30   // joint velocity limits
        };
        const sfc::Vector<kSysDof> zeta_sat = sfc::clampSymmetric<kSysDof>(zeta, zeta_abs_limit);
        // The above is for TPC
        #endif //USE_TPC

        #ifdef USE_HQP
          solver.clearTasks();
          //task: Joint Limits
          const sfc::Vector<6> q_min{-1.5*sfc::kPi4,-sfc::kPi2,-sfc::kPi2,-3*sfc::kPi4, -sfc::kPi, 0.0};
          const sfc::Vector<6> q_max{ 1.5*sfc::kPi4, sfc::kPi2, sfc::kPi2, 3*sfc::kPi4,  0.0,      3.5*sfc::kPi4};
          const sfc::Vector<6> dq_min{-0.2,-0.2,-0.2,-0.2,-0.2,-0.2};
          const sfc::Vector<6> dq_max{0.2,0.2,0.2,0.2,0.2,0.2};
          const sfc::Vector<6> nu_min{-0.1,-0.1,-0.1,-0.1,-0.1,-0.1};
          const sfc::Vector<6> nu_max{0.1,0.1,0.1,0.1,0.1,0.1};

          sfc::Vector<12> Kq_jl = sfc::Vector<12>{1,1,1,1,1,1, 1,1,1,1,1,1};
          sfc::Vector<12> Kw_jl = sfc::Vector<12>{1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};
          if (enable_jointlimits_task) { 
            solver.addTask(hqp::makeSystemConstraintsTask(uvms_, nu_min,nu_max,
                                                          q_min, q_max,dq_min,dq_max, Kq_jl, Kw_jl));
          }

          // sfc::Vector<2> gain_rp = sfc::Vector<2>{1,1};
          // sfc::Vector<2> rp_ref = sfc::Vector<2>{0,0};
          // sfc::Vector<12> Kq_rp = sfc::Vector<12>{1,1,1,5,5,1, 1,1,1,1,1,1};
          // sfc::Vector<2> Kw_rp = sfc::Vector<2>{1,1};
          // if (enable_sigma_rpy_task) { 
          //   solver.addTask(hqp::makeRollPitchTask(uvms_, rp_ref, gain_rp, Kq_rp, Kw_rp));}

          // Task: RPY
          // sfc::Vector<3> gain_rpy = sfc::Vector<3>{1,1,1};
          sfc::Vector<12> Kq_rpy = sfc::Vector<12>{1,1,1,5,5,1, 1,1,1,1,1,1};
          sfc::Vector<3> Kw_rpy = sfc::Vector<3>{1,1,1};
          if (enable_sigma_rpy_task) { 
            solver.addTask(hqp::makeRollPitchYawTask(uvms_, ref_rpy, gain_rpy, Kq_rpy, Kw_rpy));
          }

          // Task: EE configuration
          sfc::Vector<12> Kq_ee = sfc::Vector<12>{1,1,1,1,1,1, 1,1,1,1,1,1};
          sfc::Vector<6> Kw_ee = sfc::Vector<6>{1,1,1,1,1,1};
          if (enable_ee_task) {
            solver.addTask(hqp::makeEeTask(uvms_, x_ee_r, q_ee_d, gain_ee, Kq_ee,Kw_ee));
          }
          
          // Task: nominal joint configuration
          const sfc::Vector<6> gain_nominal_config{10,10,10,10,10,10};
          sfc::Vector<12> Kq_nc = sfc::Vector<12>{1,1,1,1,1,1, 1,1,1,1,1,1};
          sfc::Vector<6> Kw_nc = sfc::Vector<6>{1,1,1,1,1,1};
          if (enable_nominalconfiguration_task) {
            solver.addTask(hqp::makeNominalConfigTask(uvms_, nominal_config, gain_nominal_config, 
              Kq_nc,Kw_nc));
          }

          auto res = solver.solve();
          Eigen::VectorXd zeta_eigen = res.qdot;
          Vector<12> zeta_sat = hqp::toVector<12>(zeta_eigen);

          #ifdef DEBUG_CONTROLLER
          sfc::print(zeta_sat,std::cout,"hqp velocity");
          #endif

        #endif


        sfc::Vector6 nu_d{zeta_sat(0),zeta_sat(1),zeta_sat(2),zeta_sat(3),zeta_sat(4),zeta_sat(5)};
        sfc::Vector6 nu_error = nu_d - uvms_.vehicleVelocity();
        const Vector6 gravity = sfc::regressor_girona1000(uvms_.vehicleRpy(),
                                                        uvms_.manipulator(),
                                                        uvms_.manipulatorBaseToVehicleTransform())
                                * dynamic_parameters_ + dynamic_offset_;
        {
          std::lock_guard<std::mutex> lock(allocator_config_mutex_);
          if (allocator_desired_input_pending_) {
            allocator_.setDesiredNormalizedInput(allocator_desired_normalized_input_);
            allocator_desired_input_pending_ = false;
          }
        }

        sfc::Vector6 control_wrench{};
        sfc::Vector6 thruster_force{};
        if (enable_thruster_command) {
          #ifdef PID
            control_wrench = pid_.update(nu_error,dt);
          #endif
          #ifdef STSMC
            // control_wrench = stsmc_.update(nu_error,Vector6{},gravity,dt);
            control_wrench = stsmc_.update(nu_error,gravity,dt);
            control_wrench(3) = 0.0;
            // control_wrench = stsmc_.update(nu_error,dt);
          #endif
          // thruster_force = allocator_.allocate(control_wrench,allocator_damping);
          thruster_force = allocator_.allocate(control_wrench);
          setpoints = convertThrustsToSetpoints(thruster_force);
        } else {
          stsmc_.reset();
        }
        joint_velocity_desired(0) = zeta_sat(6);
        joint_velocity_desired(1) = zeta_sat(7);
        joint_velocity_desired(2) = zeta_sat(8);
        joint_velocity_desired(3) = zeta_sat(9);
        joint_velocity_desired(4) = zeta_sat(10);
        joint_velocity_desired(5) = zeta_sat(11);

        #ifdef DEBUG_CONTROLLER
          sfc::print(control_wrench,std::cout,"control wrench");
          sfc::print(setpoints,std::cout,"setpoints");
        #endif


        // sensor feedback
        const sfc::HomogeneousMatrix t_ft_tip = sfc::HomogeneousMatrix::fromRotationTranslation
                                        (sfc::RotationMatrix::fromRPY(-0.000, -0.000, 2.880), sfc::Vector3{0.087, 0.000, -0.245});
        const auto t_tip_inertia = uvms_.forwardKinematics();
        const auto t_ft_inertia = t_tip_inertia * t_ft_tip;
        const sfc::Vector6 sensor_feedback_filtered = wrench_filter_.update(sensor_feedback,dt);
        const sfc::Vector6 sensor_feedback_calibrated = sensor_feedback_filtered - sfc::Regressor_stupid(t_ft_inertia.rotation()) * wrenchsensor_parameters_;
        const sfc::Matrix<6,6> U_ft_tip = sfc::U_mat(t_ft_tip.rotation(),t_ft_tip.translation());
        const sfc::Vector6 sensor_feedback_calibrated_ontiplink = U_ft_tip * sensor_feedback_calibrated;
        
        const auto t_tip_body = uvms_.forwardKinematicsBodyFrame();
        const sfc::Matrix<6,6> U_tip_body = sfc::U_mat(t_tip_body.rotation(),t_tip_body.translation());
        const sfc::Vector<6> tau_e_sensed = U_tip_body * sensor_feedback_calibrated_ontiplink * (-1);
        const auto t_inertia_tip = t_tip_inertia.inverse();
        const sfc::Vector<6> h_e_ned_sensed = sfc::U_mat(t_inertia_tip.rotation(),Vector3{0,0,0})
                                            * sensor_feedback_calibrated_ontiplink;


        #ifdef DEBUG_OBSERVER
          sfc::print(sensor_feedback,std::cout,"sensor_feedback");
          sfc::print(sensor_feedback_filtered,std::cout,"sensor_feedback_filtered");
          sfc::print(sensor_feedback_calibrated,std::cout,"sensor_feedback_calibrated");
          sfc::print(sensor_feedback_calibrated_ontiplink,std::cout,"sensor_feedback_calibrated_ontiplink");
          sfc::print(tau_e_sensed,std::cout,"tau_e_sensed");
        #endif

        // observer 
        const Vector6 nu = uvms_.vehicleVelocity();
        const Vector3 nu_1{nu(0),nu(1),nu(2)};
        const Vector3 nu_2{nu(3),nu(4),nu(5)};
        const Vector6 v_ee = uvms_.endEffectorVelocityNed(nu, uvms_.manipulatorVelocity());
        const Vector3 acc = linear_acc_observer_.update(nu_1,dt);
                             
        const Vector6 thrusts = convertSetpointsToThrusts(setpoints);
        const Vector6 computed_control_wrench = allocator_.computeWrench(thrusts);
        const Vector6 gravity_minus_tau_v = gravity - computed_control_wrench;
        const Vector6 tau_e = wrench_observer_.update(gravity_minus_tau_v,acc,nu_2,dt);
        // the vector h_e_inertiaframe is force and torque on tip link expressed in ned frame.
        const Vector6 h_e_inertiaframe = sfc::pseudoInverseDls(uvms_.jacobian_first6collumns().transpose(),0.00001)  
                                          * tau_e * static_cast<sfc::Real>(-1.0);

        const Matrix<6,6> U_inertia_body_zero_translation = sfc::UMat(sfc::RotationMatrix::fromRPY(uvms_.vehicleRpy()).transpose(),Vector3{});
        const Vector6 h_e_bodyframe = U_inertia_body_zero_translation * h_e_inertiaframe;
        const Matrix<6,6> U_inertia_tip_zero_translation = sfc::UMat(t_tip_inertia.rotation().transpose(),Vector3{});
        const Vector6 h_e_tipframe = U_inertia_tip_zero_translation * h_e_inertiaframe;



        #ifdef DEBUG_OBSERVER
          sfc::print(gravity,std::cout,"gravity");
          sfc::print(thrusts,std::cout,"thrusts");
          sfc::print(computed_control_wrench,std::cout,"computed_control_wrench");
          sfc::print(gravity_minus_tau_v,std::cout,"gravity_minus_tau_v");
          sfc::print(tau_e,std::cout,"tau_e");
          sfc::print(h_e_inertiaframe,std::cout,"h_e_inertiaframe");
          sfc::print(h_e_bodyframe,std::cout,"h_e_bodyframe");
          sfc::print(h_e_tipframe,std::cout,"h_e_tipframe");
        #endif
        

        #ifdef USE_ADMITTANCE
          // admittance controller
          const Vector6 contact_force_torque = enable_admittance
              ? applyContinuousDeadzone(h_e_inertiaframe, admittance_deadzone)
              : Vector6{};
          #ifdef USE_VARIABLE_ADMITTANCE
            sfc::VariableAdmittanceController::Output out;
              // out=variable_admitance_controller_.update(x_ee_d,q_ee_d,v_ee_d,contact_force_torque,dt);
              out = enable_admittance
                ? variable_admitance_controller_.update(x_ee_d,q_ee_d,v_ee_d,contact_force_torque,dt)
                : sfc::VariableAdmittanceController::Output {};
              const Vector6 K_stiff = variable_admitance_controller_.getStiffness();
              if(!enable_admittance){
                variable_admitance_controller_.reset(x_ee_d,q_ee_d);
              }
          #elif defined(USE_IROS_FORCE)
            sfc::IrosForceAdmittanceController::Output out;
            // set desired force
            out=iros_force_controller_.update(x_ee_d,q_ee_d,v_ee_d,contact_force_torque,dt);
            Vector6 iros_force_stiffness = iros_force_controller_.getStiffness();
          #else
            sfc::QuaternionAdmittanceController::Output out;
            out=admitance_controller_.update(x_ee_d,q_ee_d,v_ee_d,contact_force_torque,dt);
          #endif // USE_VARIABLE_ADMITTANCE

        x_ee_r = enable_admittance ? out.pos_r : x_ee_d;
        q_ee_r = enable_admittance ? out.q_r : q_ee_d;
        v_ee_r = enable_admittance ? out.nu_r : v_ee_d;
        a_ee_r = enable_admittance ? out.acc_r : Vector6{};

        #else
          const Vector6 contact_force_torque = Vector6{};
          x_ee_r = x_ee_d;
          q_ee_r = q_ee_d;
          v_ee_r = v_ee_d;
          a_ee_r = Vector6{};
        #endif
        
        
        

        #ifdef DEBUG_ADMITTANCE
          sfc::print(x_ee_d,std::cout,"x_ee_d");
          sfc::print(x_ee_r,std::cout,"x_ee_r");
          sfc::print(q_ee_d,std::cout,"q_ee_d");
          sfc::print(q_ee_r,std::cout,"q_ee_r");
          sfc::print(v_ee_d,std::cout,"v_ee_d");
          sfc::print(v_ee_r,std::cout,"v_ee_r");
          sfc::print(a_ee_r,std::cout,"a_ee_r");
          #ifdef USE_VARIABLE_ADMITTANCE
            sfc::print(K_stiff,std::cout,"Stiffness gains");
          #endif
          
        #endif
        



        #ifdef DEBUG_ROSTOPIC
          publishArray6(control_wrench_array_pub_, control_wrench);
          publishArray6(force_array_pub_, thruster_force);
          publishArray6(setpoints_array_pub_, setpoints);
          publishArray6(nu_d_array_pub_, nu_d);
          publishArray6(ee_velocity_array_pub_, v_ee);
          publishArray3(x_ee_array_pub_, uvms_.endEffectorPositionNed());
          publishQuaternion(q_ee_array_pub_, uvms_.endEffectorQuaternionNed());
          publishArray3(x_ee_d_array_pub_, x_ee_d);
          publishArray3(x_ee_r_array_pub_, x_ee_r);
          publishQuaternion(q_ee_d_array_pub_, q_ee_d);
          publishQuaternion(q_ee_r_array_pub_, q_ee_r);
          publishArray6(v_ee_d_array_pub_, v_ee_d);
          publishArray6(v_ee_r_array_pub_, v_ee_r);
          publishArray6(a_ee_r_array_pub_, a_ee_r);
          publishArray6(joint_velocities_array_pub_, joint_velocity_desired);
          publishArray6(error_array_pub_, nu_error);
          publishArray6(gravity_pub_, gravity);
          publishArray6(thrusts_pub_, thrusts);
          publishArray6(computed_control_wrench_pub_, computed_control_wrench);
          publishArray6(gravity_minus_tau_v_pub_, gravity_minus_tau_v);
          publishArray6(tau_e_pub_, tau_e);
          publishArray6(tau_e_sensed_pub_, tau_e_sensed);
          publishArray6(h_e_inertiaframe_pub_, h_e_inertiaframe);
          publishArray6(h_e_bodyframe_pub_, h_e_bodyframe);
          publishArray6(h_e_tipframe_pub_, h_e_tipframe);
          publishArray6(sensor_feedback_pub_, sensor_feedback);
          publishArray6(sensor_calibrated_pub_, sensor_feedback_calibrated);
          publishArray6(sensor_calibrated_tiplink_pub_, sensor_feedback_calibrated_ontiplink);
          #ifdef USE_VARIABLE_ADMITTANCE
            publishArray6(k_stiff_pub_, K_stiff);
          #endif
          #ifdef USE_IROS_FORCE
            publishArray6(k_stiff_pub_, iros_force_stiffness);
          #endif
          
          
        #endif

        #ifdef USE_CONTROL
          if (enable_thruster_command) {
            interface_.sendThrusterSetpoints(convertThrustsToNoisedSetpoints(thruster_force));
          }
          if (enable_arm_command) {
            interface_.sendJointVelocityCommand(joint_velocity_desired);
          }
        #endif

        #ifdef USE_LOG
          if (enable_logging && logger_open_) {
            logger_.beginFrame(now.toSec());

            // current state, velocity, ee position
            logger_.logVector("nu", nu);
            logger_.logVector("acc", acc);
            logger_.logVector("eta1", uvms_.vehiclePosition());
            logger_.logVector("eta2", uvms_.vehicleRpy());
            logger_.logVector("joint_position", uvms_.manipulatorPosition());
            logger_.logVector("joint_velocity", uvms_.manipulatorVelocity());
            logger_.logVector("x_ee", uvms_.endEffectorPositionNed());
            logger_.logVector("q_ee", uvms_.endEffectorQuaternionNed());
            logger_.logVector("v_ee", v_ee);

            // cmd raw
            logger_.logVector("x_ee_d", x_ee_d);
            logger_.logVector("q_ee_d", q_ee_d);
            logger_.logVector("v_ee_d", v_ee_d);
            // cmd filtered by admittance controller
            logger_.logVector("x_ee_r", x_ee_r);
            logger_.logVector("q_ee_r", q_ee_r);
            logger_.logVector("v_ee_r", v_ee_r);

            // kinematics state variable
            #ifdef USE_TPC
              logger_.logVector("sigma_jointlimits", sigma_jointlimits);
              logger_.logVector("sigma_rpy", sigma_rpy);
              logger_.logVector("sigma_ee", sigma_ee);
              logger_.logVector("sigma_nominal", sigma_nominal);
            #endif

            // kinematics outputs
            logger_.logVector("nu_d", nu_d);
            logger_.logVector("dq_d", joint_velocity_desired);

            // dynamic controller
            logger_.logVector("nu_error", nu_error);
            logger_.logVector("thruster_force", thruster_force);
            logger_.logVector("setpoints", setpoints);
            logger_.logVector("control_wrench", control_wrench);

            // FT sensor outputs
            logger_.logVector("sensor_feedback_filtered", sensor_feedback_filtered);
            logger_.logVector("sensor_feedback_calibrated", sensor_feedback_calibrated);
            logger_.logVector("sensor_feedback_calibrated_ontiplink", sensor_feedback_calibrated_ontiplink);
            logger_.logVector("h_e_ned_sensed", h_e_ned_sensed);
                        

            // he estimated by observer
            logger_.logVector("computed_control_wrench", computed_control_wrench);
            logger_.logVector("gravity_minus_tau_v", gravity_minus_tau_v);
            logger_.logVector("tau_e", tau_e);
            logger_.logVector("h_e_inertiaframe", h_e_inertiaframe);
            logger_.logVector("h_e_tipframe", h_e_tipframe);


            // stiffness for variable stiffness
            #ifdef USE_VARIABLE_ADMITTANCE
              logger_.logVector("Stiffness",K_stiff);
            #endif
            #ifdef USE_IROS_FORCE
              logger_.logVector("Stiffness",iros_force_stiffness);
            #endif
            

            logger_.endFrame();
            
          }
        #endif
        
        // fix rate god tool
        rate.sleep();
      }

}

void GironaController::initializeController() {
  joycmd_sub_ = nh_.subscribe<geometry_msgs::Twist>(
      "/girona1000xh/joystick/velocity_cmd", 10, &GironaController::joyCmdCallback, this);
  ee_pose_cmd_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/girona1000xh/ee_pose_cmd", 10, &GironaController::eePoseCmdCallback, this);
  desired_wrench_sub_ = nh_.subscribe<geometry_msgs::WrenchStamped>(
      "/girona1000xh/desired_wrench", 10, &GironaController::desiredWrenchCallback, this);
  allocator_mu_srv_ = nh_.advertiseService("/girona1000xh/set_allocator_mu_d",
                                           &GironaController::setAllocatorMuCb,
                                           this);

  #ifdef DEBUG_ROSTOPIC
    control_wrench_array_pub_ =
        nh_.advertise<std_msgs::Float64MultiArray>("debug/control_wrench", 10);
    force_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/force", 10);
    setpoints_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/setpoints", 10);
    nu_d_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/nu_d", 10);
    ee_velocity_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/ee_velocity", 10);
    x_ee_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/x_ee", 10);
    q_ee_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/q_ee", 10);
    x_ee_d_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/x_ee_d", 10);
    x_ee_r_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/x_ee_r", 10);
    q_ee_d_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/q_ee_d", 10);
    q_ee_r_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/q_ee_r", 10);
    v_ee_d_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/v_ee_d", 10);
    v_ee_r_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/v_ee_r", 10);
    a_ee_r_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/a_ee_r", 10);
    joint_velocities_array_pub_ =
        nh_.advertise<std_msgs::Float64MultiArray>("debug/joint_velocity_desired", 10);
    error_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/error", 10);

    gravity_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/gravity", 10);
    thrusts_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/thrusts", 10);
    computed_control_wrench_pub_ =
        nh_.advertise<std_msgs::Float64MultiArray>("debug/computed_control_wrench", 10);
    gravity_minus_tau_v_pub_ =
        nh_.advertise<std_msgs::Float64MultiArray>("debug/gravity_minus_tau_v", 10);
    tau_e_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/tau_e", 10);
    tau_e_sensed_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/tau_e_sensed", 10);
    h_e_inertiaframe_pub_ =
        nh_.advertise<std_msgs::Float64MultiArray>("debug/h_e_inertiaframe", 10);
    h_e_bodyframe_pub_ =
        nh_.advertise<std_msgs::Float64MultiArray>("debug/h_e_bodyframe", 10);
    h_e_tipframe_pub_ =
        nh_.advertise<std_msgs::Float64MultiArray>("debug/h_e_tipframe", 10);
    sensor_feedback_pub_ =
        nh_.advertise<std_msgs::Float64MultiArray>("debug/sensor_feedback", 10);
    sensor_calibrated_pub_ =
        nh_.advertise<std_msgs::Float64MultiArray>("debug/sensor_calibrated", 10);
    sensor_calibrated_tiplink_pub_ =
        nh_.advertise<std_msgs::Float64MultiArray>("debug/sensor_calibrated_tiplink", 10);
    k_stiff_pub_ =
        nh_.advertise<std_msgs::Float64MultiArray>("debug/k_stiff", 10);
  #endif
  // Placeholder for DH parameters and transforms; configure as needed by your arm.
  ROS_INFO("Initialize UVMS model from yaml file (obtained from URDF)");
  
  sfc::ManipulatorFromYAML<GironaInterface::kArmDof> manip("bravo");
  const std::string yaml_path =
      "/home/sia/girona_ws/src/sensorless_force_control/config/control/bravo_joints.yaml";
  ROS_INFO("Yaml file path is: %s", yaml_path.c_str());
  const std::array<std::string, GironaInterface::kArmDof> joint_names = {{
      "bravo/joint1",
      "bravo/joint2",
      "bravo/joint3",
      "bravo/joint4",
      "bravo/joint5",
      "bravo/joint6",
  }};
  manip.setParametersFromFile(yaml_path, joint_names);
  // get the params from ros
  sfc::HomogeneousMatrix t_tool_linkend = sfc::HomogeneousMatrix::fromRotationTranslation
                                        (sfc::RotationMatrix::fromRPY(0,0,-2.88), sfc::Vector3{0.084, 0.022, 0.372});
  manip.setToolTransformationFromT(t_tool_linkend);
  // read from rosrun tf tf_echo girona1000/base_link girona1000/bravo/base_link no, this is wrong. we should use the origin link
  // rosrun tf tf_echo girona1000/base_link girona1000/bravo/base_link
  // rostopic echo /girona1000/dynamics/odometry
  sfc::RotationMatrix r = sfc::RotationMatrix::fromRPY(3.142, 0.000, -0.175);
  // sfc::Vector3 t = sfc::Vector3{0.732, -0.138, 0.271}; //from bravo/base_link to girona1000/base_link
  sfc::Vector3 t = sfc::Vector3{0.732, -0.138, 0.485};    //from bravo/base_link to girona1000/origin
  sfc::HomogeneousMatrix t_0_b = sfc::HomogeneousMatrix::fromRotationTranslation(r, t);
  uvms_.setManipulatorBaseToVehicleTransform(t_0_b);
  uvms_.setManipulator(manip);
  
  sfc::HomogeneousMatrix t_ee_ned = uvms_.forwardKinematics();
  sfc::Vector3 rpy = sfc::rpyFromRotationMatrix(t_ee_ned.rotation());
  sfc::Vector3 xyz = t_ee_ned.translation();
  ROS_INFO("Parameters init success! Do the transformation test! Try call cmd at zero joint state!");
  ROS_INFO("rosrun tf tf_echo girona1000/base_link girona1000/bravo/cp_probe_tip_link");
  ROS_INFO("EE Position (meters) X %f, Y %f, Z %f.", xyz(0),xyz(1),xyz(2));
  ROS_INFO("EE RPY (radians) Rx %f, Ry %f, Rz %f.",  rpy(0),rpy(1),rpy(2));
  // sfc::print(sfc::rpyFromRotationMatrix(t_ee_ned.rotation()),std::cout,"zero state ee rpy");
  // sfc::print(t_ee_ned.translation(),std::cout,"zero state ee xyz");
  
  
  ROS_INFO("Now we initialize the TCM matrix from Yaml");
  const std::string tcm_yaml_path = 
          "/home/sia/girona_ws/src/sensorless_force_control/config/control/tcm_with_thruster_torque_with_installation_error.yaml";
  try {
    YAML::Node root = YAML::LoadFile(tcm_yaml_path);
    const YAML::Node tcm_node = root["tcm"];
    if (!tcm_node) {
      throw std::runtime_error("Missing 'tcm' section in tcm.yaml");
    }
    const std::size_t rows = tcm_node["rows"].as<std::size_t>();
    const std::size_t cols = tcm_node["cols"].as<std::size_t>();
    if (rows != 6 || cols != 6) {
      throw std::runtime_error("TCM must be 6x6");
    }
    const YAML::Node data = tcm_node["data"];
    if (!data || !data.IsSequence() || data.size() != 6) {
      throw std::runtime_error("TCM data must be a 6x6 sequence");
    }

    sfc::Matrix<6, 6> tcm{};
    for (std::size_t r = 0; r < 6; ++r) {
      const YAML::Node row = data[r];
      if (!row.IsSequence() || row.size() != 6) {
        throw std::runtime_error("Each TCM row must have 6 elements");
      }
      for (std::size_t c = 0; c < 6; ++c) {
        tcm(r, c) = static_cast<sfc::Real>(row[c].as<double>());
      }
    }
    allocator_.setAllocationMatrix(tcm);
    sfc::Vector6 max_force{100,100,100,100,100,100};
    sfc::Vector6 min_force{-100,-100,-100,-100,-100,-100};
    allocator_.setLimits(min_force,max_force);
    allocator_.setNormalizedInputScale(max_force);
    allocator_.setDesiredNormalizedInput(allocator_desired_normalized_input_);
    allocator_.setWrenchWeights(sfc::Vector6{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    allocator_.setWorkingPointWeights(sfc::Vector6{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
    allocator_.setSmoothnessWeights(sfc::Vector6{0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    sfc::print(tcm,std::cout,"TCM matrix");
    ROS_INFO("TCM matrix loaded and set.");
  } catch (const std::exception& ex) {
    ROS_ERROR("Failed to load TCM: %s", ex.what());
  }
  #ifdef PID 
    ROS_INFO("Init PID controller");
    sfc::Vector6 kp{50,50,50,20,30,20};
    sfc::Vector6 ki{1,1,5,1,5,1};
    sfc::Vector6 kd{8,8,8,2,4,4};
    sfc::Vector6 i_sat{50,50,100,10,50,10};
    pid_.setGains(kp,ki,kd);
    pid_.setIntegratorLimits(i_sat);
  #endif

  #ifdef STSMC
    ROS_INFO("Init stsmc controller");
    stsmc_.setGains(Vector6{10.0, 10.0, 10.0, 10.0, 10.0, 10.0},
                Vector6{2.0, 2.0, 2.0, 2.0, 2.0, 2.0},
                Vector6{5.0, 5.0, 5.0, 5.0, 5.0, 5.0});
    stsmc_.setBoundaryLayer(Vector6{2.0,2.0,2.0,2.0,2.0,2.0});
    stsmc_.reset();
  #endif

  ROS_INFO("Init Accleration observer");
  linear_acc_observer_.setCutoffHz(50);

  ROS_INFO("Init Contact wrench observer");
  wrench_observer_.setMass(153.121);
  wrench_observer_.setInertia(sfc::Vector3{94.660, 103.801, 147.540});
  wrench_observer_.setGains(sfc::Vector3{1.0,1.0,1.0},sfc::Vector3{1.0,1.0,1.0});


  ROS_INFO("Init dynamic parameters from Yaml");
  // const std::string dyn_yaml_path =
  //   "/home/sia/girona_ws/src/sensorless_force_control/config/control/model_optimized.yaml";
  const std::string dyn_yaml_path =
    "/home/sia/girona_ws/src/sensorless_force_control/config/control/model_optimized_noised_system.yaml";  
    
  try {
    YAML::Node root = YAML::LoadFile(dyn_yaml_path);
    const YAML::Node dyn_node = root["params"];
    if (!dyn_node || !dyn_node.IsSequence() || dyn_node.size() != 28) {
      throw std::runtime_error("vector28 must be a sequence of 28 elements");
    }
    for (std::size_t i = 0; i < 28; ++i) {
      dynamic_parameters_(i) = static_cast<sfc::Real>(dyn_node[i].as<double>());
    }
    sfc::print(dynamic_parameters_, std::cout, "dynamic_parameters");
    ROS_INFO("Dynamic parameters loaded.");
  } catch (const std::exception& ex) {
    ROS_ERROR("Failed to load dynamic parameters: %s", ex.what());
  }

  ROS_INFO("Init dynamic offset from Yaml");
  const std::string dyn_offset_yaml_path =
    "/home/sia/girona_ws/src/sensorless_force_control/config/control/model_optimized_offset.yaml";
  try {
    YAML::Node root = YAML::LoadFile(dyn_offset_yaml_path);
    const YAML::Node offset_node = root["params"];
    if (!offset_node || !offset_node.IsSequence() || offset_node.size() != 6) {
      throw std::runtime_error("offset vector must be a sequence of 6 elements");
    }
    for (std::size_t i = 0; i < 6; ++i) {
      dynamic_offset_(i) = static_cast<sfc::Real>(offset_node[i].as<double>());
    }
    sfc::print(dynamic_offset_, std::cout, "dynamic_offset");
    ROS_INFO("Dynamic offset loaded.");
  } catch (const std::exception& ex) {
    ROS_ERROR("Failed to load dynamic offset: %s", ex.what());
  }

  ROS_INFO("Init wrench sensor dynamic parameters from Yaml");
  const std::string wrenchsensor_yaml_path =
    "/home/sia/girona_ws/src/sensorless_force_control/config/control/model_wrenchsensor.yaml";
  try {
    YAML::Node root = YAML::LoadFile(wrenchsensor_yaml_path);
    const YAML::Node dyn_node = root["params"];
    if (!dyn_node || !dyn_node.IsSequence() || dyn_node.size() != 4) {
      throw std::runtime_error("param must be a sequence of 4 elements");
    }
    for (std::size_t i = 0; i < 4; ++i) {
      wrenchsensor_parameters_(i) = static_cast<sfc::Real>(dyn_node[i].as<double>());
    }
    sfc::print(wrenchsensor_parameters_, std::cout, "wrenchsensor_parameters_");
    ROS_INFO("wrenchsensor parameters loaded.");
  } catch (const std::exception& ex) {
    ROS_ERROR("Failed to load wrenchsensor parameters: %s", ex.what());
  }

  ROS_INFO("Init wrench sensor filter cutoff frequency");
  wrench_filter_.setCutoffHz(30);

  

  dynamic_reconfigure::Server<sensorless_force_control::AdmittanceConfig>::CallbackType cb;
  cb = boost::bind(&GironaController::admittanceReconfigCb, this, _1, _2);
  admittance_server_.setCallback(cb);
  #ifdef USE_ADMITTANCE
    #ifdef USE_VARIABLE_ADMITTANCE
      const sfc::Vector6 K_mass{100,100,100,100,100,100};
      const sfc::Vector6 K_damp{1000,1000,1000,1000,1000,1000};
      const sfc::Vector6 K_stiff{20,20,20,20,20,20};
      variable_admitance_controller_.setGains(K_mass,K_damp,K_stiff);
      const sfc::Vector6 K_s{0.1,0.1,0.1,0.1,0.1,0.1};
      const sfc::Vector6 K_min{20,20,20,20,20,20};
      const sfc::Vector6 K_max{300,300,300,300,300,300};
      const sfc::Vector6 nu_0{0.02,0.02,0.02,0.02,0.02,0.02};  
      variable_admitance_controller_.setVariableStiffnessParams(K_s,K_min,K_max,nu_0);
    #elif defined(USE_IROS_FORCE)
      const sfc::Vector6 mass{20,20,20,20,20,20};
      const sfc::Vector6 damping{100,100,100,100,100,100};
      const sfc::Vector6 stiffness{50,50,50,50,50,50}; 
      const sfc::Vector6 K_s{5.0,5.0,5.0,5.0,5.0,5.0};
      const sfc::Vector6 K_min{20,20,20,20,20,20};
      const sfc::Vector6 K_max{300,300,300,300,300,300};
      iros_force_controller_.setGains(mass,damping,stiffness);
      iros_force_controller_.setForceIntegralParams(K_s,K_min,K_max);
    #else
      const sfc::Vector6 mass{100,100,100,100,100,100};
      const sfc::Vector6 damping{200,200,200,200,200,200};
      const sfc::Vector6 stiffness{1000,1000,1000,1000,1000,1000}; // we can vary the stiffness based on the velocity.
      admitance_controller_.setGains(mass,damping,stiffness);
    #endif
  #endif
  

  #ifdef USE_LOG
    const std::string log_dir = "/home/sia/girona_ws/src/sensorless_force_control/log/";
    const std::time_t now = std::time(nullptr);
    std::tm tm_now{};
    localtime_r(&now, &tm_now);
    std::ostringstream name;
    name << "noised_thruster_controller_data_" << std::put_time(&tm_now, "%Y%m%d%H%M%S") << ".csv";
    const std::string csv_path = log_dir + name.str();

    logger_ = sfc::Logger(csv_path);
    logger_open_ = logger_.open();
    if (!logger_open_) {
      ROS_ERROR("Failed to open CSV file: %s", csv_path.c_str());
    } else {
      ROS_INFO("CSV log path: %s", csv_path.c_str());
    }
  #endif
}

void GironaController::joyCmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  if (!msg) {
    return;
  }
  joy_cmd_ = *msg;
}

void GironaController::eePoseCmdCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (!msg) {
    return;
  }
  ee_pose_cmd_ = *msg;
  ee_pose_cmd_received_ = true;
}

void GironaController::desiredWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  if (!msg) {
    return;
  }
#if defined(USE_IROS_FORCE)
  sfc::Vector6 w_hat_d{};
  w_hat_d(0) = static_cast<sfc::Real>(msg->wrench.force.x);
  w_hat_d(1) = static_cast<sfc::Real>(msg->wrench.force.y);
  w_hat_d(2) = static_cast<sfc::Real>(msg->wrench.force.z);
  w_hat_d(3) = static_cast<sfc::Real>(msg->wrench.torque.x);
  w_hat_d(4) = static_cast<sfc::Real>(msg->wrench.torque.y);
  w_hat_d(5) = static_cast<sfc::Real>(msg->wrench.torque.z);
  iros_force_controller_.setDesiredWrench(w_hat_d);
#endif
}

}  // namespace sfc

int main(int argc, char** argv) {
  ros::init(argc, argv, "girona_controller");
  ros::NodeHandle pnh("~");
  const std::string robot_name = pnh.param<std::string>("robot_name", "girona1000");
  ros::NodeHandle nh("/" + robot_name);
  sfc::GironaController controller(nh, pnh);
  controller.start();
  ros::waitForShutdown();
  controller.stop();
  return 0;
}
