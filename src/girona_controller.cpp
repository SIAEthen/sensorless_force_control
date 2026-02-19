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

}  // namespace

namespace sfc {

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
  admitance_controller_.setGains(mass, damping, stiffness);
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
    sfc::Vector6 joint_velocities{};
    ros::Time last = ros::Time::now();

    // variable for admittance control
    sfc::Vector3 x_ee_d{0,3.5,2.0};
    // sfc::Quaternion q_ee_d = sfc::Quaternion::fromRPY(-sfc::kPi2,-sfc::kPi2,0.0);
    sfc::Quaternion q_ee_d = sfc::Quaternion{0,0,-0.707,-0.707};

    // test the accuracy, touch the sheep surface.
    // sfc::Vector3 x_ee_d{0.165781, -4.08583, 3.0};
    // sfc::Quaternion q_ee_d = sfc::Quaternion{-0.903388,0.0184309, -0.00873894, -0.428338};

    sfc::Vector6 v_ee_d{};

    sfc::Vector3 x_ee_r = x_ee_d;
    sfc::Quaternion q_ee_r = q_ee_d;
    sfc::Vector6 v_ee_r{};
    sfc::Vector6 a_ee_r{};
    admitance_controller_.reset(x_ee_d,q_ee_d);



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
        const Vector6 velcmd{joy_cmd_.linear.x,joy_cmd_.linear.y,joy_cmd_.linear.z,
                             joy_cmd_.angular.x,joy_cmd_.angular.y,joy_cmd_.angular.z};
        velcmd2configurations(velcmd,x_ee_d,q_ee_d,dt);
        #ifdef DEBUG_JOYSTICK
            sfc::print(velcmd,std::cout,"velcmd");
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

        const sfc::Vector<6> q_min{-3*sfc::kPi4,-sfc::kPi2,-sfc::kPi2,-3*sfc::kPi4, -sfc::kPi, 0.0};
        const sfc::Vector<6> q_max{ 3*sfc::kPi4, sfc::kPi2, sfc::kPi2, 3*sfc::kPi4,  0.0,      3.5*sfc::kPi4};
        const sfc::Real rho = 0.2;
        const sfc::Real ds = 0.2;
        const sfc::Real gain = 0.1;
        sfc::Matrix<6, kSysDof> J_jointlimits{};
        sfc::Vector6 sigma_jointlimits{};
        sfc::buildJointLimitDamperTask<6>(uvms_,q_min,q_max,rho,ds,gain,J_jointlimits,sigma_jointlimits);
        zeta = sfc::taskPrioritySolveStep<kSysDof, 6>(sigma_jointlimits, J_jointlimits, N, zeta, damping);
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

        // Task 1.5: roll/pitch/yaw stabilization
        const sfc::Vector<3> ref_rpy{0.0,0.0,sfc::kPi2};
        const sfc::Vector<3> gain_rpy{0.0,1.0,2.0};
        sfc::Matrix<3, kSysDof> J_rpy{};
        sfc::Vector<3> sigma_rpy{};
        sfc::Vector<3> task_vel_rpy{};
        sfc::buildRollPitchYawTask(uvms_, ref_rpy, J_rpy, sigma_rpy);
        sfc::buildTaskVelocity<3>(sfc::Vector3{},sigma_rpy,gain_rpy,task_vel_rpy);
        zeta = sfc::taskPrioritySolveStep<kSysDof, 3>(task_vel_rpy, J_rpy, N, zeta, damping);
        #ifdef DEBUG_CONTROLLER
          sfc::print(sigma_rpy,std::cout,"sigma_rpy");
          sfc::print(zeta,std::cout,"zeta");
        #endif

        // Task 2: end-effector task (set your references)
        const sfc::Vector3 ref_ee_pos{0,0,2.5};
        // const sfc::Quaternion ref_ee_quat = uvms_.endEffectorQuaternionNed();
        const sfc::Quaternion ref_ee_quat = sfc::Quaternion::fromRPY(-sfc::kPi2,-sfc::kPi2,0.0);
        const sfc::Vector<6> gain_ee{1.0,1.0,1.0,10.0,10.0,10.0};
        sfc::Matrix<6, kSysDof> J_ee{};
        sfc::Vector<6> sigma_ee{};
        sfc::Vector<6> task_vel_ee{};
        // build ee task with desired quaternion or referenced quaternion
        sfc::buildEeTask(uvms_, x_ee_r, q_ee_d, J_ee, sigma_ee);
        // sfc::buildTaskVelocity<6>(v_ee_r,sigma_ee,gain_ee,task_vel_ee);
        sfc::buildTaskVelocity<6>(Vector6{},sigma_ee,gain_ee,task_vel_ee);
        zeta = sfc::taskPrioritySolveStep<kSysDof, 6>(sigma_ee, J_ee, N, zeta, damping);
        #ifdef DEBUG_CONTROLLER
          sfc::print(sigma_ee,std::cout,"sigma_ee");
          sfc::print(zeta,std::cout,"zeta");
        #endif

        // Task 3: nominal joint configuration
        const sfc::Vector<6> nominal_config{0,0,0,0,0,1.0};
        const sfc::Vector<6> gain_nominal_config{10,10,10,10,10,10};
        sfc::Matrix<6, kSysDof> J_nominal{};
        sfc::Vector<6> sigma_nominal{};
        sfc::Vector<6> task_vel_nominal{};
        sfc::buildNominalConfigTask(uvms_, nominal_config, J_nominal, sigma_nominal);
        sfc::buildTaskVelocity<6>(sfc::Vector6{},sigma_nominal,gain_nominal_config,task_vel_nominal);
        zeta = sfc::taskPrioritySolveStep<kSysDof, 6>(sigma_nominal, J_nominal, N, zeta, damping);
        #ifdef DEBUG_CONTROLLER
          sfc::print(sigma_nominal,std::cout,"sigma_nominal");
          sfc::print(zeta,std::cout,"velocity");
        #endif

        sfc::Vector6 nu_d{zeta(0),zeta(1),zeta(2),zeta(3),zeta(4),zeta(5)};
        sfc::Vector6 error = nu_d - uvms_.vehicleVelocity();
        sfc::Vector6 control_wrench = pid_.update(error,dt);
        sfc::Vector6 force = allocator_.allocate(control_wrench,0.0001);
        setpoints = convertForceToSetpoints(force);
        joint_velocities(0) = zeta(6);
        joint_velocities(1) = zeta(7);
        joint_velocities(2) = zeta(8);
        joint_velocities(3) = zeta(9);
        joint_velocities(4) = zeta(10);
        joint_velocities(5) = zeta(11);

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
        #ifdef DEBUG_OBSERVER
          sfc::print(sensor_feedback,std::cout,"sensor_feedback");
          sfc::print(sensor_feedback_filtered,std::cout,"sensor_feedback_filtered");
          sfc::print(sensor_feedback_calibrated,std::cout,"sensor_feedback_calibrated");
          sfc::print(sensor_feedback_calibrated_ontiplink,std::cout,"sensor_feedback_calibrated_ontiplink");
        #endif

        // observer 
        const Vector6 nu = uvms_.vehicleVelocity();
        const Vector3 nu_1{nu(0),nu(1),nu(2)};
        const Vector3 nu_2{nu(3),nu(4),nu(5)};
        const Vector3 acc = linear_acc_observer_.update(nu_1,dt);
        const Vector6 gravity = sfc::regressor_girona1000(uvms_.vehicleRpy(),
                                                        uvms_.manipulator(),
                                                        uvms_.manipulatorBaseToVehicleTransform())
                                * dynamic_parameters_;                        
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

        // admittance controller
        sfc::QuaternionAdmittanceController::Output out;
        // const Vector6 contact_force_torque = sensor_feedback_calibrated_ontiplink;
        const Vector6 contact_force_torque = h_e_inertiaframe;
        out=admitance_controller_.update(x_ee_d,q_ee_d,v_ee_d,contact_force_torque,dt);
        x_ee_r = out.pos_r;
        q_ee_r = out.q_r;
        v_ee_r = out.nu_r;
        a_ee_r = out.acc_r;
        
        #ifdef DEBUG_ADMITTANCE
          sfc::print(x_ee_d,std::cout,"x_ee_d");
          sfc::print(x_ee_r,std::cout,"x_ee_r");
          sfc::print(q_ee_d,std::cout,"q_ee_d");
          sfc::print(q_ee_r,std::cout,"q_ee_r");
          sfc::print(v_ee_d,std::cout,"v_ee_d");
          sfc::print(v_ee_r,std::cout,"v_ee_r");
          sfc::print(a_ee_r,std::cout,"a_ee_r");
        #endif
        



        #ifdef DEBUG_ROSTOPIC
          publishArray6(control_wrench_array_pub_, control_wrench);
          publishArray6(force_array_pub_, force);
          publishArray6(setpoints_array_pub_, setpoints);
          publishArray6(nu_d_array_pub_, nu_d);
          publishArray6(joint_velocities_array_pub_, joint_velocities);
          publishArray6(error_array_pub_, error);
          publishArray6(gravity_pub_, gravity);
          publishArray6(thrusts_pub_, thrusts);
          publishArray6(computed_control_wrench_pub_, computed_control_wrench);
          publishArray6(gravity_minus_tau_v_pub_, gravity_minus_tau_v);
          publishArray6(tau_e_pub_, tau_e);
          publishArray6(h_e_inertiaframe_pub_, h_e_inertiaframe);
          publishArray6(h_e_bodyframe_pub_, h_e_bodyframe);
          publishArray6(h_e_tipframe_pub_, h_e_tipframe);
          publishArray6(sensor_feedback_pub_, sensor_feedback);
          publishArray6(sensor_calibrated_pub_, sensor_feedback_calibrated);
          publishArray6(sensor_calibrated_tiplink_pub_, sensor_feedback_calibrated_ontiplink);
          
        #endif

        #ifdef USE_CONTROL
          interface_.sendThrusterSetpoints(setpoints);
          interface_.sendJointVelocityCommand(joint_velocities);
        #endif
        
        // fix rate god tool
        rate.sleep();
      }

}

void GironaController::initializeController() {
  joycmd_sub_ = nh_.subscribe<geometry_msgs::Twist>(
      "/girona1000xh/joystick/velocity_cmd", 10, &GironaController::joyCmdCallback, this);

#ifdef DEBUG_ROSTOPIC
  control_wrench_array_pub_ =
      nh_.advertise<std_msgs::Float64MultiArray>("debug/control_wrench", 10);
  force_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/force", 10);
  setpoints_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/setpoints", 10);
  nu_d_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/nu_d", 10);
  joint_velocities_array_pub_ =
      nh_.advertise<std_msgs::Float64MultiArray>("debug/joint_velocities", 10);
  error_array_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/error", 10);
#endif

  gravity_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/gravity", 10);
  thrusts_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/thrusts", 10);
  computed_control_wrench_pub_ =
      nh_.advertise<std_msgs::Float64MultiArray>("debug/computed_control_wrench", 10);
  gravity_minus_tau_v_pub_ =
      nh_.advertise<std_msgs::Float64MultiArray>("debug/gravity_minus_tau_v", 10);
  tau_e_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("debug/tau_e", 10);
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
  sfc::Vector3 t = sfc::Vector3{0.732, -0.138, 0.271}; //from bravo/base_link to girona1000/base_link
  // sfc::Vector3 t = sfc::Vector3{0.732, -0.138, 0.485};    //from bravo/base_link to girona1000/origin
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
          "/home/sia/girona_ws/src/sensorless_force_control/config/control/tcm.yaml";
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
    sfc::print(tcm,std::cout,"TCM matrix");
    ROS_INFO("TCM matrix loaded and set.");
  } catch (const std::exception& ex) {
    ROS_ERROR("Failed to load TCM: %s", ex.what());
  }

  ROS_INFO("Init PID controller");
  sfc::Vector6 kp{50,50,50,20,30,20};
  sfc::Vector6 ki{1,1,5,1,5,1};
  sfc::Vector6 kd{8,8,8,2,4,4};
  sfc::Vector6 i_sat{50,50,100,10,50,10};
  pid_.setGains(kp,ki,kd);
  pid_.setIntegratorLimits(i_sat);

  ROS_INFO("Init Accleration observer");
  linear_acc_observer_.setCutoffHz(50);

  ROS_INFO("Init Contact wrench observer");
  wrench_observer_.setMass(153.121);
  wrench_observer_.setInertia(sfc::Vector3{94.660, 103.801, 147.540});
  wrench_observer_.setGains(sfc::Vector3{1.0,1.0,1.0},sfc::Vector3{1.0,1.0,1.0});


  ROS_INFO("Init dynamic parameters from Yaml");
  const std::string dyn_yaml_path =
    "/home/sia/girona_ws/src/sensorless_force_control/config/control/model_optimized.yaml";
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

  const sfc::Vector6 mass{100,100,100,100,100,100};
  const sfc::Vector6 damping{200,200,200,200,200,200};
  const sfc::Vector6 stiffness{1000,1000,1000,1000,1000,1000}; // we can vary the stiffness based on the velocity.
  admitance_controller_.setGains(mass,damping,stiffness);

  dynamic_reconfigure::Server<sensorless_force_control::AdmittanceConfig>::CallbackType cb;
  cb = boost::bind(&GironaController::admittanceReconfigCb, this, _1, _2);
  admittance_server_.setCallback(cb);

}

void GironaController::joyCmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  if (!msg) {
    return;
  }
  joy_cmd_ = *msg;  // 先存起来，控制循环里再用
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
