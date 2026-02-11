#include "girona_controller.h"

#include <chrono>
#include <utility>

#include <yaml-cpp/yaml.h>
#define DEBUG_CONTROLLER

namespace sfc {

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
    while (control_running_.load() && ros::ok()) {
      
        // joint_velocities[0] = -0.1;
        // joint_velocities[1] = -0.1;
        // joint_velocities[2] = -0.1;
        // joint_velocities[3] = -0.1;
        // joint_velocities[4] = -0.1;
        // joint_velocities[5] = -0.1;
        // (void)interface_.vehicleState();
        // (void)interface_.manipulatorState();
        // (void)interface_.wrench();
        ros::Time now = ros::Time::now();
        double dt = (now - last).toSec();
        last = now;

        uvms_.setVehicleState(interface_.vehicleState());
        uvms_.setManipulatorState(interface_.manipulatorState());




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
        
        // // Task 1: roll/pitch stabilization
        const sfc::Vector<2> ref_rp{0.0,0.0};
        const sfc::Vector<2> gain_rp{0.0,1.0};
        sfc::Matrix<2, kSysDof> J_rp{};
        sfc::Vector<2> sigma_rp{};
        sfc::Vector<2> task_vel_rp{};
        sfc::buildRollPitchTask(uvms_, ref_rp, J_rp, sigma_rp);
        sfc::buildTaskVelocity<2>(sfc::Vector<2>{},sigma_rp,gain_rp,task_vel_rp);
        zeta = sfc::taskPrioritySolveStep<kSysDof, 2>(task_vel_rp, J_rp, N, zeta, damping);
        #ifdef DEBUG_CONTROLLER
          sfc::print(sigma_rp,std::cout,"sigma_rp");
          sfc::print(zeta,std::cout,"zeta");
        #endif

        // // Task 1.5: roll/pitch/yaw stabilization
        // const sfc::Vector<3> ref_rpy{0.0,0.0,sfc::kPi2};
        // const sfc::Vector<3> gain_rpy{0.0,1.0,2.0};
        // sfc::Matrix<3, kSysDof> J_rpy{};
        // sfc::Vector<3> sigma_rpy{};
        // sfc::Vector<3> task_vel_rpy{};
        // sfc::buildRollPitchYawTask(uvms_, ref_rpy, J_rpy, sigma_rpy);
        // sfc::buildTaskVelocity<3>(sfc::Vector3{},sigma_rpy,gain_rpy,task_vel_rpy);
        // zeta = sfc::taskPrioritySolveStep<kSysDof, 3>(task_vel_rpy, J_rpy, N, zeta, damping);
        // #ifdef DEBUG_CONTROLLER
        //   sfc::print(sigma_rpy,std::cout,"sigma_rpy");
        //   sfc::print(zeta,std::cout,"zeta");
        // #endif

        // Task 2: end-effector task (set your references)
        const sfc::Vector3 ref_ee_pos{0,0,2.5};
        // const sfc::Quaternion ref_ee_quat = uvms_.endEffectorQuaternionNed();
        const sfc::Quaternion ref_ee_quat = sfc::Quaternion::fromRPY(-sfc::kPi2,-sfc::kPi2,0.0);
        const sfc::Vector<6> gain_ee{1.0,1.0,1.0,10.0,10.0,10.0};
        sfc::Matrix<6, kSysDof> J_ee{};
        sfc::Vector<6> sigma_ee{};
        sfc::Vector<6> task_vel_ee{};
        sfc::buildEeTask(uvms_, ref_ee_pos, ref_ee_quat, J_ee, sigma_ee);
        sfc::buildTaskVelocity<6>(sfc::Vector6{},sigma_ee,gain_ee,task_vel_ee);
        zeta = sfc::taskPrioritySolveStep<kSysDof, 6>(sigma_ee, J_ee, N, zeta, damping);
        #ifdef DEBUG_CONTROLLER
          sfc::print(sigma_ee,std::cout,"sigma_ee");
          sfc::print(zeta,std::cout,"zeta");
        #endif

        // Task 3: nominal joint configuration
        const sfc::Vector<6> nominal_config{};
        const sfc::Vector<6> gain_nominal_config{1,1,1,1,1,1};
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

        interface_.sendThrusterSetpoints(setpoints);
        interface_.sendJointVelocityCommand(joint_velocities);
        rate.sleep();
      }

}

void GironaController::initializeController() {
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
