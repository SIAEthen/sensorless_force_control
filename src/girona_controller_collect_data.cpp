#include "girona_controller_collect_data.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <utility>

#include <yaml-cpp/yaml.h>
// #define DEBUG_CONTROLLER 
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
    sfc::Vector6 joint_velocities{};
    ros::Time last = ros::Time::now();
    ros::Time reset_time = ros::Time::now();
    Vector6 desired_joint_position = getRandomJointPosition();
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
        const sfc::Vector6 ft_sensor_feedback = interface_.wrench();




        constexpr std::size_t kSysDof = 12;
        sfc::Matrix<kSysDof, kSysDof> N = sfc::identity<kSysDof>();
        sfc::Vector<kSysDof> zeta{};
        const sfc::Real damping = static_cast<sfc::Real>(1e-3);
        
        // Task: Vehicle position
        const sfc::Vector<3> xyz_ref{0.0,0.0,2.0};
        const sfc::Vector<3> xyz_gain{2,2,1};
        sfc::Matrix<3, kSysDof> J_xyz{};
        sfc::Vector<3> sigma_xyz{};
        sfc::Vector<3> task_vel_xyz{};
        sfc::buildVehiclePositionTask(uvms_, xyz_ref, J_xyz, sigma_xyz);
        sfc::buildTaskVelocity<3>(sfc::Vector3{},sigma_xyz,xyz_gain,task_vel_xyz);
        zeta = sfc::taskPrioritySolveStep<kSysDof, 3>(task_vel_xyz, J_xyz, N, zeta, damping);
        #ifdef DEBUG_CONTROLLER
          sfc::print(sigma_xyz,std::cout,"sigma_xyz");
          sfc::print(zeta,std::cout,"zeta");
        #endif
        // Task 1: roll/pitch stabilization
        // sfc::Matrix<2, kSysDof> J_rp{};
        // sfc::Vector<2> sigma_rp{};
        // const sfc::Vector<2> rp_ref{0.0,0.0};
        // sfc::buildRollPitchTask(uvms_, rp_ref, J_rp, sigma_rp);
        // zeta = sfc::taskPrioritySolveStep<kSysDof, 2>(sigma_rp, J_rp, N, zeta, damping);
        // Task 1.5: roll/pitch/yaw stabilization
        const sfc::Vector<3> rpy_ref{0.0,0.0,0.5};
        const sfc::Vector<3> rpy_gain{0.0,1.0,2.0};
        sfc::Matrix<3, kSysDof> J_rpy{};
        sfc::Vector<3> sigma_rpy{};
        sfc::Vector<3> task_vel_rpy{};
        sfc::buildRollPitchYawTask(uvms_, rpy_ref, J_rpy, sigma_rpy);
        sfc::buildTaskVelocity<3>(sfc::Vector3{},sigma_rpy,rpy_gain,task_vel_rpy);
        zeta = sfc::taskPrioritySolveStep<kSysDof, 3>(task_vel_rpy, J_rpy, N, zeta, damping);
        #ifdef DEBUG_CONTROLLER
          sfc::print(sigma_rpy,std::cout,"sigma_rpy");
          sfc::print(zeta,std::cout,"zeta");
        #endif


        // Task 3: nominal joint configuration
        const sfc::Vector<6> nominal_config = desired_joint_position;
        const sfc::Vector<6> nominal_gain{1,1,1,1,1,1};
        sfc::Matrix<6, kSysDof> J_nominal{};
        sfc::Vector<6> sigma_nominal{};
        sfc::Vector<6> task_vel_nominal{};
        sfc::buildNominalConfigTask(uvms_, nominal_config, J_nominal, sigma_nominal);
        sfc::buildTaskVelocity<6>(sfc::Vector6{},sigma_nominal,nominal_gain,task_vel_nominal);
        zeta = sfc::taskPrioritySolveStep<kSysDof, 6>(sigma_nominal, J_nominal, N, zeta, damping);
        #ifdef DEBUG_CONTROLLER
          sfc::print(zeta,std::cout,"velocity");
        #endif

        if(sfc::vectorNorm(uvms_.vehicleVelocity())< 0.01
            && sfc::vectorNorm(uvms_.manipulatorPosition()-desired_joint_position)< 0.05
            && std::fabs(uvms_.vehicleRpy()(1))< 0.01
            && (ros::Time::now().toSec()-reset_time.toSec()) > 30 // big than 30 seconds
          )
          {
            reset_time = ros::Time::now(); //reset time
            desired_joint_position = getRandomJointPosition();
            sfc::print(desired_joint_position,std::cout,"desired_joint_position");
            logFrame(ros::Time::now().toSec(),
                      uvms_.vehiclePosition(),
                      uvms_.vehicleRpy(),
                      uvms_.manipulatorPosition(),
                      setpoints,
                      zeta,
                      sigma_xyz,
                      sigma_rpy,
                      sigma_nominal,
                      ft_sensor_feedback);
            
          }else{
            std::cout << sfc::vectorNorm(uvms_.vehicleVelocity()) << " " 
            << sfc::vectorNorm(uvms_.manipulatorPosition()-desired_joint_position) << " " 
            << std::fabs(uvms_.vehicleRpy()(1)) << std::endl;
          }



        sfc::Vector6 nu_d{zeta(0),zeta(1),zeta(2),zeta(3),zeta(4),zeta(5)};
        sfc::Vector6 error = nu_d - uvms_.vehicleVelocity();
        sfc::Vector6 control_wrench = pid_.update(error,dt);
        sfc::Vector6 force = allocator_.allocate(control_wrench,0.0001);
        setpoints = convertForceToSetpoints(force);
        #ifdef DEBUG_CONTROLLER
          sfc::print(control_wrench,std::cout,"control wrench");
          sfc::print(setpoints,std::cout,"setpoints");
        #endif
        joint_velocities(0) = zeta(6);
        joint_velocities(1) = zeta(7);
        joint_velocities(2) = zeta(8);
        joint_velocities(3) = zeta(9);
        joint_velocities(4) = zeta(10);
        joint_velocities(5) = zeta(11);

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
  sfc::Vector6 kp{50,50,50,0,40,20};
  sfc::Vector6 ki{1,1,5,0,8,1};
  sfc::Vector6 kd{8,8,8,0,4,4};
  sfc::Vector6 i_sat{50,50,100,0,50,10};
  pid_.setGains(kp,ki,kd);
  pid_.setIntegratorLimits(i_sat);

  p_.setGains(sfc::Vector6{1,1,1,1,1,1});

  const std::string log_dir = "/home/sia/girona_ws/src/sensorless_force_control/log/";
  const std::time_t now = std::time(nullptr);
  std::tm tm_now{};
  localtime_r(&now, &tm_now);
  std::ostringstream name;
  name << "controller_data_" << std::put_time(&tm_now, "%Y%m%d%H%M%S") << ".csv";
  const std::string csv_path = log_dir + name.str();

  logger_ = sfc::Logger(csv_path);
  logger_open_ = logger_.open();
  if (!logger_open_) {
    ROS_ERROR("Failed to open CSV file: %s", csv_path.c_str());
  } else {
    ROS_INFO("CSV log path: %s", csv_path.c_str());
  }

}

void GironaController::logFrame(double stamp_sec,
                                const sfc::Vector3& vehicle_xyz,
                                const sfc::Vector3& vehicle_rpy,
                                const sfc::Vector6& current_joint,
                                const sfc::Vector6& setpoints,
                                const sfc::Vector<12>& zeta,
                                const sfc::Vector3& xyz_err,
                                const sfc::Vector3& rpy_err,
                                const sfc::Vector6& nominal_err,
                                const sfc::Vector6& wrench_sensor) {
  if (!logger_open_) {
    return;
  }
  logger_.beginFrame(stamp_sec);
  logger_.logVector("vehicle_xyz", vehicle_xyz);
  logger_.logVector("vehicle_rpy", vehicle_rpy);
  logger_.logVector("current_joint", current_joint);
  logger_.logVector("setpoints", setpoints);
  logger_.logVector("zeta", zeta);
  logger_.logVector("xyz_err", xyz_err);
  logger_.logVector("rpy_err", rpy_err);
  logger_.logVector("nominal_err", nominal_err);
  logger_.logVector("wrench_sensor", wrench_sensor);
  logger_.endFrame();
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
