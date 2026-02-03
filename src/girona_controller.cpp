#include "girona_controller.h"

#include <chrono>
#include <utility>

namespace sfc {

GironaController::GironaController(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh)),
      pnh_(std::move(pnh)),
      interface_(nh_, pnh_),
      uvms_("vehicle", "arm"),
      spinner_(1) {
  n_thrusters_ = static_cast<std::size_t>(pnh_.param<int>("n_thrusters", 6));
  initUvms();
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
    ros::Rate rate(50.0);
    std::vector<double> setpoints(n_thrusters_, 0.0);
    std::vector<double> joint_velocities(GironaInterface::kArmDof, 0.0);

    while (control_running_.load() && ros::ok()) {
      
        joint_velocities[0] = -0.1;
        (void)interface_.vehicleState();
        (void)interface_.manipulatorState();
        (void)interface_.wrench();
        interface_.sendThrusterSetpoints(setpoints);
        interface_.sendJointVelocityCommand(joint_velocities);
        rate.sleep();
      }

}

void GironaController::initUvms() {
  // Placeholder for DH parameters and transforms; configure as needed by your arm.
  
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
