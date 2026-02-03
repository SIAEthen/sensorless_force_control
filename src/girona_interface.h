#ifndef SFC_GIRONA_INTERFACE_H_
#define SFC_GIRONA_INTERFACE_H_

#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <cola2_msgs/Setpoints.h>

#include "functionlib/hardware/manipulator_base.h"
#include "functionlib/hardware/vehicle_base.h"
#include "functionlib/utilts/vector.h"

namespace sfc {

class GironaInterface {
 public:
  static constexpr std::size_t kArmDof = 6;

  GironaInterface(ros::NodeHandle nh, ros::NodeHandle pnh);

  bool haveVehicleState() const;
  bool haveJointState() const;
  bool haveWrench() const;

  sfc::VehicleBase::State vehicleState() const;
  sfc::ManipulatorBase<kArmDof>::State manipulatorState() const;
  sensor_msgs::JointState jointState() const;
  sfc::Vector6 wrench() const;

  void sendThrusterSetpoints(const std::vector<double>& setpoints);
  void sendJointVelocityCommand(const std::vector<double>& velocities);

 private:
  void odomCallback(const nav_msgs::Odometry& msg);
  void jointCallback(const sensor_msgs::JointState& msg);
  void wrenchCallback(const geometry_msgs::WrenchStamped& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber joint_sub_;
  ros::Subscriber wrench_sub_;
  ros::Publisher thruster_pub_;
  ros::Publisher joint_velocity_pub_;

  std::string odom_topic_;
  std::string joint_state_topic_;
  std::string thruster_setpoints_topic_;
  std::string joint_velocity_topic_;
  std::string wrench_topic_;

  mutable std::mutex mutex_;
  sfc::VehicleBase::State vehicle_state_{};
  sfc::ManipulatorBase<kArmDof>::State manip_state_{};
  sensor_msgs::JointState joint_;
  sfc::Vector6 wrench_{};

  bool have_vehicle_ = false;
  bool have_joint_ = false;
  bool have_wrench_ = false;
};

}  // namespace sfc

#endif  // SFC_GIRONA_INTERFACE_H_
