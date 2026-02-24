#include "girona_interface.h"

#include <algorithm>
#include <utility>

#include "functionlib/utilts/rotation.h"

namespace sfc {

constexpr std::size_t GironaInterface::kArmDof;

GironaInterface::GironaInterface(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh)), pnh_(std::move(pnh)) {
  odom_topic_ = pnh_.param<std::string>("odom_topic", "dynamics/odometry");
  joint_state_topic_ = pnh_.param<std::string>("joint_state_topic", "joint_states");
  thruster_setpoints_topic_ =
      pnh_.param<std::string>("thruster_setpoints_topic",
                              "controller/passthrough_thruster_setpoints");
  joint_velocity_topic_ =
      pnh_.param<std::string>("joint_velocity_topic",
                              "controller/joint_velocity_cmd");
  wrench_topic_ = pnh_.param<std::string>("wrench_topic", "ft_sensor/force_torque");

  odom_sub_ = nh_.subscribe(odom_topic_, 10, &GironaInterface::odomCallback, this);
  joint_sub_ = nh_.subscribe(joint_state_topic_, 10, &GironaInterface::jointCallback, this);
  wrench_sub_ = nh_.subscribe(wrench_topic_, 10, &GironaInterface::wrenchCallback, this);
  thruster_pub_ = nh_.advertise<cola2_msgs::Setpoints>(thruster_setpoints_topic_, 10);
  joint_velocity_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(joint_velocity_topic_, 10);
}

bool GironaInterface::haveVehicleState() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return have_vehicle_;
}

bool GironaInterface::haveJointState() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return have_joint_;
}

bool GironaInterface::haveWrench() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return have_wrench_;
}

sfc::VehicleBase::State GironaInterface::vehicleState() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return vehicle_state_;
}

sfc::ManipulatorBase<GironaInterface::kArmDof>::State GironaInterface::manipulatorState() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return manip_state_;
}

sensor_msgs::JointState GironaInterface::jointState() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return joint_;
}

sfc::Vector6 GironaInterface::wrench() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return wrench_;
}

void GironaInterface::sendThrusterSetpoints(const sfc::Vector6& setpoints) {
  cola2_msgs::Setpoints msg;
  msg.header.stamp = ros::Time::now();
  msg.setpoints.assign(setpoints.data.begin(), setpoints.data.end());
  thruster_pub_.publish(msg);
}

void GironaInterface::sendJointVelocityCommand(const sfc::Vector6& velocities) {
  std_msgs::Float64MultiArray msg;
  msg.data.assign(velocities.data.begin(), velocities.data.end());
  joint_velocity_pub_.publish(msg);
}

void GironaInterface::odomCallback(const nav_msgs::Odometry& msg) {
  sfc::VehicleBase::State state{};
  const sfc::Vector3 p_ned_base{
      static_cast<sfc::Real>(msg.pose.pose.position.x),
      static_cast<sfc::Real>(msg.pose.pose.position.y),
      static_cast<sfc::Real>(msg.pose.pose.position.z)};

  const auto& q_msg = msg.pose.pose.orientation;
  const sfc::Quaternion q{static_cast<sfc::Real>(q_msg.w),
                          static_cast<sfc::Real>(q_msg.x),
                          static_cast<sfc::Real>(q_msg.y),
                          static_cast<sfc::Real>(q_msg.z)};
  const sfc::RotationMatrix r_ned_base = sfc::RotationMatrix::fromQuaternion(q);
  // Static TF from base_link to origin from `tf_echo`:
  // base_link -> origin: translation [0, 0, -0.214], rotation identity.
  const sfc::Vector3 p_base_origin{0.0, 0.0, -0.214};
  const sfc::Vector3 p_ned_origin = p_ned_base + (r_ned_base * p_base_origin);

  state.position[0] = p_ned_origin(0);
  state.position[1] = p_ned_origin(1);
  state.position[2] = p_ned_origin(2);

  const sfc::Vector3 rpy = sfc::rpyFromQuaternion(q);
  state.position[3] = rpy(0);
  state.position[4] = rpy(1);
  state.position[5] = rpy(2);

  // Odometry twist is assumed to be expressed in child_frame_id (base_link).
  // Convert linear velocity from base origin to origin point with rigid-body relation:
  // v_origin = v_base + omega x r_base_to_origin (all in base frame).
  const sfc::Vector3 v_base{
      static_cast<sfc::Real>(msg.twist.twist.linear.x),
      static_cast<sfc::Real>(msg.twist.twist.linear.y),
      static_cast<sfc::Real>(msg.twist.twist.linear.z)};
  const sfc::Vector3 omega_base{
      static_cast<sfc::Real>(msg.twist.twist.angular.x),
      static_cast<sfc::Real>(msg.twist.twist.angular.y),
      static_cast<sfc::Real>(msg.twist.twist.angular.z)};
  const sfc::Vector3 v_origin = v_base + sfc::cross(omega_base, p_base_origin);

  state.velocity[0] = v_origin(0);
  state.velocity[1] = v_origin(1);
  state.velocity[2] = v_origin(2);
  state.velocity[3] = omega_base(0);
  state.velocity[4] = omega_base(1);
  state.velocity[5] = omega_base(2);

  std::lock_guard<std::mutex> lock(mutex_);
  vehicle_state_ = state;
  have_vehicle_ = true;
}

void GironaInterface::jointCallback(const sensor_msgs::JointState& msg) {
  sfc::ManipulatorBase<kArmDof>::State state{};
  const std::size_t count = std::min<std::size_t>(msg.position.size(), kArmDof);
  for (std::size_t i = 0; i < kArmDof; ++i) {
    state.q[i] = static_cast<sfc::Real>(msg.position[i]);
  }

  const std::size_t vel_count = std::min<std::size_t>(msg.velocity.size(), kArmDof);
  for (std::size_t i = 0; i < vel_count; ++i) {
    state.dq[i] = static_cast<sfc::Real>(msg.velocity[i]);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  joint_ = msg;
  manip_state_ = state;
  have_joint_ = true;
}

void GironaInterface::wrenchCallback(const geometry_msgs::WrenchStamped& msg) {
  sfc::Vector6 wrench{};
  wrench(0) = static_cast<sfc::Real>(msg.wrench.force.x);
  wrench(1) = static_cast<sfc::Real>(msg.wrench.force.y);
  wrench(2) = static_cast<sfc::Real>(msg.wrench.force.z);
  wrench(3) = static_cast<sfc::Real>(msg.wrench.torque.x);
  wrench(4) = static_cast<sfc::Real>(msg.wrench.torque.y);
  wrench(5) = static_cast<sfc::Real>(msg.wrench.torque.z);

  std::lock_guard<std::mutex> lock(mutex_);
  wrench_ = wrench;
  have_wrench_ = true;
}

}  // namespace sfc
