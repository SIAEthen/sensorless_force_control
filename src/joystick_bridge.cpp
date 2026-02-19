#include "joystick_bridge.h"

#include <utility>

namespace sfc {

JoystickBridge::JoystickBridge(ros::NodeHandle nh, ros::NodeHandle pnh)
    : nh_(std::move(nh)),
      pnh_(std::move(pnh)) {
  pnh_.param<std::string>("joy_input_topic", joy_input_topic_, joy_input_topic_);
  pnh_.param<std::string>("joy_raw_topic", joy_raw_topic_, joy_raw_topic_);
  pnh_.param<std::string>("axes_topic", axes_topic_, axes_topic_);
  pnh_.param<std::string>("buttons_topic", buttons_topic_, buttons_topic_);
  pnh_.param<std::string>("velocity_topic", velocity_topic_, velocity_topic_);
  pnh_.param<int>("queue_size", queue_size_, queue_size_);
  pnh_.param<int>("axis_x", axis_x_, axis_x_);
  pnh_.param<int>("axis_y", axis_y_, axis_y_);
  pnh_.param<int>("axis_z", axis_z_, axis_z_);
  pnh_.param<int>("axis_rx", axis_rx_, axis_rx_);
  pnh_.param<int>("axis_ry", axis_ry_, axis_ry_);
  pnh_.param<int>("axis_rz", axis_rz_, axis_rz_);

  pnh_.param<double>("scale_x", scale_x_, scale_x_);
  pnh_.param<double>("scale_y", scale_y_, scale_y_);
  pnh_.param<double>("scale_z", scale_z_, scale_z_);
  pnh_.param<double>("scale_rx", scale_rx_, scale_rx_);
  pnh_.param<double>("scale_ry", scale_ry_, scale_ry_);
  pnh_.param<double>("scale_rz", scale_rz_, scale_rz_);

  joy_raw_pub_ = nh_.advertise<sensor_msgs::Joy>(joy_raw_topic_, queue_size_);
  axes_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(axes_topic_, queue_size_);
  buttons_pub_ = nh_.advertise<std_msgs::Int32MultiArray>(buttons_topic_, queue_size_);
  velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(velocity_topic_, queue_size_);
  joy_sub_ = nh_.subscribe(joy_input_topic_, queue_size_, &JoystickBridge::joyCallback, this);

  ROS_INFO("Joystick bridge initialized.");
  ROS_INFO("Input topic: %s", joy_input_topic_.c_str());
  ROS_INFO("Output topics: raw=%s, axes=%s, buttons=%s, vel=%s",
           joy_raw_topic_.c_str(), axes_topic_.c_str(), buttons_topic_.c_str(),
           velocity_topic_.c_str());
}

void JoystickBridge::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  joy_raw_pub_.publish(*msg);

  std_msgs::Float64MultiArray axes_msg;
  axes_msg.data.resize(msg->axes.size());
  for (std::size_t i = 0; i < msg->axes.size(); ++i) {
    axes_msg.data[i] = msg->axes[i];
  }
  axes_pub_.publish(axes_msg);

  std_msgs::Int32MultiArray buttons_msg;
  buttons_msg.data.resize(msg->buttons.size());
  for (std::size_t i = 0; i < msg->buttons.size(); ++i) {
    buttons_msg.data[i] = msg->buttons[i];
  }
  buttons_pub_.publish(buttons_msg);

  auto axisValue = [&](int index) -> double {
    if (index < 0) {
      return 0.0;
    }
    const std::size_t idx = static_cast<std::size_t>(index);
    if (idx >= msg->axes.size()) {
      return 0.0;
    }
    return static_cast<double>(msg->axes[idx]);
  };

  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = scale_x_ * axisValue(axis_x_);
  vel_msg.linear.y = scale_y_ * axisValue(axis_y_);
  vel_msg.linear.z = scale_z_ * axisValue(axis_z_);
  vel_msg.angular.x = scale_rx_ * axisValue(axis_rx_);
  vel_msg.angular.y = scale_ry_ * axisValue(axis_ry_);
  vel_msg.angular.z = scale_rz_ * axisValue(axis_rz_);
  velocity_pub_.publish(vel_msg);
}

}  // namespace sfc

int main(int argc, char** argv) {
  ros::init(argc, argv, "joystick_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  sfc::JoystickBridge bridge(nh, pnh);
  ros::spin();
  return 0;
}
