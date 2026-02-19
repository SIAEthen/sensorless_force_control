#ifndef SFC_JOYSTICK_BRIDGE_H_
#define SFC_JOYSTICK_BRIDGE_H_

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

namespace sfc {

class JoystickBridge {
 public:
  JoystickBridge(ros::NodeHandle nh, ros::NodeHandle pnh);

 private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber joy_sub_;
  ros::Publisher joy_raw_pub_;
  ros::Publisher axes_pub_;
  ros::Publisher buttons_pub_;
  ros::Publisher velocity_pub_;

  std::string joy_input_topic_{"joy"};
  std::string joy_raw_topic_{"joystick/raw"};
  std::string axes_topic_{"joystick/axes"};
  std::string buttons_topic_{"joystick/buttons"};
  std::string velocity_topic_{"joystick/velocity_cmd"};
  int queue_size_{10};

  int axis_x_{0};
  int axis_y_{1};
  int axis_z_{2};
  int axis_rx_{3};
  int axis_ry_{4};
  int axis_rz_{5};

  double scale_x_{0.1};
  double scale_y_{0.1};
  double scale_z_{0.1};
  double scale_rx_{0.1};
  double scale_ry_{0.1};
  double scale_rz_{0.1};
};

}  // namespace sfc

#endif  // SFC_JOYSTICK_BRIDGE_H_
