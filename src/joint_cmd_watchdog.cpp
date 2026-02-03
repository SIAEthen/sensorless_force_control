#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

class JointCmdWatchdog {
 public:
  JointCmdWatchdog(ros::NodeHandle nh, ros::NodeHandle pnh)
      : nh_(std::move(nh)), pnh_(std::move(pnh)) {
    input_topic_ = pnh_.param<std::string>("input_topic", "controller/joint_velocity_cmd");
    output_topic_ =
        pnh_.param<std::string>("output_topic", "bravo/joint_velocity_controller/command");
    timeout_sec_ = pnh_.param<double>("timeout_sec", 0.2);
    publish_rate_hz_ = pnh_.param<double>("publish_rate_hz", 50.0);
    dof_ = static_cast<std::size_t>(pnh_.param<int>("dof", 6));

    sub_ = nh_.subscribe(input_topic_, 10, &JointCmdWatchdog::cmdCallback, this);
    pub_ = nh_.advertise<std_msgs::Float64MultiArray>(output_topic_, 10);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(publish_rate_hz_, 1e-3)),
                             &JointCmdWatchdog::onTimer, this);
  }

 private:
  void cmdCallback(const std_msgs::Float64MultiArray& msg) {
    last_cmd_ = msg;
    last_cmd_time_ = ros::Time::now();
    have_cmd_ = true;
  }

  void onTimer(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    const bool stale = !have_cmd_ || (now - last_cmd_time_).toSec() > timeout_sec_;

    if (stale) {
      if (!was_stale_) {
        ROS_WARN("watchdog: joint vel cmd time out");
      }
      was_stale_ = true;
      std_msgs::Float64MultiArray zero;
      zero.data.assign(dof_, 0.0);
      pub_.publish(zero);
      return;
    }

    if (was_stale_) {
      ROS_WARN("watchdog: joint vel cmd starts");
    }
    was_stale_ = false;
    pub_.publish(last_cmd_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Timer timer_;

  std::string input_topic_;
  std::string output_topic_;
  double timeout_sec_{0.2};
  double publish_rate_hz_{50.0};
  std::size_t dof_{6};

  std_msgs::Float64MultiArray last_cmd_;
  ros::Time last_cmd_time_{0};
  bool have_cmd_{false};
  bool was_stale_{true};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "joint_cmd_watchdog");
  ros::NodeHandle pnh("~");
  const std::string robot_name = pnh.param<std::string>("robot_name", "girona1000");
  ros::NodeHandle nh("/" + robot_name);

  JointCmdWatchdog watchdog(nh, pnh);
  ros::spin();
  return 0;
}
