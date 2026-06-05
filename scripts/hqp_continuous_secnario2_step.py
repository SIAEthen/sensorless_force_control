#!/usr/bin/env python3
"""
HQP step response test: command a single EE position and observe the response.

Flow:
  1. Enable logging, wait 5 s
  2. Enable EE task
  3. Publish STEP_TARGET at 10 Hz for 50 s
  4. Disable EE task, disable logging, exit

Only position is commanded; orientation is fixed throughout.

EE pose command topic: /girona1000xh/ee_pose_cmd  (geometry_msgs/PoseStamped)

Usage:
    rosrun sensorless_force_control hqp_continuous_secnario2_step.py
"""

import rospy
from geometry_msgs.msg import PoseStamped
from dynamic_reconfigure.client import Client as DynClient

# ── Step target position [m] ─────────────────────────────────────────────────
INT_X = 0.0
INT_Y = 0.0
INT_Z = 2.5

STEP_X =  1.0
STEP_Y =  0.5
STEP_Z =  1.5

# ── Fixed orientation (quaternion w, x, y, z) ────────────────────────────────
TARGET_QW =  0.0
TARGET_QX =  0.0
TARGET_QY = -0.707
TARGET_QZ = -0.707

# ── Timing ───────────────────────────────────────────────────────────────────
HOLD_DURATION_S = 50.0
PUBLISH_RATE_HZ = 10.0
TOPIC           = "/girona1000xh/ee_pose_cmd"


def make_pose_msg(x, y, z, qw, qx, qy, qz) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id    = "world"
    msg.pose.position.x    = x
    msg.pose.position.y    = y
    msg.pose.position.z    = z
    msg.pose.orientation.w = qw
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    return msg


def run():
    rospy.init_node("hqp_step_response", anonymous=False)
    rospy.loginfo("Waiting for subscriber...")
    rospy.sleep(1.0)

    dyn = DynClient("/girona_controller", timeout=5.0)
    pub = rospy.Publisher(TOPIC, PoseStamped, queue_size=1)

    init_msg = make_pose_msg(INT_X,INT_Y,INT_Z,
                             TARGET_QW, TARGET_QX, TARGET_QY, TARGET_QZ)
    step_msg = make_pose_msg(STEP_X, STEP_Y, STEP_Z,
                              TARGET_QW, TARGET_QX, TARGET_QY, TARGET_QZ)

    for i in range(10):
        pub.publish(init_msg)
        rospy.sleep(0.1)
    rospy.loginfo("Logging enabled. Waiting 30 s...")
    rospy.sleep(30.0)

    # 1. Enable logging, wait 5 s
    dyn.update_configuration({"enable_logging": True})
    rospy.loginfo("Logging enabled. Waiting 5 s...")
    rospy.sleep(5.0)


    # 3. Publish step target at 10 Hz for 50 s
    rospy.loginfo(
        f"Step target: ({STEP_X}, {STEP_Y}, {STEP_Z})  "
        f"Holding for {HOLD_DURATION_S:.0f} s ..."
    )

    rate    = rospy.Rate(PUBLISH_RATE_HZ)
    t_start = rospy.Time.now()

    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - t_start).to_sec()
        if elapsed >= HOLD_DURATION_S:
            break
        step_msg.header.stamp = rospy.Time.now()
        pub.publish(step_msg)
        rospy.loginfo_throttle(5.0, f"[t={elapsed:.1f} s]  holding step target")
        rate.sleep()
    
    # 1. disable logging, wait 5 s
    dyn.update_configuration({"enable_logging": False})
    rospy.loginfo("Logging disabled. Waiting 5 s...")
    rospy.sleep(5.0)
    rospy.loginfo("Step hold complete.")




if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass