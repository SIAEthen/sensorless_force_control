#!/usr/bin/env python3
"""
HQP scenario 2: move EE along a horizontal circle for 30 s while logging.

The EE starts at (CENTER_X + RADIUS, CENTER_Y, CENTER_Z) and traces a circle
in the XY plane.  Orientation is fixed throughout.

Flow:
  1. Enable logging, wait 5 s
  2. Enable EE task
  3. Publish circular trajectory at 10 Hz for 30 s
  4. Stop publishing, disable EE task
  5. Wait 5 s, disable logging

EE pose command topic: /girona1000xh/ee_pose_cmd  (geometry_msgs/PoseStamped)
Quaternion convention: w, x, y, z  (same as controller)

Usage:
    rosrun sensorless_force_control hqp_continuous_secnario2.py
"""

import math
import rospy
from geometry_msgs.msg import PoseStamped
from dynamic_reconfigure.client import Client as DynClient

# ── Circle centre (initial EE position) ─────────────────────────────────────
CENTER_X =  0.0   # [m]
CENTER_Y =  0.0   # [m]
CENTER_Z =  2.0   # [m]

# ── Circle parameters ────────────────────────────────────────────────────────
RADIUS   =  0.5   # [m]  circle radius
PERIOD_S = 100.0   # [s]  time for one full revolution

# ── Fixed orientation throughout (quaternion w, x, y, z) ────────────────────
TARGET_QW =  0.0
TARGET_QX =  0.0
TARGET_QY = -0.707
TARGET_QZ = -0.707

# ── Timing ───────────────────────────────────────────────────────────────────
PUBLISH_DURATION_S = PERIOD_S
PUBLISH_RATE_HZ    = 10.0
TOPIC              = "/girona1000xh/ee_pose_cmd"


def circle_position(t: float):
    """Return (x, y, z) on the circle at elapsed time t [s]."""
    omega = 2.0 * math.pi / PERIOD_S
    x = CENTER_X + RADIUS * math.cos(omega * t)
    y = CENTER_Y + RADIUS * math.sin(omega * t)
    z = CENTER_Z
    return x, y, z


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
    rospy.init_node("hqp_comparison_scenario2", anonymous=False)
    rospy.loginfo("Waiting for subscriber...")
    rospy.sleep(1.0)

    dyn = DynClient("/girona_controller", timeout=5.0)
    pub = rospy.Publisher(TOPIC, PoseStamped, queue_size=1)

    # ── Pre-generate all trajectory points ────────────────────────────────
    n_points   = int(PUBLISH_DURATION_S * PUBLISH_RATE_HZ)
    dt         = 1.0 / PUBLISH_RATE_HZ
    trajectory = [
        make_pose_msg(*circle_position(i * dt), TARGET_QW, TARGET_QX, TARGET_QY, TARGET_QZ)
        for i in range(n_points)
    ]
    x0, y0, z0 = circle_position(0.0)
    rospy.loginfo(
        f"Trajectory ready: {n_points} points  |  "
        f"centre=({CENTER_X}, {CENTER_Y}, {CENTER_Z})  "
        f"radius={RADIUS} m  period={PERIOD_S} s  "
        f"start=({x0:.3f}, {y0:.3f}, {z0:.3f})"
    )


    rospy.loginfo(f"Starting circular trajectory ({n_points} points at {PUBLISH_RATE_HZ:.0f} Hz)...")
    rate = rospy.Rate(PUBLISH_RATE_HZ)

    dyn.update_configuration({"gain_ee_1": 5,"gain_ee_2": 5,"gain_ee_3": 5,
                              "gain_ee_4": 5,"gain_ee_5": 5,"gain_ee_6": 5})

    # 3. Publish first point, hold 10 s so robot reaches start pose
    trajectory[0].header.stamp = rospy.Time.now()
    for i in range(10):
        pub.publish(trajectory[0])
        rate.sleep()
    rospy.loginfo(f"First point sent: ({x0:.3f}, {y0:.3f}, {z0:.3f}). Waiting 20 s...")
    rospy.sleep(30.0)


    # 1. Enable logging, wait 5 s
    dyn.update_configuration({"enable_logging": True})
    rospy.loginfo("Logging enabled. Waiting 5 s...")
    rospy.sleep(5.0)

    # 2. Enable EE task
    dyn.update_configuration({"enable_ee_task": True})
    rospy.loginfo("EE task enabled.")

    

    # 4. Publish all points at 10 Hz
   
    for idx, msg in enumerate(trajectory):
        if rospy.is_shutdown():
            break
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rospy.loginfo_throttle(
            2.0,
            f"[{idx+1}/{n_points}]  ({msg.pose.position.x:.3f}, "
            f"{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})"
        )
        rate.sleep()

    rospy.loginfo("Trajectory complete.")

    # 5. Disable logging and exit
    dyn.update_configuration({"enable_logging": False})
    rospy.loginfo("Logging disabled. Done.")


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
