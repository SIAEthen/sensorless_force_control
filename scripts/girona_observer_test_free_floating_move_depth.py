#!/usr/bin/env python3
"""
Publish a 3-segment end-effector position trajectory.
Trajectory: A -> B (5s) -> C (5s) -> D (5s)
Orientation fixed throughout.

Usage:
    rosrun sensorless_force_control send_ee_trajectory.py
"""

import numpy as np
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from dynamic_reconfigure.client import Client as DynClient

# ── 修改这里定义轨迹点 (NED frame, 单位: m) ─────────────────────────────────

PUBLISH_HZ = 10          # publish rate [Hz]

D1 = 1.7   # start depth [m]
T1 = 20   # duration D1→D2 [s]
D2 = 2.7   # intermediate depth [m]
T2 = 20   # duration D2→D3 [s]
D3 = 1.7   # final depth [m]

# ─────────────────────────────────────────────────────────────────────────────


def build_trajectory():
    """1-D depth trajectory: D1 → D2 → D3."""
    traj = []
    for d_start, d_end, duration in [(D1, D2, T1), (D2, D3, T2)]:
        steps = max(1, int(duration * PUBLISH_HZ))
        for i in range(steps):
            alpha = i / float(steps)
            depth = (1.0 - alpha) * d_start + alpha * d_end
            traj.append(depth)
    traj.append(D3)   # hold final point
    return traj


def plot_trajectory(traj):
    t = np.arange(len(traj)) / float(PUBLISH_HZ)

    _, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, traj, color="tab:blue", linewidth=1.5)
    ax.axvline(T1, color="gray", linestyle=":", linewidth=1)
    ax.text(T1 / 2,       min(D1, D2) - 0.05, f"D1→D2  ({T1}s)", ha="center", fontsize=9, color="gray")
    ax.text(T1 + T2 / 2,  min(D2, D3) - 0.05, f"D2→D3  ({T2}s)", ha="center", fontsize=9, color="gray")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Depth [m]  (+z down, NED)")
    ax.set_title(f"Depth trajectory  D1={D1} m → D2={D2} m → D3={D3} m")
    ax.grid(True, linestyle="--", alpha=0.5)
    plt.tight_layout()
    plt.show()




def run():
    traj = build_trajectory()
    plot_trajectory(traj)

    rospy.init_node("send_depth", anonymous=False)
    rate = rospy.Rate(PUBLISH_HZ)

    rospy.loginfo("Waiting for subscriber...")
    rospy.sleep(1.0)
    dyn = DynClient("/girona_controller", timeout=5.0)
    dyn.update_configuration({"enable_admittance": False, "enable_logging": True})
    rospy.loginfo("Admittance and logging enabled.")

    rospy.loginfo("Publishing")
    for depth in traj:
        if rospy.is_shutdown():
            break
        dyn.update_configuration({"desired_depth": depth})
        rate.sleep()

    rospy.loginfo("Trajectory finished.")

    dyn.update_configuration({"enable_admittance": False, "enable_logging": False})
    rospy.loginfo("Admittance and logging disabled.")


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
