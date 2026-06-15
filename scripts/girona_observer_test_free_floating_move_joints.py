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

Q1 = [ 0.0, 0.0,  0.0,  0.0, -1.579,  0.0]
Q2 = [ 0.8, 0.3, -0.3,  0.6, -1.579,  0.8]
Q3 = [ 0.0, 0.0,  0.0,  0.0, -1.579,  0.0]
Q4 = [-0.8, -0.3, 0.3, -0.6, -1.579, -0.8]
Q5 = [ 0.0, 0.0,  0.0,  0.0, -1.579,  0.0]
T = 15
# ─────────────────────────────────────────────────────────────────────────────


WAYPOINTS = [Q1, Q2, Q3, Q4, Q5]   # sequence of joint configurations
JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]


def build_trajectory():
    """Linear interpolation Q1→Q2→Q3→Q4→Q5, each segment T seconds."""
    traj = []
    for q_start, q_end in zip(WAYPOINTS[:-1], WAYPOINTS[1:]):
        q_s = np.array(q_start)
        q_e = np.array(q_end)
        steps = max(1, int(T * PUBLISH_HZ))
        for i in range(steps):
            alpha = i / float(steps)
            traj.append((1.0 - alpha) * q_s + alpha * q_e)
    traj.append(np.array(WAYPOINTS[-1]))   # hold final config
    return traj


def plot_trajectory(traj):
    pts = np.array(traj)                           # (N, 6)
    t   = np.arange(len(pts)) / float(PUBLISH_HZ)
    n_seg = len(WAYPOINTS) - 1
    boundaries = [i * T for i in range(n_seg + 1)]
    seg_labels = [f"Q{i+1}→Q{i+2}" for i in range(n_seg)]
    colors = plt.cm.tab10(np.linspace(0, 0.6, 6))

    _, ax = plt.subplots(figsize=(12, 5))
    for j in range(6):
        ax.plot(t, pts[:, j], label=JOINT_NAMES[j], color=colors[j], linewidth=1.5)
    for tb in boundaries[1:-1]:
        ax.axvline(tb, color="gray", linestyle=":", linewidth=1)
    for i, label in enumerate(seg_labels):
        mid = (boundaries[i] + boundaries[i + 1]) / 2.0
        ax.text(mid, ax.get_ylim()[1] if ax.get_ylim()[1] != 1.0 else 0.5,
                label, ha="center", fontsize=8, color="gray")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Joint angle [rad]")
    ax.set_title(f"Joint trajectory preview  (T={T}s per segment)")
    ax.legend(loc="upper right", fontsize=8)
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
    for q in traj:
        if rospy.is_shutdown():
            break
        dyn.update_configuration({"nominal_config_1": q[0],
                                  "nominal_config_2": q[1],
                                  "nominal_config_3": q[2],
                                  "nominal_config_4": q[3],
                                  "nominal_config_5": q[4],
                                  "nominal_config_6": q[5]})
        rate.sleep()

    rospy.loginfo("Trajectory finished.")
    
    dyn.update_configuration({"enable_admittance": False, "enable_logging": False})
    rospy.loginfo("Admittance and logging disabled.")


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
