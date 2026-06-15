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

# End-effector waypoints [x, y, z] in NED frame
A  = np.array([0.0,  0.0,  0.0])   # home / starting pose
B  = np.array([0.03, 0.0,  0.0])   # push forward +x
B1 = np.array([0.03, 0.0,  0.0])   # push forward +x
C  = np.array([0.15,  0.0,  0.0])   # slide sideways
D  = np.array([0.04,  0.0,  0.0])   # return to home
E  = np.array([0.15,  0.0,  0.0]) 

F  = np.array([0.0,  0.0,  0.0]) 


# Duration [s] for each segment — edit independently
SEG_DURATIONS = {
    "1A->B" : 10.0,
    "2B->B1": 100.0,
    "3B1->C": 80.0,
    "4C->C":  50.0,
}

# ─────────────────────────────────────────────────────────────────────────────


def build_trajectory():
    """Linear interpolation A→B→C→D with per-segment durations."""
    segments = [("1A->B", A, B),
                ("2B->B1", B, B1), 
                ("3B1->C", B1, C),
                ("4C->C",C,C)]
    traj = []
    for name, start, end in segments:
        steps = max(1, int(SEG_DURATIONS[name] * PUBLISH_HZ))
        for i in range(steps):
            alpha = i / float(steps)
            traj.append((1.0 - alpha) * start + alpha * end)
    traj.append(D.copy())   # hold final point for one extra sample
    return traj


def make_msg(pos):
    msg = Twist()
    msg.linear.x = float(pos[0])
    msg.linear.y = float(pos[1])
    msg.linear.z = float(pos[2])
    return msg


def plot_trajectory(traj):
    """Plot x/y/z commands vs time so you can verify the trajectory before sending."""
    pts = np.array(traj)                          # (N, 3)
    t   = np.arange(len(pts)) / float(PUBLISH_HZ) # time axis [s]

    # Segment boundary times for vertical reference lines
    boundaries = [0.0]
    for dur in SEG_DURATIONS.values():
        boundaries.append(boundaries[-1] + dur)

    labels = ["x (forward)", "y (lateral)", "z (depth)"]
    colors = ["tab:blue", "tab:orange", "tab:green"]

    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    fig.suptitle("Hand-cmd trajectory preview", fontsize=13)

    for ax, col, label, color in zip(axes, range(3), labels, colors):
        ax.plot(t, pts[:, col], color=color, linewidth=1.5)
        ax.set_ylabel(f"{label} [m]")
        ax.grid(True, linestyle="--", alpha=0.5)
        for tb in boundaries[1:-1]:          # skip start and end
            ax.axvline(tb, color="gray", linestyle=":", linewidth=1)

    # Annotate segment names
    seg_names = list(SEG_DURATIONS.keys())
    for i, name in enumerate(seg_names):
        mid = (boundaries[i] + boundaries[i + 1]) / 2.0
        axes[0].text(mid, axes[0].get_ylim()[1], name,
                     ha="center", va="bottom", fontsize=9, color="gray")

    axes[-1].set_xlabel("Time [s]")
    plt.tight_layout()
    plt.show()


def run():
    traj = build_trajectory()
    plot_trajectory(traj)

    rospy.init_node("send_hand_cmd", anonymous=False)
    pub  = rospy.Publisher("/girona1000xh/hand_cmd", Twist, queue_size=10)
    rate = rospy.Rate(PUBLISH_HZ)

    rospy.loginfo("Generating trajectory...")
    traj = build_trajectory()
    rospy.loginfo("Trajectory ready: %d points, total %.1f s",
                  len(traj), len(traj) / float(PUBLISH_HZ))
    plot_trajectory(traj)

    rospy.loginfo("Waiting for subscriber...")
    rospy.sleep(1.0)

    pub.publish(make_msg(traj[0]))
    rospy.sleep(1)
    dyn = DynClient("/girona_controller", timeout=5.0)
    dyn.update_configuration({"enable_admittance": False, "enable_logging": True})
    rospy.loginfo("Admittance and logging enabled.")

    rospy.loginfo("Publishing: A -> B -> C -> D")
    for pos in traj:
        if rospy.is_shutdown():
            break
        pub.publish(make_msg(pos))
        rate.sleep()

    rospy.loginfo("Trajectory finished.")
    rospy.loginfo("Publishing: 10 times zero velocity")
    for i in range(10):
        pub.publish(make_msg(np.zeros(3)))
        rate.sleep()
    

    dyn.update_configuration({"enable_admittance": False, "enable_logging": False})
    rospy.loginfo("Admittance and logging disabled.")


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
