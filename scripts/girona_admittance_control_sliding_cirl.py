#!/usr/bin/env python3
"""
Publish a two-segment desired wrench trajectory:
  Segment 1 : constant wrench, duration T1 (s)
  Segment 2 : sinusoidal wrench, each frequency in FREQ_LIST runs for exactly
              one period (T = 1/f s), then moves to the next frequency.

Topic : desired_wrench  (geometry_msgs/WrenchStamped)

Usage:
    rosrun sensorless_force_control send_force_trajectory.py
"""

import numpy as np
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import PoseStamped
from dynamic_reconfigure.client import Client as DynClient
from utilts_uvms_math import Rpy2Rot
# 
DEPTH_0 = 1.8
X_0 = -0.5
Y_0 = 3.5
DELTA_X = 0.5
DELTA_Y = 0.4
DELTA_Z = 0.5
T_1   = 25
POS_A = [X_0,  Y_0,  DEPTH_0]
POS_B = [X_0,  Y_0+DELTA_Y,  DEPTH_0]



RADIUS    = 0.35   # circle radius [m]
N_LAPS    = 1      # number of full circles
T_PER_LAP = 60     # duration per lap [s]

CONT_Y    = POS_B[1]                   # y stays constant during circle (contact surface)
CIRCLE_CX = POS_B[0]                   # circle centre x
CIRCLE_CZ = POS_B[2] + RADIUS         # circle centre z; start point (θ=-π/2) = POS_B

T_3 = 25
POS_a = POS_B
POS_b = POS_A

QUAT_FIXED = [0.0, 0.0, -0.707, -0.707] 
PUBLISH_HZ   = 10    # 发布频率 (Hz)
FRAME_ID     = "world_ned"

def build_approaching_trajectory():
    """预先生成完整轨迹点列表，返回 list of [x, y, z]。"""
    steps = int(T_1 * PUBLISH_HZ)
    segments = [(POS_A, POS_A),(POS_A, POS_B)]
    traj = []
    for p_start, p_end in segments:
        p0 = np.array(p_start, dtype=float)
        p1 = np.array(p_end,   dtype=float)
        for i in range(steps):
            alpha = i / float(steps)
            traj.append(((1.0 - alpha) * p0 + alpha * p1).tolist())
    # traj.append(list(POS_D))  # 末尾终点
    return traj

def build_sliding_trajectory():
    """Circular trajectory in xz-plane at y=CONT_Y."""
    total_steps = int(T_PER_LAP * N_LAPS * PUBLISH_HZ)
    theta = np.linspace(-np.pi / 2,
                        -np.pi / 2 + 2 * np.pi * N_LAPS,
                        total_steps, endpoint=False)
    return [[CIRCLE_CX + RADIUS * np.cos(th),
             CONT_Y,
             CIRCLE_CZ + RADIUS * np.sin(th)] for th in theta]

def build_releasing_trajectory():
    """预先生成完整轨迹点列表，返回 list of [x, y, z]。"""
    steps = int(T_3 * PUBLISH_HZ)
    segments = [(POS_a,POS_b)]
    traj = []
    for p_start, p_end in segments:
        p0 = np.array(p_start, dtype=float)
        p1 = np.array(p_end,   dtype=float)
        for i in range(steps):
            alpha = i / float(steps)
            traj.append(((1.0 - alpha) * p0 + alpha * p1).tolist())
    # traj.append(list(POS_D))  # 末尾终点
    return traj

def plot_pos_trajectory(traj_approach, traj_slide, traj_release):
    dt    = 1.0 / PUBLISH_HZ
    pts_a = np.array(traj_approach)
    pts_s = np.array(traj_slide)
    pts_r = np.array(traj_release)
    pts   = np.vstack([pts_a, pts_s, pts_r])
    t     = np.arange(len(pts)) * dt
    t_slide   = len(pts_a) * dt
    t_release = t_slide + len(pts_s) * dt

    fig = plt.figure(figsize=(14, 7))
    fig.suptitle("Admittance sliding trajectory preview (circle)", fontsize=13)

    # xz-plane view
    ax_xz = fig.add_subplot(1, 2, 1, aspect="equal")
    ax_xz.plot(pts_a[:, 0], pts_a[:, 2], color="tab:gray",   linestyle="--", linewidth=1.5, label="approach")
    ax_xz.plot(pts_s[:, 0], pts_s[:, 2], color="tab:blue",   linewidth=1.5,                label="circle")
    ax_xz.plot(pts_r[:, 0], pts_r[:, 2], color="tab:orange", linestyle="--", linewidth=1.5, label="release")
    th_ref = np.linspace(0, 2 * np.pi, 300)
    ax_xz.plot(CIRCLE_CX + RADIUS * np.cos(th_ref),
               CIRCLE_CZ + RADIUS * np.sin(th_ref),
               color="tab:red", linestyle=":", linewidth=1, label="ideal circle")
    ax_xz.plot(CIRCLE_CX, CIRCLE_CZ, "r+", markersize=8, label="centre")
    ax_xz.plot(pts_s[0, 0], pts_s[0, 2], "go", markersize=7, label="start")
    ax_xz.set_xlabel("x [m]"); ax_xz.set_ylabel("z [m]")
    ax_xz.set_title("xz-plane (contact surface)"); ax_xz.legend(fontsize=8)
    ax_xz.grid(True, linestyle="--", alpha=0.5)

    # x/y/z vs time
    labels = ["x [m]", "y [m]", "z [m]"]
    colors = ["tab:blue", "tab:orange", "tab:green"]
    axes_t = []
    for row, (label, color) in enumerate(zip(labels, colors)):
        ax = fig.add_subplot(3, 2, 2 * (row + 1), sharex=axes_t[0] if axes_t else None)
        ax.plot(t, pts[:, row], color=color, linewidth=1.2)
        ax.axvline(t_slide,   color="gray",   linestyle="--", linewidth=1,
                   label="start circle"  if row == 0 else None)
        ax.axvline(t_release, color="salmon", linestyle="--", linewidth=1,
                   label="start release" if row == 0 else None)
        ax.set_ylabel(label); ax.grid(True, linestyle="--", alpha=0.5)
        axes_t.append(ax)
    axes_t[0].legend(fontsize=8)
    axes_t[-1].set_xlabel("Time [s]")
    plt.tight_layout()
    plt.show()


def make_msg(pos):
    q = np.array(QUAT_FIXED, dtype=float)
    q /= np.linalg.norm(q)
    msg = PoseStamped()
    msg.header.stamp    = rospy.Time.now()
    msg.header.frame_id = FRAME_ID
    msg.pose.position.x    = pos[0]
    msg.pose.position.y    = pos[1]
    msg.pose.position.z    = pos[2]
    msg.pose.orientation.w = q[0]
    msg.pose.orientation.x = q[1]
    msg.pose.orientation.y = q[2]
    msg.pose.orientation.z = q[3]
    return msg
# ── 修改这里 ──────────────────────────────────────────────────────────────────

def run():
    # build & plot before rospy.init_node() to avoid Tkinter thread crash
    traj_approach = build_approaching_trajectory()
    traj_slide    = build_sliding_trajectory()
    traj_release  = build_releasing_trajectory()
    plot_pos_trajectory(traj_approach, traj_slide, traj_release)

    rospy.init_node("send_trajectory", anonymous=False)
    pub_pos = rospy.Publisher("/girona1000xh/ee_pose_cmd", PoseStamped, queue_size=10)
    rate    = rospy.Rate(PUBLISH_HZ)
    dyn     = DynClient("/girona_controller", timeout=5.0)
    rospy.loginfo("Trajectories ready — approach: %d pts, circle: %d pts, release: %d pts",
                  len(traj_approach), len(traj_slide), len(traj_release))
    
    dyn.update_configuration({"enable_admittance": True, "enable_logging": True})
    rospy.loginfo("Publishing pos trajectory...")
    for pos in traj_approach:
        if rospy.is_shutdown():
            break
        pub_pos.publish(make_msg(pos))
        rate.sleep()
    rospy.loginfo("APPROACH POS Trajectory finished.")

    for pos in traj_slide:
        if rospy.is_shutdown():
            break
        pub_pos.publish(make_msg(pos))
        rate.sleep()
    rospy.loginfo("SLIDING POS Trajectory finished.")

    for pos in traj_release:
        if rospy.is_shutdown():
            break
        pub_pos.publish(make_msg(pos))
        rate.sleep()
    rospy.loginfo("Relasing POS Trajectory finished.")



    dyn.update_configuration({"enable_admittance": False,"enable_logging": False})
    rospy.loginfo("Logging disabled.")


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
