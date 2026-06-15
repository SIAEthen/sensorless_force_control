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
INIT_X = -0.4
INIT_Y = 3.3
INIT_Z = 1.7
CONT_Y = 3.75  # move to this y, then we collide, this is toward the panel
POS_A = [INIT_X,  INIT_Y,  INIT_Z]
POS_B = [INIT_X,  CONT_Y,  INIT_Z]

RADIUS = 0.25      # circle radius [m]
N_LAPS = 1         # number of full circles
T_PER_LAP = 80     # duration of each full circle [s]

# Circle centre in xz-plane; start point (θ = -π/2) = [INIT_X, CONT_Y, INIT_Z]
CIRCLE_CX = INIT_X
CIRCLE_CZ = INIT_Z + RADIUS

QUAT_FIXED = [0.0, 0.0, -0.707, -0.707] 

SEG_DURATION = 25.0   # 每段时长 (s)
PUBLISH_HZ   = 10    # 发布频率 (Hz)
FRAME_ID     = "world_ned"


def build_approaching_trajectory():
    """预先生成完整轨迹点列表，返回 list of [x, y, z]。"""
    steps = int(SEG_DURATION * PUBLISH_HZ)
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
    """Circular trajectory in xz-plane at y=CONT_Y, then retract to POS_A."""
    # --- circular laps ---
    total_steps = int(T_PER_LAP * N_LAPS * PUBLISH_HZ)
    theta = np.linspace(-np.pi / 2,
                        -np.pi / 2 + 2 * np.pi * N_LAPS,
                        total_steps, endpoint=False)
    traj = [[CIRCLE_CX + RADIUS * np.cos(th),
             CONT_Y,
             CIRCLE_CZ + RADIUS * np.sin(th)] for th in theta]

    # --- retract: last circle point → POS_A ---
    retract_steps = int(SEG_DURATION * PUBLISH_HZ)
    p_last = np.array(traj[-1])
    p_home = np.array(POS_A)
    for i in range(retract_steps):
        alpha = i / float(retract_steps)
        traj.append(((1.0 - alpha) * p_last + alpha * p_home).tolist())

    return traj

# ── 修改这里 ──────────────────────────────────────────────────────────────────

# 第一段：常值力，持续 T1 秒
W_CONST = [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]   # [fx, fy, fz, tx, ty, tz]  N / N·m
T1      = 50.0                                # 第一段时长 (s)

# 第二段：正弦信号，每个频率持续恰好一个周期 (T = 1/f)
W_AMP     = [0.0, 3.0, 0.0, 0.0, 0.0, 0.0]              # 各轴幅值 (N / N·m)
FREQ_LIST = [0.02,0.04,0.06,0.08,0.10]  # Hz

FRAME_ID_WRENCH = "girona1000/base_link"
LABELS          = ["fx (N)", "fy (N)", "fz (N)", "tx (N·m)", "ty (N·m)", "tz (N·m)"]
# ─────────────────────────────────────────────────────────────────────────────


def build_force_trajectory():
    """预先生成完整力轨迹，返回 (time_array, traj_array)。
    traj_array shape: (N, 6)
    """
    dt    = 1.0 / PUBLISH_HZ
    const = np.array(W_CONST, dtype=float)
    rows  = []

    # 第一段：常值
    steps1 = int(T1 * PUBLISH_HZ)
    for _ in range(steps1):
        rows.append(const.copy())

    rows.append(const) 

    traj = np.array(rows)
    time = np.arange(len(traj)) * dt
    return time, traj


def plot_trajectory(time, traj):
    fig, axes = plt.subplots(6, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("Desired Wrench Trajectory", fontsize=13)

    for i, (ax, label) in enumerate(zip(axes, LABELS)):
        ax.plot(time, traj[:, i], linewidth=1.2)
        ax.set_ylabel(label, fontsize=9)
        ax.axvline(T1, color="gray", linestyle="--", linewidth=0.8, label="seg2 start" if i == 0 else None)
        ax.grid(True, linewidth=0.4)

    axes[0].legend(fontsize=8)
    axes[-1].set_xlabel("Time (s)", fontsize=10)
    plt.tight_layout()
    plt.show()

def plot_pos_trajectory(traj_approach, traj_slide):
    dt    = 1.0 / PUBLISH_HZ
    pts_a = np.array(traj_approach)          # (Na, 3)
    pts_s = np.array(traj_slide)             # (Ns, 3)
    pts   = np.vstack([pts_a, pts_s])
    t     = np.arange(len(pts)) * dt
    t_boundary  = len(pts_a) * dt
    t_retract   = t_boundary + T_PER_LAP * N_LAPS

    fig = plt.figure(figsize=(14, 7))
    fig.suptitle("Position trajectory preview (circular)", fontsize=13)

    # ── left: xz-plane top view ──────────────────────────────────────────────
    ax_xz = fig.add_subplot(1, 2, 1, aspect="equal")
    ax_xz.plot(pts_a[:, 0], pts_a[:, 2], color="tab:gray",
               linewidth=1.5, linestyle="--", label="approach")
    ax_xz.plot(pts_s[:, 0], pts_s[:, 2], color="tab:blue",
               linewidth=1.5, label="circle + retract")
    # reference circle
    th_ref = np.linspace(0, 2 * np.pi, 200)
    ax_xz.plot(CIRCLE_CX + RADIUS * np.cos(th_ref),
               CIRCLE_CZ + RADIUS * np.sin(th_ref),
               color="tab:red", linestyle=":", linewidth=1, label="ideal circle")
    ax_xz.plot(CIRCLE_CX, CIRCLE_CZ, "r+", markersize=8, label="centre")
    ax_xz.plot(pts_s[0, 0], pts_s[0, 2], "go", markersize=7, label="start")
    ax_xz.set_xlabel("x [m]")
    ax_xz.set_ylabel("z [m]")
    ax_xz.set_title("xz-plane  (contact surface)")
    ax_xz.legend(fontsize=8)
    ax_xz.grid(True, linestyle="--", alpha=0.5)

    # ── right: x / y / z vs time ─────────────────────────────────────────────
    labels = ["x [m]", "y [m]", "z [m]"]
    colors = ["tab:blue", "tab:orange", "tab:green"]
    axes_t = []
    for row, (label, color) in enumerate(zip(labels, colors)):
        ax = fig.add_subplot(3, 2, 2 * (row + 1), sharex=axes_t[0] if axes_t else None)
        ax.plot(t, pts[:, row], color=color, linewidth=1.2)
        ax.axvline(t_boundary, color="gray",    linestyle="--", linewidth=1,
                   label="start circle" if row == 0 else None)
        ax.axvline(t_retract,  color="salmon",  linestyle="--", linewidth=1,
                   label="start retract" if row == 0 else None)
        ax.set_ylabel(label)
        ax.grid(True, linestyle="--", alpha=0.5)
        axes_t.append(ax)
    axes_t[0].legend(fontsize=8)
    axes_t[-1].set_xlabel("Time [s]")

    plt.tight_layout()
    plt.show()


def make_msg_pos(pos):
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


def make_msg(wrench):
    msg = WrenchStamped()
    msg.header.stamp    = rospy.Time.now()
    msg.header.frame_id = FRAME_ID_WRENCH
    msg.wrench.force.x  = float(wrench[0])
    msg.wrench.force.y  = float(wrench[1])
    msg.wrench.force.z  = float(wrench[2])
    msg.wrench.torque.x = float(wrench[3])
    msg.wrench.torque.y = float(wrench[4])
    msg.wrench.torque.z = float(wrench[5])
    return msg


def run():
    # build & plot everything before rospy.init_node() to avoid Tkinter thread crash
    time, traj    = build_force_trajectory()
    traj_approach = build_approaching_trajectory()
    traj_slide    = build_sliding_trajectory()
    plot_trajectory(time, traj)
    plot_pos_trajectory(traj_approach, traj_slide)

    rospy.init_node("send_force_trajectory", anonymous=False)
    pub_pos = rospy.Publisher("/girona1000xh/ee_pose_cmd",    PoseStamped,   queue_size=10)
    pub     = rospy.Publisher("/girona1000xh/desired_wrench", WrenchStamped, queue_size=10)
    rate    = rospy.Rate(PUBLISH_HZ)
    dyn     = DynClient("/girona_controller", timeout=5.0)

    rospy.loginfo("Force traj: %d pts (%.1f s) | Approach: %d pts | Slide: %d pts",
                  len(traj), time[-1], len(traj_approach), len(traj_slide))
    dyn.update_configuration({"enable_admittance": True, "enable_logging": True})
    rospy.loginfo("Publishing pos trajectory...")
    for pos in traj_approach:
        if rospy.is_shutdown():
            break
        pub_pos.publish(make_msg_pos(pos))
        rate.sleep()
    rospy.loginfo("APPROACH POS Trajectory finished.")


    rospy.loginfo("Publishing force trajectory...")
    seg2_idx  = int(T1 * PUBLISH_HZ)
    freq_steps = [max(1, int((1.0 / f) * PUBLISH_HZ)) for f in FREQ_LIST]
    boundaries = [seg2_idx + sum(freq_steps[:k]) for k in range(len(FREQ_LIST))]

    for idx, wrench in enumerate(traj):
        if rospy.is_shutdown():
            break
        if idx in boundaries:
            fi = boundaries.index(idx)
            rospy.loginfo("Seg2 [%d/%d]: freq = %.1f Hz, period = %.2f s",
                          fi + 1, len(FREQ_LIST), FREQ_LIST[fi], 1.0 / FREQ_LIST[fi])
        pub.publish(make_msg(wrench))
        rate.sleep()

    rospy.loginfo("Force trajectory finished.")

    for pos in traj_slide:
        if rospy.is_shutdown():
            break
        pub_pos.publish(make_msg_pos(pos))
        rate.sleep()
    rospy.loginfo("SLIDING POS Trajectory finished.")



    dyn.update_configuration({"enable_admittance": False, "enable_logging": False})
    rospy.loginfo("Logging disabled.")


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
