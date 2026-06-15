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
INIT_X = -0.1
INIT_Y = 3.0
INIT_Z = 1.5
CONT_Y = 3.3 # move to this y, then we collide, this is toward the panel
#slide
DELT_x = 0.5
DELT_z = 0.5
POS_A = [INIT_X,  INIT_Y,  INIT_Z]  
POS_B = [INIT_X,  CONT_Y,  INIT_Z]

T = 30
POS_1 = [INIT_X,         CONT_Y,  INIT_Z]
POS_2 = [INIT_X+DELT_x,  CONT_Y,  INIT_Z]
POS_3 = [INIT_X+DELT_x,  CONT_Y,  INIT_Z+DELT_z]
POS_4 = [INIT_X,         CONT_Y,  INIT_Z+DELT_z]
POS_5 = [INIT_X,         CONT_Y,  INIT_Z]

QUAT_FIXED = [0.0, 0.0, -0.707, -0.707] 

SEG_DURATION = 30.0   # 每段时长 (s)
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
    """预先生成完整轨迹点列表，返回 list of [x, y, z]。"""
    steps = int(T * PUBLISH_HZ)
    segments = [(POS_1,POS_2),(POS_2,POS_3),
                (POS_3,POS_4),(POS_4,POS_5),
                (POS_5,POS_A)]
    traj = []
    for p_start, p_end in segments:
        p0 = np.array(p_start, dtype=float)
        p1 = np.array(p_end,   dtype=float)
        for i in range(steps):
            alpha = i / float(steps)
            traj.append(((1.0 - alpha) * p0 + alpha * p1).tolist())
    # traj.append(list(POS_D))  # 末尾终点
    return traj

# ── 修改这里 ──────────────────────────────────────────────────────────────────

# 第一段：常值力，持续 T1 秒
W_CONST = [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]   # [fx, fy, fz, tx, ty, tz]  N / N·m
T1      = 50.0                                # 第一段时长 (s)

# 第二段：正弦信号，每个频率持续恰好一个周期 (T = 1/f)
W_AMP     = [0.0, 3.0, 0.0, 0.0, 0.0, 0.0]              # 各轴幅值 (N / N·m)
FREQ_LIST = [0.02,0.04,0.06,0.08,0.10]  # Hz

PUBLISH_HZ = 10     # 发布频率 (Hz)
FRAME_ID   = "girona1000/base_link"
LABELS     = ["fx (N)", "fy (N)", "fz (N)", "tx (N·m)", "ty (N·m)", "tz (N·m)"]
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
    dt = 1.0 / PUBLISH_HZ
    pts_a = np.array(traj_approach)   # (Na, 3)
    pts_s = np.array(traj_slide)      # (Ns, 3)
    pts   = np.vstack([pts_a, pts_s])
    t     = np.arange(len(pts)) * dt
    t_boundary = len(pts_a) * dt

    labels = ["x [m]", "y [m]", "z [m]"]
    colors = ["tab:blue", "tab:orange", "tab:green"]
    waypoints = [POS_1, POS_2, POS_3, POS_4, POS_5]

    fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    fig.suptitle("Position trajectory preview", fontsize=13)
    for ax, col, label, color in zip(axes, range(3), labels, colors):
        ax.plot(t, pts[:, col], color=color, linewidth=1.5)
        ax.axvline(t_boundary, color="gray", linestyle="--", linewidth=1, label="approach / slide")
        ax.set_ylabel(label)
        ax.grid(True, linestyle="--", alpha=0.5)
    axes[0].legend(fontsize=8)
    axes[-1].set_xlabel("Time [s]")
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
    msg.header.frame_id = FRAME_ID
    msg.wrench.force.x  = float(wrench[0])
    msg.wrench.force.y  = float(wrench[1])
    msg.wrench.force.z  = float(wrench[2])
    msg.wrench.torque.x = float(wrench[3])
    msg.wrench.torque.y = float(wrench[4])
    msg.wrench.torque.z = float(wrench[5])
    return msg


def run():
    time, traj = build_force_trajectory()
    plot_trajectory(time, traj)

    rospy.init_node("send_force_trajectory", anonymous=False)
    pub_pos  = rospy.Publisher("/girona1000xh/ee_pose_cmd", PoseStamped, queue_size=10)
    pub  = rospy.Publisher("/girona1000xh/desired_wrench", WrenchStamped, queue_size=10)
    rate = rospy.Rate(PUBLISH_HZ)

    dyn = DynClient("/girona_controller", timeout=5.0)
    


    rospy.loginfo("Generating force trajectory...")
    time, traj = build_force_trajectory()
    total_dur = time[-1]
    rospy.loginfo("Trajectory ready: %d points, total %.2f s", len(traj), total_dur)

    plot_trajectory(time, traj)
    rospy.loginfo("Generating trajectory...")
    traj_approach = build_approaching_trajectory()

    traj_slide = build_sliding_trajectory()
    rospy.loginfo("Trajectory ready: %d points, total %.1f s",
                  len(traj), len(traj) / float(PUBLISH_HZ))
    plot_pos_trajectory(traj_approach, traj_slide)
    
    
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
