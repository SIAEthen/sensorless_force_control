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
X_0 = -1.2
Y_0 = 3.6
DELTA_X = 0.5
DELTA_Y = 0.3
DELTA_Z = 0.5
T_1   = 40
POS_A = [X_0,  Y_0,  DEPTH_0]
POS_B = [X_0,  Y_0 + DELTA_Y, DEPTH_0]

T_2 = 40
POS_1 = [X_0,  Y_0 + DELTA_Y,  DEPTH_0]
POS_2 = [X_0+DELTA_X,  Y_0 + DELTA_Y,  DEPTH_0]
POS_3 = [X_0+DELTA_X,  Y_0 + DELTA_Y,  DEPTH_0+ DELTA_Z]
POS_4 = [X_0,  Y_0 + DELTA_Y,  DEPTH_0+ DELTA_Z]
POS_5 = [X_0,  Y_0 + DELTA_Y,  DEPTH_0]

T_3 = 50
POS_a = [X_0,  Y_0 + DELTA_Y,  DEPTH_0]
POS_b = [X_0,  Y_0,  DEPTH_0]

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
    """预先生成完整轨迹点列表，返回 list of [x, y, z]。"""
    steps = int(T_2 * PUBLISH_HZ)
    segments = [(POS_1,POS_2),(POS_2,POS_3),(POS_3,POS_4),(POS_4,POS_5)]
    traj = []
    for p_start, p_end in segments:
        p0 = np.array(p_start, dtype=float)
        p1 = np.array(p_end,   dtype=float)
        for i in range(steps):
            alpha = i / float(steps)
            traj.append(((1.0 - alpha) * p0 + alpha * p1).tolist())
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

    labels = ["x [m]", "y [m]", "z [m]"]
    colors = ["tab:blue", "tab:orange", "tab:green"]
    _, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    plt.suptitle("Position trajectory preview (rectangle)", fontsize=13)
    for ax, col, label, color in zip(axes, range(3), labels, colors):
        ax.plot(t, pts[:, col], color=color, linewidth=1.5)
        ax.axvline(t_slide,   color="gray",   linestyle="--", linewidth=1,
                   label="start slide"   if col == 0 else None)
        ax.axvline(t_release, color="salmon", linestyle="--", linewidth=1,
                   label="start release" if col == 0 else None)
        ax.set_ylabel(label)
        ax.grid(True, linestyle="--", alpha=0.5)
    axes[0].legend(fontsize=8)
    axes[-1].set_xlabel("Time [s]")
    plt.tight_layout()
    plt.show()

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
    rospy.loginfo("Trajectories ready — approach: %d pts, slide: %d pts, release: %d pts",
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



    dyn.update_configuration({"enable_admittance": False, "enable_logging": False})
    rospy.loginfo("Logging disabled.")


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
