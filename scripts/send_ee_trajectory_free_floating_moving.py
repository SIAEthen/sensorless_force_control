#!/usr/bin/env python3
"""
Publish a constant-velocity end-effector trajectory.
The EE moves from POS_START at (VEL_X, VEL_Y, VEL_Z) for T seconds.
Orientation is fixed throughout.

Usage:
    rosrun sensorless_force_control send_ee_trajectory_free_floating_moving.py
"""

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from dynamic_reconfigure.client import Client as DynClient

# ── 修改这里 ──────────────────────────────────────────────────────────────────
POS_START = [0.0,  0.0,  2.0]   # 起始位置 [x, y, z] (NED, m)

VEL_X =  0.06   # x 轴速度 (m/s)
VEL_X =  0.04   # x 轴速度 (m/s)
VEL_X =  0.02   # x 轴速度 (m/s)
VEL_X =  0.00   # x 轴速度 (m/s)

# VEL_Z =  0.06   # x 轴速度 (m/s)
# VEL_Z =  0.04   # x 轴速度 (m/s)
# VEL_Z =  0.02   # x 轴速度 (m/s)
VEL_Z =  0.0    # z 轴速度 (m/s)

VEL_Y =  0.06   # y 轴速度 (m/s)
VEL_Y =  0.04   # y 轴速度 (m/s)
VEL_Y =  0.02   # y 轴速度 (m/s)
VEL_Y =  0.00   # y 轴速度 (m/s)


T          = 40.0   # 运动总时长 (s)
PUBLISH_HZ = 10     # 发布频率 (Hz)

QUAT_FIXED = [0.0, 0.0, -0.707, -0.707]  # [w, x, y, z]，固定姿态
FRAME_ID   = "world_ned"
# ─────────────────────────────────────────────────────────────────────────────


def build_trajectory():
    """预先生成完整轨迹点列表，返回 list of [x, y, z]。"""
    dt    = 1.0 / PUBLISH_HZ
    steps = int(T * PUBLISH_HZ)
    vel   = np.array([VEL_X, VEL_Y, VEL_Z], dtype=float)
    p0    = np.array(POS_START, dtype=float)
    traj  = []
    for i in range(steps + 1):
        traj.append((p0 + vel * i * dt).tolist())
    return traj


def make_msg(pos):
    q = np.array(QUAT_FIXED, dtype=float)
    q /= np.linalg.norm(q)
    msg = PoseStamped()
    msg.header.stamp       = rospy.Time.now()
    msg.header.frame_id    = FRAME_ID
    msg.pose.position.x    = pos[0]
    msg.pose.position.y    = pos[1]
    msg.pose.position.z    = pos[2]
    msg.pose.orientation.w = q[0]
    msg.pose.orientation.x = q[1]
    msg.pose.orientation.y = q[2]
    msg.pose.orientation.z = q[3]
    return msg


def run():
    rospy.init_node("send_ee_trajectory_free_floating_moving", anonymous=False)
    pub  = rospy.Publisher("/girona1000xh/ee_pose_cmd", PoseStamped, queue_size=10)
    rate = rospy.Rate(PUBLISH_HZ)

    rospy.loginfo("Generating trajectory: vel=[%.3f, %.3f, %.3f] m/s, T=%.1f s",
                  VEL_X, VEL_Y, VEL_Z, T)
    traj = build_trajectory()
    pos_end = traj[-1]
    rospy.loginfo("Start: [%.3f, %.3f, %.3f]  End: [%.3f, %.3f, %.3f]  Points: %d",
                  POS_START[0], POS_START[1], POS_START[2],
                  pos_end[0],   pos_end[1],   pos_end[2],   len(traj))

    rospy.loginfo("Waiting for subscriber...")
    rospy.sleep(1.0)

    dyn = DynClient("/girona_controller", timeout=5.0)
    dyn.update_configuration({"enable_admittance": False, "enable_logging": True})
    rospy.loginfo("Admittance and logging enabled.")
    rospy.sleep(5.0)

    rospy.loginfo("Publishing trajectory...")
    for pos in traj:
        if rospy.is_shutdown():
            break
        pub.publish(make_msg(pos))
        rate.sleep()

    rospy.loginfo("Trajectory finished.")

    dyn.update_configuration({"enable_admittance": False, "enable_logging": False})
    rospy.loginfo("Admittance and logging disabled.")
    
    pub.publish(make_msg(POS_START))



if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
