#!/usr/bin/env python3
"""
Publish a 3-segment end-effector position trajectory.
Trajectory: A -> B (5s) -> C (5s) -> D (5s)
Orientation fixed throughout.

Usage:
    rosrun sensorless_force_control send_ee_trajectory.py
"""

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from dynamic_reconfigure.client import Client as DynClient

# ── 修改这里定义轨迹点 (NED frame, 单位: m) ─────────────────────────────────
POS_A = [0.3,  3.6,  2.0]
POS_B = [0.3,  3.8,  2.0]
POS_C = [0.1,  3.8,  2.0]
POS_D = [0.1,  3.6,  2.0]
POS_E = [0.1,  3.6,  2.0]

QUAT_FIXED = [0.0, 0.0, -0.707, -0.707]  # [w, x, y, z]，整段轨迹姿态不变

SEG_DURATION = 30.0   # 每段时长 (s)
PUBLISH_HZ   = 10    # 发布频率 (Hz)
FRAME_ID     = "world_ned"
# ─────────────────────────────────────────────────────────────────────────────


def build_trajectory():
    """预先生成完整轨迹点列表，返回 list of [x, y, z]。"""
    steps = int(SEG_DURATION * PUBLISH_HZ)
    segments = [(POS_A, POS_B), (POS_B, POS_C), (POS_C, POS_D), (POS_D, POS_E)]
    traj = []
    for p_start, p_end in segments:
        p0 = np.array(p_start, dtype=float)
        p1 = np.array(p_end,   dtype=float)
        for i in range(steps):
            alpha = i / float(steps)
            traj.append(((1.0 - alpha) * p0 + alpha * p1).tolist())
    traj.append(list(POS_D))  # 末尾终点
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


def run():
    rospy.init_node("send_ee_trajectory", anonymous=False)
    pub  = rospy.Publisher("/girona1000xh/ee_pose_cmd", PoseStamped, queue_size=10)
    rate = rospy.Rate(PUBLISH_HZ)

    rospy.loginfo("Generating trajectory...")
    traj = build_trajectory()
    rospy.loginfo("Trajectory ready: %d points, total %.1f s",
                  len(traj), len(traj) / float(PUBLISH_HZ))

    rospy.loginfo("Waiting for subscriber...")
    rospy.sleep(1.0)

    dyn = DynClient("/girona_controller", timeout=5.0)
    dyn.update_configuration({"enable_admittance": True, "enable_logging": True})
    rospy.loginfo("Admittance and logging enabled.")
    rospy.sleep(5)

    rospy.loginfo("Publishing: A -> B -> C -> D")
    for pos in traj:
        if rospy.is_shutdown():
            break
        pub.publish(make_msg(pos))
        rate.sleep()

    rospy.loginfo("Trajectory finished.")

    dyn.update_configuration({"enable_admittance": False, "enable_logging": False})
    rospy.loginfo("Admittance and logging disabled.")
    pub.publish(make_msg(POS_A))

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
