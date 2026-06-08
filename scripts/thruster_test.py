#!/usr/bin/env python3
"""
Thruster test node — manual single-channel or wrench-based thruster excitation.

Parameters (dynamic_reconfigure):
  force   [-20, 20] N   — value for linear  channels (1-3)
  torque  [-10, 10] Nm  — value for angular channels (4-6)
  channel [1, 6]        — target DOF/thruster index
  enable  bool          — when True, publish setpoints every callback
  use_tcm bool          — True: wrench → TCM^+ → thruster forces → setpoints
                          False: direct, value applied to channel-th thruster only

Polynomial model mirrors exp_thruster_model.h (girona_poly_nominal_model).
TCM loaded from config/control/tcm_with_thruster_torque.yaml.
Publishes to controller/passthrough_thruster_setpoints (cola2_msgs/Setpoints).
"""

import os
import yaml
import numpy as np
import rospy
from cola2_msgs.msg import Setpoints
from dynamic_reconfigure.server import Server
from sensorless_force_control.cfg import ThrusterTestConfig


# ── Polynomial model (mirrors exp_thruster_model.h) ─────────────────────────

_A = [0.0473235, 0.063145, -0.00256629, 5.2432528e-5, -5.0454768e-7, 1.842152e-9]
_B = [0.04997604, 0.05975017, -0.0022952, 4.39588388e-5, -3.9543457e-7, 1.34487e-9]


def force_to_setpoint(f: float) -> float:
    """Convert thruster force [N] to normalised setpoint via 5th-order polynomial."""
    if f >= 0.0:
        return sum(_A[i] * f**i for i in range(6))
    else:
        fn = -f
        return -sum(_B[i] * fn**i for i in range(6))


# ── TCM loader ───────────────────────────────────────────────────────────────

def load_tcm(yaml_path: str) -> np.ndarray:
    with open(yaml_path, "r") as fh:
        root = yaml.safe_load(fh)
    node = root["tcm"]
    rows, cols = node["rows"], node["cols"]
    data = node["data"]
    mat = np.array(data, dtype=float)
    if mat.shape != (rows, cols):
        raise ValueError(f"TCM shape mismatch: expected ({rows},{cols}), got {mat.shape}")
    return mat


# ── Main node ────────────────────────────────────────────────────────────────

class ThrusterTestNode:
    _TCM_DEFAULT = os.path.join(
        os.path.dirname(__file__),
        "../config/control/tcm_with_thruster_torque.yaml")

    def __init__(self):
        rospy.init_node("thruster_test", anonymous=False)

        # TCM
        tcm_path = rospy.get_param("~tcm_yaml", self._TCM_DEFAULT)
        try:
            tcm = load_tcm(tcm_path)
            _lam = 0.05
            _U, _s, _Vt = np.linalg.svd(tcm)
            self._tcm_inv = _Vt.T @ np.diag(_s / (_s**2 + _lam**2)) @ _U.T
            rospy.loginfo(f"[thruster_test] TCM loaded from {tcm_path}")
        except Exception as e:
            rospy.logwarn(f"[thruster_test] Could not load TCM: {e}. use_tcm will have no effect.")
            self._tcm_inv = None

        # Publisher
        topic = rospy.get_param(
            "~thruster_topic", "controller/passthrough_thruster_setpoints")
        self._pub = rospy.Publisher(topic, Setpoints, queue_size=1)
        rospy.loginfo(f"[thruster_test] ns='{rospy.get_namespace()}'  topic='{self._pub.resolved_name}'")

        # Current config — updated by reconfig callback, consumed by publish loop
        self._config = None

        # Dynamic reconfigure
        Server(ThrusterTestConfig, self._reconfig_cb)

        # 10 Hz publish loop
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._publish()
            rate.sleep()

    def _reconfig_cb(self, config, level):
        self._config = config
        return config

    def _publish(self):
        cfg = self._config
        if cfg is None or not cfg.enable:
            return

        ch = cfg.channel                             # 1-6

        if cfg.use_tcm and self._tcm_inv is not None:
            # Body wrench (force/torque) → TCM^+ → per-thruster forces
            value = cfg.force if ch <= 3 else cfg.torque
            wrench = np.zeros(6)
            wrench[ch - 1] = value
            thruster_forces = self._tcm_inv @ wrench
        else:
            # Direct: thruster_force applied straight to selected thruster
            value = cfg.thruster_force
            thruster_forces = np.zeros(6)
            thruster_forces[ch - 1] = value

        setpoints = [force_to_setpoint(f) for f in thruster_forces]

        msg = Setpoints()
        msg.header.stamp = rospy.Time.now()
        msg.setpoints = setpoints
        self._pub.publish(msg)

        rospy.loginfo_throttle(
            1.0,
            f"[thruster_test] ch={ch}  value={value:.3f}  "
            f"use_tcm={cfg.use_tcm}  "
            f"forces=[{', '.join(f'{v:.2f}' for v in thruster_forces)}]  "
            f"setpoints=[{', '.join(f'{v:.4f}' for v in setpoints)}]")


if __name__ == "__main__":
    try:
        ThrusterTestNode()
    except rospy.ROSInterruptException:
        pass
