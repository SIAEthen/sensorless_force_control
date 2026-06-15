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

from dynamic_reconfigure.client import Client as DynClient




def run():
    rospy.init_node("hqp_comparison_scenario1", anonymous=False)
    rospy.loginfo("Waiting for subscriber...")
    rospy.sleep(1.0)

    dyn = DynClient("/girona_controller", timeout=5.0)

    # 1. 开 logging
    dyn.update_configuration({"enable_logging": True})
    rospy.loginfo("Logging enabled.")
    rospy.sleep(5.0)

    # 2. 关掉 EE task，等 30s
    dyn.update_configuration({"enable_ee_task": False})
    rospy.loginfo("EE task disabled. Waiting 50 s ...")
    rospy.sleep(40.0)

    # 3. 重新激活 EE task，等 30s
    dyn.update_configuration({"enable_ee_task": True})
    rospy.loginfo("EE task enabled. Waiting 50 s ...")
    rospy.sleep(40.0)

    # 4. 关 logging
    dyn.update_configuration({"enable_logging": False})
    rospy.loginfo("Logging disabled. Done.")



if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
