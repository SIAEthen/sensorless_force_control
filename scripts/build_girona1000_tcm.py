#!/usr/bin/env python3

import re
from pathlib import Path
from typing import Dict, List, Tuple
import yaml
import numpy as np
from utilts_uvms_math import *
from utilts_task_priority_control import *

def print_setpoints_ros(setpoints):
    """
    Print setpoints in rostopic pub YAML style.
    setpoints: iterable of floats (e.g. list or np.array)
    """
    sp_str = ", ".join(f"{x:.6f}" for x in setpoints)

    msg = f'''rostopic pub -r 20 /girona1000/controller/passthrough_thruster_setpoints cola2_msgs/Setpoints "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
setpoints: [{sp_str}]"
'''
    
    print(msg)

def read_thruster_origins(scn_path: str) -> Dict[str, np.ndarray]:
  """Read Thruster1..Thruster6 origin rpy/xyz from a .scn file.

  Returns a dict:
    {
      "Thruster1": np.array([r, p, y, x, y, z]),
      ...
    }
  """
  text = Path(scn_path).read_text(encoding="utf-8")

  thrusters: Dict[str, np.ndarray] = {}
  for i in range(1, 7):
    name = f"Thruster{i}"
    # Match actuator block for this thruster
    block_match = re.search(
        rf"<actuator\s+name=\"{name}\"[^>]*>(.*?)</actuator>",
        text,
        re.DOTALL,
    )
    if not block_match:
      raise ValueError(f"Actuator block not found for {name}")
    block = block_match.group(1)

    origin_match = re.search(
        r"<origin\s+rpy=\"([^\"]+)\"\s+xyz=\"([^\"]+)\"\s*/>",
        block,
    )
    if not origin_match:
      raise ValueError(f"Origin not found for {name}")

    rpy = [float(v) for v in origin_match.group(1).split()]
    xyz = [float(v) for v in origin_match.group(2).split()]
    thrusters[name] = np.array(rpy + xyz, dtype=float)

  return thrusters


def thruster_matrix_from_scn(scn_path: str) -> np.ndarray:
  """Return a 6x6 matrix with rows [r p y x y z] for Thruster1..Thruster6."""
  thrusters = read_thruster_origins(scn_path)
  rpy: List[np.ndarray] = []
  xyz: List[np.ndarray] = []
  for i in range(1, 7):
    rpy.append(thrusters[f"Thruster{i}"][0:3])
    xyz.append(thrusters[f"Thruster{i}"][3:6])
  return np.vstack(rpy).transpose(),np.vstack(xyz).transpose()

def position_to_tcm(rpy,xyz)  -> np.ndarray:
  n = rpy.shape[1]
  tcm = np.zeros((6,n))
  z = np.array([-1.0,0,0])
  # torque = r X F
  for i in range(n):
    axes = Rpy2Rot(rpy[:,i]) @ z
    r = xyz[:,i]
    tcm[0:3,i] = axes
    tcm[3:6,i] = S(r)@axes
  return tcm

def main() -> None:
  scn_path = Path(__file__).resolve().parents[1] / "scenarios" / "girona1000_vectorial.scn"
  rpy,xyz = thruster_matrix_from_scn(str(scn_path))
  np.set_printoptions(precision=6, suppress=True)
  print(rpy)
  print(xyz)
  tcm = position_to_tcm(rpy,xyz)
  print(tcm)

  # Data structure for YAML
  data = {
      "tcm": {
          "rows": int(tcm.shape[0]),
          "cols": int(tcm.shape[1]),
          "data": tcm.tolist()
      }
  }

  # Save to yaml
  yaml_path = "/home/sia/girona_ws/src/sensorless_force_control/config/control/tcm.yaml"
  with open(yaml_path, "w+") as f:
      yaml.dump(data, f, default_flow_style=False, sort_keys=False)

  print(f"TCM saved to {yaml_path}")
  with open(yaml_path, "r") as f:
        cfg = yaml.safe_load(f)
  rows = cfg["tcm"].get("rows", None)
  cols = cfg["tcm"].get("cols", None)
  data = cfg["tcm"].get("data", None)

  tcm_read = np.array(data)
  print("tcm read from yaml file ",tcm_read)
  print("tcm rank ",np.linalg.matrix_rank(tcm_read))
  print(np.linalg.inv(tcm_read))
  print(np.linalg.pinv(tcm_read))
  print(np.linalg.pinv(tcm_read)@np.array([1,0,0,0,0,0]))
  print(tcm_read@np.linalg.pinv(tcm_read)@np.array([1,0,0,0,0,0]))

  cond = np.linalg.cond(tcm_read)
  s = np.linalg.svd(tcm_read, compute_uv=False)
  print("cond:", cond)
  print("sigma min/max:", s.min(), s.max())
  print("damped_pseudoinverse(tcm_read):", damped_pseudoinverse(tcm_read))
#   damped_pseudoinverse(tcm_read): 
# [[-0.353393  0.346095 -0.00005  -0.047496 -0.000125 -0.4483  ]
#  [-0.353432 -0.343636  0.000052  0.062861  0.000121  0.4483  ]
#  [ 0.353433  0.346093  0.000052 -0.047507 -0.000125  0.4483  ]
#  [ 0.353393 -0.343638 -0.00005   0.06285   0.000121 -0.4483  ]
#  [-0.143213  0.000015 -0.5       0.000092  0.895095  0.      ]
#  [ 0.143215 -0.000015 -0.5      -0.000092 -0.895095 -0.      ]]
  print(damped_pseudoinverse(tcm_read)@np.array([4.73009, -181.302, 85.0827, -0.0728264, -12.1141, -14.0994]))
  print(damped_pseudoinverse(tcm_read)@np.array([1,0,0,0,0,0]))
  print(tcm_read@damped_pseudoinverse(tcm_read)@np.array([1,0,0,0,0,0]))
  for i in range(6):
    setpoint = np.zeros(6)
    setpoint[i] = 0.5
    print("########################")
    print(setpoint)
    print_setpoints_ros(damped_pseudoinverse(tcm_read)@setpoint)
    print("########################")
  offset = tcm_read @ np.array([50,50,50,50,0,0])
  print(offset)
if __name__ == "__main__":
  main()
