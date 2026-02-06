# this is very stupid, I quit and now I will use the urdf configuration
import numpy as np
from cmath import atan, pi

dh_a =     np.array([46.0,293.6,40.8,40.8,40.8,0,120])/1000.0
dh_alpha = np.array([pi/2,0.0,-pi/2,-pi/2,-pi/2,pi/2,0.0])
dh_d = np.array([107.4,0.0,0.0,-160.0,0.0,-223.5,0.0])/1000.0
theta_a = atan(5.2/293.55).real
print(theta_a)
dh_theta = np.array([pi,-pi/2+theta_a,-pi/2-theta_a,0,0,0,-pi/2])

from utilts_uvms_math import *


R_0_B = Rpy2Rot(np.array([pi, 0.0, -0.175]))
p_B0_B = np.array([0.7322, -0.1382, 0.485]).reshape(3, 1)
T_0_B = np.vstack((np.hstack((R_0_B,p_B0_B)),
                   np.array([0,0,0,1])))

T_0_B = np.eye(4)

eta = np.array([0,0,0,0,0,0])
DH = np.vstack((dh_a,dh_alpha,dh_d,dh_theta)).transpose()

#we dont have frame 7 in this robot, so we only use the first 6 rows
DH_i = DH[0:6,:]
T_ee_ned = Directkinematics(eta,DH_i,T_0_B)
q_ee = Rot2Quat(T_ee_ned[0:3,0:3])
rpy_ee = Quat2Rpy(q_ee)
print(T_ee_ned)
print(q_ee)
print(rpy_ee)


