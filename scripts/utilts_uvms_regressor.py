import numpy as np
from utilts_uvms_math import *

theta_est = np.array([-29.9808,
  -12.8688,
   -0.5505,
  324.8560,
   -1.3510,
   -0.5277,
   -4.9991,
    0.0069,
    0.6645,
   -0.7884,
    0.1726,
    0.0069,
    1.6882,
   -0.4823,
    0.0069,
   -0.1778,
    2.1379,
    0.0078,
   -0.0503,
   -0.1778])
# theta_est = np.array([
#   -27.3965,
#   -10.9510,
#    -0.5638,
#   325.4957,
#    -0.1600,
#    -0.0521,
#     0.0057,
#     2.6687,
#     0.6672,
#    -0.0765,
#    -2.6687,
#     0.0001,
#    -1.0959,
#     0.0415,
#    -0.0144,
#     0.0001,
#     1.1467,
#    -0.1285,
#     0.0001,
#     0.0105
# ])
GironaPARAMS = {
    "DH": np.array([
        [0.1065, -1.5708,   0,       0],
        [0.2333,  0,        0,       0],
        [0.1030,  1.5708,   0,       0],
        [0,       0,        0.201,       0]
    ]),
    # "T_0_B": np.array([
    #     [1, 0, 0, 0.1030],
    #     [0, 1, 0, 0],
    #     [0, 0, 1, 0.2010],
    #     [0, 0, 0, 1]
    # ]),
    "T_0_B": np.array([
        [1, 0, 0, 0.74],
        [0, 1, 0, -0.011],
        [0, 0, 1, 0.3856],
        [0, 0, 0, 1]
    ])
    }
def get_GironaParams(q):
    DH_raw = GironaPARAMS['DH']  # 4x4 matrix
    DH = DH_raw.copy()
    DH[:, 3] = DH_raw[:, 3] + q.flatten()

    T_0_B = GironaPARAMS['T_0_B']
    return DH, T_0_B
# girona 500 experiment
def Regressor_stupid(R):
    """
    对应 MATLAB 函数 Regressor_stupid(R)
    G. Antonelli, Simurv 4.0, 2013

    参数:
        R : 3x3 numpy.ndarray 旋转矩阵
    返回:
        Phi : 6x4 numpy.ndarray
    """
    z = np.array([0, 0, 1])
    Rz = R.T @ z  # R' * z

    Phi = np.block([
        [-Rz.reshape(3,1), np.zeros((3,3))],
        [np.zeros((3,1)), S(Rz)]
    ])
    return Phi

def U_mat(R_b_a, r_ab_a):
    """
    Force/moment transformation matrix from frame b to frame a

    Parameters
    ----------
    R_b_a : (3,3) np.ndarray
        Rotation matrix from frame b to frame a
    r_ab_a : (3,) or (3,1) np.ndarray
        Vector from origin of frame a to origin of frame b, expressed in frame a

    Returns
    -------
    U : (6,6) np.ndarray
        Transformation matrix
    """
    r_ab_a = np.asarray(r_ab_a).reshape(3)
    U = np.block([
        [R_b_a,                np.zeros((3,3))],
        [S(r_ab_a) @ R_b_a,    R_b_a]
    ])
    return U

# pass test
def Regressor_Girona500_inertia_frame(eta, q):
    """
    Python translation of Regressor_Girona500_inertia_frame.m
    G. Antonelli, Simurv 4.0, 2013
    """
    
    q = np.asarray(q).reshape(-1, 1)
    n = q.shape[0]

    DH_raw = GironaPARAMS['DH']  # 4x4 matrix
    DH = DH_raw.copy()
    DH[:, 3] = DH_raw[:, 3] + q.flatten()

    T_0_B = GironaPARAMS['T_0_B']
    R_0_B = T_0_B[0:3, 0:3]
    r_B0_B = T_0_B[0:3, 3]

    # 从姿态确定旋转矩阵 RBI
    if eta.shape[0] == 7:
        RBI = Quat2Rot(eta[3:7])
    else:
        RBI = Rpy2Rot(eta[3:6])

    R0I = RBI @ R_0_B

    RiI = np.zeros((3,3,n))
    RiI[:,:,0] = R0I @ Rot_dh(DH[0,1], DH[0,3])
    for i in range(1, n):
        RiI[:,:,i] = RiI[:,:,i-1] @ Rot_dh(DH[i,1], DH[i,3])

    # U 变换
    U_B_I = U_mat(RBI, [0,0,0])
    U_0_B = U_mat(R_0_B, r_B0_B)
    U_0_I = U_B_I @ U_0_B

    # 初始化回归矩阵
    Y = np.zeros((6, 4*(n+1)))
    Y[:, 0:4] = U_B_I @ Regressor_stupid(RBI)

    U_i_I = U_0_I
    for i in range(n):
        T = Homogeneous_dh(DH[i, :])
        UU = U_mat(T[0:3, 0:3], T[0:3, 3])
        U_i_I = U_i_I @ UU
        Y[:, 4*(i+1):4*(i+2)] = U_i_I @ Regressor_stupid(RiI[:,:,i])

    return Y

#pass test
def Regressor_Girona500_body_frame(eta, q):
    Y_inertia_frame = Regressor_Girona500_inertia_frame(eta, q)
    # 从姿态确定旋转矩阵 RBI
    if eta.shape[0] == 7:
        RBI = Quat2Rot(eta[3:7])
    else:
        RBI = Rpy2Rot(eta[3:6])
    R_I_B = RBI.transpose()
    U_I_B = U_mat(R_I_B, [0,0,0])
    Y_body_frame = U_I_B @ Y_inertia_frame
    return Y_body_frame

def get_g_body_frame(eta, q):
    global theta_est
    Y = Regressor_Girona500_body_frame(eta, q)
    return Y @ theta_est
def get_g_inertia_frame(eta,q):
    global theta_est
    Y = Regressor_Girona500_inertia_frame(eta, q)
    return Y @ theta_est

if __name__ == "__main__":
    # eta = np.zeros(6)
    # q = np.zeros(4)
    # Y = Regressor_Girona500_inertia_frame(eta, q)
    
    # print(Y @ theta_est)
    q = np.array([0,0.1,0,0])
    eta = np.array([0,0,0, 0.1,0.3,0.2])
    # Y = Regressor_Girona500_inertia_frame(eta, q)
    Y = Regressor_Girona500_body_frame(eta, q)
    print(Y @ theta_est)
    print(get_g_inertia_frame(eta,q))