import numpy as np

from utilts_uvms_math import *
def task_ee_configuration_jacobian(eta,DH,T_0_B,nu_ee_d,x_ee_d,quat_ee_d,K_e):
    J_sigma = Jacobian(eta,DH,T_0_B)
    T_current = Directkinematics(eta,DH,T_0_B)
    quat_ee = Rot2Quat(T_current[0:3,0:3])
    x_ee = T_current[0:3,3]
    dsigma_eecfg = np.zeros(6)
    dsigma_eecfg[0:3] = x_ee_d - x_ee
    dsigma_eecfg[3:6] = quat_err(quat_ee_d,quat_ee)
    return J_sigma,dsigma_eecfg


def damped_pseudoinverse(A, damping=1e-4):
    U, S, Vt = np.linalg.svd(A, full_matrices=False)
    S_damped = S / (S**2 + damping**2)
    return Vt.T @ np.diag(S_damped) @ U.T

def solve_task_priority_control(J_list, sigma_dot_list, dof):
    """
    Recursive task-priority solver.

    Parameters
    ----------
    J_list : list of ndarray
        List of task Jacobians J_i, each of shape (k_i, dof)
    sigma_dot_list : list of ndarray
        List of desired task velocities \dot{sigma}_i, each of shape (k_i,)
    dof : int
        Dimension of the generalized velocity (e.g. 6 + n)

    Returns
    -------
    zeta : ndarray
        Final solution vector \zeta_m, shape (dof,)
    """

    m = len(J_list)
    assert m == len(sigma_dot_list)

    # Initialization
    zeta = np.zeros(dof)
    N = np.eye(dof)

    for i in range(m):
        J = J_list[i]
        sigma_dot = sigma_dot_list[i]

        # Projected Jacobian
        JN = J @ N

        # Pseudoinverse
        # JN_pinv = np.linalg.pinv(JN)
        JN_pinv = damped_pseudoinverse(JN)

        # Recursive update
        zeta = zeta + N @ JN_pinv @ (sigma_dot - J @ zeta)

        # Null-space update
        N = N @ (np.eye(dof) - JN_pinv @ JN)

    return zeta


if __name__ == "__main__":
    dof = 7  # 例如 7 自由度机械臂

    # 固定的 Jacobian，用于可重复测试
    J1 = np.array([
        [0.8, 0.0, 0.1, -0.2, 0.0, 0.3, -0.1],
        [0.0, 0.9, -0.1, 0.0, 0.2, -0.2, 0.0],
        [0.1, -0.2, 0.7, 0.0, -0.1, 0.0, 0.4],
    ], dtype=float)  # 高优先级任务
    J2 = np.array([
        [0.2, -0.1, 0.0, 0.6, 0.0, -0.3, 0.1],
        [0.0, 0.3, -0.2, 0.0, 0.5, 0.0, -0.4],
    ], dtype=float)  # 低优先级任务

    sigma_dot1 = np.array([0.1, 0.0, -0.1])
    sigma_dot2 = np.array([0.05, -0.02])

    J_list = [J1, J2]
    sigma_dot_list = [sigma_dot1, sigma_dot2]

    zeta = solve_task_priority_control(J_list, sigma_dot_list, dof)

    print("Generalized velocity:", zeta)
