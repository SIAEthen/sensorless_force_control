from utilts_uvms_math import *
import time
import random
import sys
from pynput import keyboard
from math import pi,sqrt
from scipy.signal import butter, lfilter_zi, lfilter
import numpy as np
from utilts_uvms_math import *
# ---------------------------
# Admittance 6-DOF
# ---------------------------

class Admittance6DOF:
    """
    Admittance dynamics (in body frame).
    M_a * x_dd + D_a * x_d + K_a * x = F_input
    State: x (6) position/orientation delta (linear m, angular rad), x_d (6), x_dd (6)
    """
    def __init__(self, M_a=None, D_a=None, K_a=None,K_I=None, dt=0.01, saturations=None):
        self.dt = dt
        self.M = M_a
        self.D = D_a
        self.K = K_a
        self.KI = K_I
        # precompute inverse mass diag (assumes diagonal M)
        self.Minv = np.linalg.inv(self.M)
        # state
        self.x = np.zeros(6)      # delta position/orientation (m, rad)
        self.x_d = np.zeros(6)    # delta velocity (m/s, rad/s)
        self.x_dd = np.zeros(6)   # delta acceleration (m/s2, rad/s2)
        self.f_integral = np.zeros(6)
        # optional saturation per axis (abs max)
        if saturations is None:
            self.sat = np.full(6, np.inf)
        else:
            self.sat = np.asarray(saturations)

    def update(self, F_in_body, nu_norm, nu_norm_threshold):
        """
        Advance admittance by one time step given wrench input in body frame.
        Returns (delta_pos_orient, delta_vel, delta_acc), all 6-vectors in body frame.
        """
        if nu_norm< nu_norm_threshold:
            F = np.asarray(F_in_body).reshape(6)
            # compute acceleration: x_dd = M^{-1} (F - D x_d - K x)
            rhs = F - (self.D @ self.x_d) - (self.K @ self.x)
            self.x_dd = self.Minv @ rhs
            # integrate (Euler)
            self.x_d = self.x_d + self.x_dd * self.dt
            self.x = self.x + self.x_d * self.dt
            # saturate states
            self.x = np.clip(self.x, -self.sat, self.sat)
            self.x_d = np.clip(self.x_d, -self.sat, self.sat)
        else:
            self.reset()
        return self.x.copy(), self.x_d.copy(), self.x_dd.copy()

    def reset(self):
        self.x = np.zeros(6)
        self.x_d = np.zeros(6)
        self.x_dd = np.zeros(6)
        self.f_integral = np.zeros(6)


class QuaternionImpedanceController:
    
    def __init__(self, M_a=None, D_a=None, K_a=None, dt=0.001):
        """
        初始化阻抗控制器
        M, D, K: 6x6矩阵或标量（自动转为对角阵）
        dt: 控制周期
        """
        self.dt = dt
        self.M = M_a
        self.D = D_a
        self.K = K_a
        self.delta_nu = np.zeros(6)  # 速度误差
        self.nu_r = np.zeros(6)      # 参考速度
        self.pos_r = np.zeros(3)     # 参考位置
        self.q_r = np.array([0, 0, 0, 1])  # 参考姿态

    def update(self, p_d, q_d, nu_d, w_hat):
        """
        更新控制器状态
        输入:
            p_d: 期望位置 (3,)
            q_d: 期望姿态四元数 [x,y,z,w]
            nu_d: 期望速度 (6,)
            w_hat: 测量外力/力矩 (6,)
        输出:
            pos_r, q_r, nu_r, acc_r
        """
        # --- 误差计算 ---
        e_p = p_d - self.pos_r
        e_R = quat_err(q_d, self.q_r)
        e = np.hstack((e_p, e_R))

        # --- 阻抗模型 ---
        # M Δν_dot = w_hat - D Δν - K e
        acc_r = np.linalg.inv(self.M).dot(w_hat - self.D.dot(self.delta_nu) - self.K.dot(e))

        # --- 状态积分 ---
        self.delta_nu += acc_r * self.dt
        self.nu_r = nu_d - self.delta_nu
        self.pos_r += self.nu_r[:3] * self.dt

        # --- 更新姿态 ---
        omega_r = self.nu_r[3:]
        omega_norm = np.linalg.norm(omega_r)
        if omega_norm > 1e-6:
            dq = np.hstack((omega_r/omega_norm * np.sin(0.5*omega_norm*self.dt),
                            np.cos(0.5*omega_norm*self.dt)))
            self.q_r = quat_mult(self.q_r, dq)
            self.q_r /= np.linalg.norm(self.q_r)

        return self.pos_r, self.q_r, self.nu_r, acc_r

    def reset(self,pos_r,q_r):
        self.delta_nu = np.zeros(6)  # 速度误差
        self.nu_r = np.zeros(6)      # 参考速度
        self.pos_r = pos_r
        self.q_r = q_r

class QuaternionForceController:
    
    def __init__(self, M_a=None, D_a=None, K_a=None,K1=None,K2=None, dt=0.001):
        """
        初始化阻抗控制器
        M, D, K: 6x6矩阵或标量（自动转为对角阵）
        dt: 控制周期
        """
        self.dt = dt
        self.M = M_a
        self.D = D_a
        self.K = K_a
        self.K1 = K1
        self.K2 = K2
        self.int_wrench_error = np.zeros(6)
        self.delta_nu = np.zeros(6)  # 速度误差
        self.nu_r = np.zeros(6)      # 参考速度
        self.pos_r = np.zeros(3)     # 参考位置
        self.q_r = np.array([0, 0, 0, 1])  # 参考姿态

    def update(self, p_d, q_d, nu_d, wrench_error):
        """
        更新控制器状态
        输入:
            p_d: 期望位置 (3,)
            q_d: 期望姿态四元数 [x,y,z,w]
            nu_d: 期望速度 (6,)
            wrench_error: 测量外力/力矩 误差 (6,)
        输出:
            pos_r, q_r, nu_r, acc_r
        """
        # --- 误差计算 ---
        e_p = p_d - self.pos_r
        e_R = quat_err(q_d, self.q_r)
        e = np.hstack((e_p, e_R))

        # force close loop
        control = self.K1 @ wrench_error + self.K2 @ self.int_wrench_error
        self.int_wrench_error = self.int_wrench_error + wrench_error * self.dt

        # --- 阻抗模型 ---
        # M Δν_dot = wrench_error - D Δν - K e
        acc_r = np.linalg.inv(self.M).dot(control - self.D.dot(self.delta_nu) - self.K.dot(e))

        # --- 状态积分 ---
        self.delta_nu += acc_r * self.dt
        self.nu_r = nu_d - self.delta_nu
        self.pos_r += self.nu_r[:3] * self.dt

        # --- 更新姿态 ---
        omega_r = self.nu_r[3:]
        omega_norm = np.linalg.norm(omega_r)
        if omega_norm > 1e-6:
            dq = np.hstack((omega_r/omega_norm * np.sin(0.5*omega_norm*self.dt),
                            np.cos(0.5*omega_norm*self.dt)))
            self.q_r = quat_mult(self.q_r, dq)
            self.q_r /= np.linalg.norm(self.q_r)

        return self.pos_r, self.q_r, self.nu_r, acc_r

    def reset(self,pos_r,q_r):
        self.delta_nu = np.zeros(6)  # 速度误差
        self.nu_r = np.zeros(6)      # 参考速度
        self.pos_r = pos_r
        self.q_r = q_r
        self.int_wrench_error = np.zeros(6)


class InverseFreeContactWrenchObserver:
    
    def __init__(self, K, dt=0.001):
        self.dt = dt
        self.he_hat = np.zeros(6)
        self.K = K

    def update(self, J, tau_m_g):
        # J     J_vehicle, first 6 colloms of J
        # tau_m_g   model term, tau-g
        self.d_he_hat = -J@J.transpose()@self.he_hat + J@tau_m_g
        self.he_hat = self.d_he_hat * self.dt + self.he_hat
        return self.he_hat
    


