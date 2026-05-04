#pragma once

/**
 * Cascaded HQP — slack-variable formulation (Kanoun 2011 / Escande 2014).
 *
 * Each priority level i solves:
 *
 *   min_{qdot, w_i}  1/2 * qdot' * Qi * qdot  +  w_i' * Qwi * w_i
 *
 *   s.t.  lb_k - wk*  <=  Jk * qdot         <=  ub_k - wk*   (k < i)
 *         lb_i        <=  Ji * qdot + w_i    <=  ub_i
 *         qdot_lb     <=  qdot               <=  qdot_ub       (optional)
 *
 * wk* : optimal slack from level k, fixed when solving level i.
 *
 * QP variables at level i:  z = [qdot; w_i]  in R^{n_q + m_i}
 * Hessian:                  H = diag(Qi, 2*Qwi)
 * Gradient:                 g = 0
 */

#include <Eigen/Dense>
#include <cassert>
#include <string>
#include <vector>
#include "hqp_cascaded_solver.h"

// ─────────────────────────────────────────────────────────────────────────────
// Task builders: same logic as possible_tasks.h, output HQPCascadedTask.
// Usage:  solver.addTask(hqp::makeEeTask(uvms, pos_ref, q_ref, gain));
// ─────────────────────────────────────────────────────────────────────────────
#include "config.h"
#include "robot_model/uvms_single_arm.h"
#include "utilts/error_representation.h"
#include "utilts/matrix.h"

namespace hqp {

static constexpr double kInf = 1e30;

// sfc fixed-size → Eigen
template <std::size_t R, std::size_t C>
inline Eigen::MatrixXd toEigen(const sfc::Matrix<R, C>& m) {
    Eigen::MatrixXd e(R, C);
    for (std::size_t r = 0; r < R; ++r)
        for (std::size_t c = 0; c < C; ++c)
            e(r, c) = static_cast<double>(m(r, c));
    return e;
}
template <std::size_t N>
inline Eigen::VectorXd toEigen(const sfc::Vector<N>& v) {
    Eigen::VectorXd e(N);
    for (std::size_t i = 0; i < N; ++i) e(i) = static_cast<double>(v(i));
    return e;
}

template <std::size_t N>
inline Eigen::MatrixXd toEigenMatrix(const sfc::Vector<N>& v) {
    Eigen::VectorXd vec = toEigen(v);
    Eigen::DiagonalMatrix<double, N> mat(vec);
    return mat;
}

template <std::size_t N>
inline sfc::Vector<N> toVector(const Eigen::VectorXd& e){
    assert(static_cast<std::size_t>(e.size()) == N && "toVector: size mismatch");
    sfc::Vector<N> v{};
    for (std::size_t i = 0; i < N; ++i) v(i) = static_cast<sfc::Real>(e(i));
    return v;
}

// ── Roll/pitch  (2-DOF equality) ─────────────────────────────────────────
template <std::size_t ArmDof,
          typename ManipulatorT = sfc::ManipulatorFromYAML<ArmDof>,
          typename VehicleT    = sfc::VehicleBase>
inline HQPCascadedTask makeRollPitchTask(
    const sfc::UvmsSingleArm<ArmDof, ManipulatorT, VehicleT>& uvms,
    const sfc::Vector<2>& rp_ref,
    const sfc::Vector<2>& gain,
    const sfc::Vector<6 + ArmDof>& Kq,
    const sfc::Vector<2>& Kw,
    std::string name = "roll_pitch")
{
    sfc::Matrix<2, 6 + ArmDof> J{};
    const sfc::Matrix<2, 3> map{1,0,0, 0,1,0};
    const auto j_rp = map * uvms.vehicle().J_ko_inv();
    for (std::size_t r = 0; r < 2; ++r)
        for (std::size_t c = 0; c < 3; ++c)
            J(r, c + 3) = j_rp(r, c);

    const sfc::Vector3 rpy = uvms.vehicleRpy();
    Eigen::VectorXd b(2);
    b(0) = static_cast<double>(gain(0) * (rp_ref(0) - rpy(0)));
    b(1) = static_cast<double>(gain(1) * (rp_ref(1) - rpy(1)));

    return HQPCascadedTask(toEigen(J), b, 
                        toEigenMatrix(Kq),toEigenMatrix(Kw), std::move(name));
}

// ── Roll/pitch/yaw  (3-DOF equality) ─────────────────────────────────────
template <std::size_t ArmDof,
          typename ManipulatorT = sfc::ManipulatorFromYAML<ArmDof>,
          typename VehicleT    = sfc::VehicleBase>
inline HQPCascadedTask makeRollPitchYawTask(
    const sfc::UvmsSingleArm<ArmDof, ManipulatorT, VehicleT>& uvms,
    const sfc::Vector3& rpy_ref,
    const sfc::Vector3& gain,
    const sfc::Vector<6 + ArmDof>& Kq,
    const sfc::Vector3& Kw,
    std::string name = "roll_pitch_yaw")
{
    sfc::Matrix<3, 6 + ArmDof> J{};
    const auto j_inv = uvms.vehicle().J_ko_inv();
    for (std::size_t r = 0; r < 3; ++r)
        for (std::size_t c = 0; c < 3; ++c)
            J(r, c + 3) = j_inv(r, c);

    const sfc::Vector3 rpy = uvms.vehicleRpy();
    Eigen::VectorXd b(3);
    for (int i = 0; i < 3; ++i)
        b(i) = static_cast<double>(gain(i) * (rpy_ref(i) - rpy(i)));

    return HQPCascadedTask(toEigen(J), b,
    toEigenMatrix(Kq),toEigenMatrix(Kw),
    std::move(name));
}

// ── End-effector pose  (6-DOF equality) ──────────────────────────────────
template <std::size_t ArmDof,
          typename ManipulatorT = sfc::ManipulatorFromYAML<ArmDof>,
          typename VehicleT    = sfc::VehicleBase>
inline HQPCascadedTask makeEeTask(
    const sfc::UvmsSingleArm<ArmDof, ManipulatorT, VehicleT>& uvms,
    const sfc::Vector3&    pos_ref,
    const sfc::Quaternion& q_ref,
    const sfc::Vector<6>& gain,
    const sfc::Vector<6 + ArmDof>& Kq,
    const sfc::Vector<6>& Kw,
    std::string name = "ee_pose")
{
    sfc::Matrix<6, 6 + ArmDof> J = uvms.jacobian();
    for (std::size_t r = 0; r < 6; ++r) J(r, 3) = sfc::Real(0);  // zero yaw

    const sfc::Vector3    pos_now = uvms.endEffectorPositionNed();
    const sfc::Quaternion q_now   = uvms.endEffectorQuaternionNed();
    const sfc::Quaternion q_err   = sfc::quatError(q_ref, q_now);

    Eigen::VectorXd b(6);
    b(0) = static_cast<double>(gain(0) * (pos_ref(0) - pos_now(0)));
    b(1) = static_cast<double>(gain(1) * (pos_ref(1) - pos_now(1)));
    b(2) = static_cast<double>(gain(2) * (pos_ref(2) - pos_now(2)));
    b(3) = static_cast<double>(gain(3) * q_err.x);
    b(4) = static_cast<double>(gain(4) * q_err.y);
    b(5) = static_cast<double>(gain(5) * q_err.z);

    return HQPCascadedTask(toEigen(J), b, 
    toEigenMatrix(Kq),toEigenMatrix(Kw),
    std::move(name));
}

// ── Vehicle position  (3-DOF equality) ───────────────────────────────────
template <std::size_t ArmDof,
          typename ManipulatorT = sfc::ManipulatorFromYAML<ArmDof>,
          typename VehicleT    = sfc::VehicleBase>
inline HQPCascadedTask makeVehiclePositionTask(
    const sfc::UvmsSingleArm<ArmDof, ManipulatorT, VehicleT>& uvms,
    const sfc::Vector3& pos_ref,
    const sfc::Vector3& gain,
    const sfc::Vector<6 + ArmDof>& Kq,
    const sfc::Vector3& Kw,
    std::string name = "vehicle_pos")
{
    sfc::RotationMatrix r_b2i = uvms.vehicleRotationMatrixBaseToInertial();
    sfc::Matrix<3, 3 + ArmDof> zero{};
    sfc::Matrix<3, 6 + ArmDof> J = sfc::hstack<3, 3, 3 + ArmDof>(r_b2i.m, zero);

    const sfc::Vector3 pos_now = uvms.vehiclePosition();
    Eigen::VectorXd b(3);
    for (int i = 0; i < 3; ++i)
        b(i) = static_cast<double>(gain(i) * (pos_ref(i) - pos_now(i)));

    return HQPCascadedTask(toEigen(J), b, 
    toEigenMatrix(Kq),toEigenMatrix(Kw),
    std::move(name));
}

// ── Nominal joint config  (ArmDof equality) ──────────────────────────────
template <std::size_t ArmDof,
          typename ManipulatorT = sfc::ManipulatorFromYAML<ArmDof>,
          typename VehicleT    = sfc::VehicleBase>
inline HQPCascadedTask makeNominalConfigTask(
    const sfc::UvmsSingleArm<ArmDof, ManipulatorT, VehicleT>& uvms,
    const sfc::Vector<ArmDof>& q_nominal,
    const sfc::Vector<ArmDof>& gain,
    const sfc::Vector<6 + ArmDof>& Kq,
    const sfc::Vector<ArmDof>& Kw,
    std::string name = "nominal_config")
{
    sfc::Matrix<ArmDof, 6 + ArmDof> J{};
    for (std::size_t i = 0; i < ArmDof; ++i) J(i, 6 + i) = sfc::Real(1);

    const sfc::Vector<ArmDof> q_now = uvms.manipulatorPosition();
    Eigen::VectorXd b(ArmDof);
    for (std::size_t i = 0; i < ArmDof; ++i)
        b(i) = static_cast<double>(gain(i) * (q_nominal(i) - q_now(i)));

    return HQPCascadedTask(toEigen(J), b, 
    toEigenMatrix(Kq),toEigenMatrix(Kw),
    std::move(name));
}

// ── Joint limit avoidance  (ArmDof inequality) ───────────────────────────
// Per joint: lb/ub activated when distance to limit < rho.
template <std::size_t ArmDof,
          typename ManipulatorT = sfc::ManipulatorFromYAML<ArmDof>,
          typename VehicleT    = sfc::VehicleBase>
inline HQPCascadedTask makeJointLimitTask(
    const sfc::UvmsSingleArm<ArmDof, ManipulatorT, VehicleT>& uvms,
    const sfc::Vector<ArmDof>& q_min,
    const sfc::Vector<ArmDof>& q_max,
    const sfc::Vector<6 + ArmDof>& Kq,
    const sfc::Vector<ArmDof>& Kw,
    std::string name = "joint_limit")
{
    sfc::Matrix<ArmDof, 6 + ArmDof> J{};
    for (std::size_t i = 0; i < ArmDof; ++i) J(i, 6 + i) = sfc::Real(1);

    return HQPCascadedTask(toEigen(J), toEigen(q_min),  toEigen(q_max), 
    toEigenMatrix(Kq),toEigenMatrix(Kw),
    std::move(name));
}

}  // namespace hqp
