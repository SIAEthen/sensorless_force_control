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
 *
 * ── Task types ────────────────────────────────────────────────────────────
 * Equality   (J·ζ = b)          : b = beta·b_task + (1-beta)·J·ζ*
 * Inequality (lb ≤ J·ζ ≤ ub)   : CBF / hard bounds, no blending
 */

#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <string>
#include <vector>
#include "hqp_cascaded_solver.h"

#include "config.h"
#include "robot_model/uvms_single_arm.h"
#include "utilts/error_representation.h"
#include "utilts/matrix.h"

namespace hqp {

// ── TaskActivator ──────────────────────────────────────────────────────────
// Smooth task activation with sigmoid S-curve: Inactive → Activating → On → Deactivating → Inactive
// beta ∈ [0,1]: blending weight for task builders (beta=1 fully active, beta=0 fully inactive).
class TaskActivator {
 public:
  enum class Stage { kOff, kActivating, kOn, kDeactivating };

  explicit TaskActivator(double eps = 1e-5) : eps_(eps) {}

  void activate()   { target_ = 1.0; }
  void deactivate() { target_ = 0.0; }

  // Time-based sigmoid: duration = total transition time (s), steepness = S-curve sharpness.
  // s_ is allowed to overshoot [0,1] by `margin` so the sigmoid fully saturates to 0/1.
  // Required margin: s* = 0.5 + ln((1-eps)/eps) / steepness ≈ 0.5 + 11.5/steepness.
  // wise suggestion, do not change steepness, beta might not be zero or near zero
  void stepTime(double dt, double duration = 1.0, double steepness = 10.0) {
    const double rate   = (duration > 0.0) ? dt / duration : 1.0;
    const double margin = std::log((1.0 - eps_) / eps_) / steepness - 0.5;
    if (target_ > 0.5) s_ = std::min(1.0 + margin, s_ + rate);
    else               s_ = std::max(0.0 - margin, s_ - rate);
    beta_ = 1.0 / (1.0 + std::exp(-steepness * (s_ - 0.5)));
    if (beta_ < eps_)       beta_ = 0.0;
    if (beta_ > 1.0 - eps_) beta_ = 1.0;
  }

  // Value-based: v in [a, b] → beta in [0, 1]; a→0, b→1 regardless of sign.
  // Uses a rescaled sigmoid so beta is exactly 0 at v=a and exactly 1 at v=b.
  // v outside [a, b] is clamped to 0 or 1.
  void stepValue(double v, double a, double b, double steepness = 6.0) {
    const double denom = b - a;
    double t = (std::fabs(denom) > 1e-12) ? (v - a) / denom : (v >= b ? 1.0 : 0.0);
    t = std::max(0.0, std::min(1.0, t));
    const double s0 = 1.0 / (1.0 + std::exp( steepness * 0.5));  // sigma(t=0)
    const double s1 = 1.0 / (1.0 + std::exp(-steepness * 0.5));  // sigma(t=1)
    const double st = 1.0 / (1.0 + std::exp(-steepness * (t - 0.5)));
    beta_ = (st - s0) / (s1 - s0);
  }

  void reset(double beta = 0.0) { beta_ = beta; s_ = beta; target_ = 0.0; }

  double beta()   const { return beta_; }
  double target() const { return target_; }

  Stage stage() const {
    if (target_ > 0.5) return (beta_ >= 1.0 - eps_) ? Stage::kOn          : Stage::kActivating;
    else               return (beta_ <= eps_)        ? Stage::kOff         : Stage::kDeactivating;
  }

  bool isFullyActive()   const { return stage() == Stage::kOn; }
  bool isFullyInactive() const { return stage() == Stage::kOff; }
  bool isTransitioning() const {
    const Stage s = stage();
    return s == Stage::kActivating || s == Stage::kDeactivating;
  }

 private:
  double beta_{0.0};
  double s_{0.0};
  double target_{0.0};
  double eps_;
};

// ── Helpers ────────────────────────────────────────────────────────────────

static constexpr double kInf = 1e30;

inline double checkBarrier(double val, double val_min) {
  return val - val_min;
}

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
inline sfc::Vector<N> toVector(const Eigen::VectorXd& e) {
    assert(static_cast<std::size_t>(e.size()) == N && "toVector: size mismatch");
    sfc::Vector<N> v{};
    for (std::size_t i = 0; i < N; ++i) v(i) = static_cast<sfc::Real>(e(i));
    return v;
}

// ═══════════════════════════════════════════════════════════════════════════
// EQUALITY TASKS   (J·ζ = b)
// All support blending:  b_eff = beta·b_task + (1-beta)·J·ζ*
// beta=1.0, zeta_star=0 are the defaults → fully active, no blending.
// ═══════════════════════════════════════════════════════════════════════════

// ── Roll / pitch  (2-DOF) ─────────────────────────────────────────────────
template <std::size_t ArmDof,
          typename ManipulatorT = sfc::ManipulatorFromYAML<ArmDof>,
          typename VehicleT    = sfc::VehicleBase>
inline HQPCascadedTask makeRollPitchTask(
    const sfc::UvmsSingleArm<ArmDof, ManipulatorT, VehicleT>& uvms,
    const sfc::Vector<2>& rp_ref,
    const sfc::Vector<2>& gain,
    const sfc::Vector<6 + ArmDof>& Kq,
    const sfc::Vector<2>& Kw,
    double beta = 1.0,
    sfc::Vector<6 + ArmDof> zeta_star = sfc::Vector<6 + ArmDof>{0},
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

    const Eigen::MatrixXd J_eig = toEigen(J);
    b = beta * b + (1.0 - beta) * J_eig * toEigen(zeta_star);
    return HQPCascadedTask(J_eig, b, toEigenMatrix(Kq), toEigenMatrix(Kw), std::move(name));
}

// ── Roll / pitch / yaw  (3-DOF) ───────────────────────────────────────────
template <std::size_t ArmDof,
          typename ManipulatorT = sfc::ManipulatorFromYAML<ArmDof>,
          typename VehicleT    = sfc::VehicleBase>
inline HQPCascadedTask makeRollPitchYawTask(
    const sfc::UvmsSingleArm<ArmDof, ManipulatorT, VehicleT>& uvms,
    const sfc::Vector3& rpy_ref,
    const sfc::Vector3& gain,
    const sfc::Vector<6 + ArmDof>& Kq,
    const sfc::Vector3& Kw,
    double beta = 1.0,
    sfc::Vector<6 + ArmDof> zeta_star = sfc::Vector<6 + ArmDof>{0},
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

    const Eigen::MatrixXd J_eig = toEigen(J);
    b = beta * b + (1.0 - beta) * J_eig * toEigen(zeta_star);
    return HQPCascadedTask(J_eig, b, toEigenMatrix(Kq), toEigenMatrix(Kw), std::move(name));
}

// ── Vehicle position  (3-DOF) ─────────────────────────────────────────────
template <std::size_t ArmDof,
          typename ManipulatorT = sfc::ManipulatorFromYAML<ArmDof>,
          typename VehicleT    = sfc::VehicleBase>
inline HQPCascadedTask makeVehiclePositionTask(
    const sfc::UvmsSingleArm<ArmDof, ManipulatorT, VehicleT>& uvms,
    const sfc::Vector3& pos_ref,
    const sfc::Vector3& gain,
    const sfc::Vector<6 + ArmDof>& Kq,
    const sfc::Vector3& Kw,
    double beta = 1.0,
    sfc::Vector<6 + ArmDof> zeta_star = sfc::Vector<6 + ArmDof>{0},
    std::string name = "vehicle_pos")
{
    sfc::RotationMatrix r_b2i = uvms.vehicleRotationMatrixBaseToInertial();
    sfc::Matrix<3, 3 + ArmDof> zero{};
    sfc::Matrix<3, 6 + ArmDof> J = sfc::hstack<3, 3, 3 + ArmDof>(r_b2i.m, zero);

    const sfc::Vector3 pos_now = uvms.vehiclePosition();
    Eigen::VectorXd b(3);
    for (int i = 0; i < 3; ++i)
        b(i) = static_cast<double>(gain(i) * (pos_ref(i) - pos_now(i)));

    const Eigen::MatrixXd J_eig = toEigen(J);
    b = beta * b + (1.0 - beta) * J_eig * toEigen(zeta_star);
    return HQPCascadedTask(J_eig, b, toEigenMatrix(Kq), toEigenMatrix(Kw), std::move(name));
}

// ── End-effector pose  (6-DOF) ────────────────────────────────────────────
// Nakamura variable DLS applied before blending.
template <std::size_t ArmDof,
          typename ManipulatorT = sfc::ManipulatorFromYAML<ArmDof>,
          typename VehicleT    = sfc::VehicleBase>
inline HQPCascadedTask makeEeTask(
    const sfc::UvmsSingleArm<ArmDof, ManipulatorT, VehicleT>& uvms,
    const sfc::Vector3&    pos_ref,
    const sfc::Quaternion& q_ref,
    const sfc::Vector<6>&  gain,
    const sfc::Vector<6 + ArmDof>& Kq,
    const sfc::Vector<6>&  Kw,
    double sigma_threshold = 0.01,
    double lambda_max      = 0.001,
    double beta = 1.0,
    sfc::Vector<6 + ArmDof> zeta_star = sfc::Vector<6 + ArmDof>{0},
    std::string name = "ee_pose")
{
    sfc::Matrix<6, 6 + ArmDof> J = uvms.jacobian();

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

    const Eigen::MatrixXd J_eig = toEigen(J);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_eig, Eigen::ComputeThinU);
    const Eigen::VectorXd& sigma = svd.singularValues();
    Eigen::VectorXd scale(sigma.size());
    for (int i = 0; i < sigma.size(); ++i) {
        const double s = sigma(i);
        double lam = 0.0;
        if (s < sigma_threshold) {
            const double r = s / sigma_threshold;
            lam = lambda_max * (1.0 - r * r);
        }
        scale(i) = (s * s) / (s * s + lam * lam);
    }
    b = svd.matrixU() * scale.asDiagonal() * (svd.matrixU().transpose() * b);
    b = beta * b + (1.0 - beta) * J_eig * toEigen(zeta_star);

    return HQPCascadedTask(J_eig, b, toEigenMatrix(Kq), toEigenMatrix(Kw), std::move(name));
}

// ── End-effector pose + feedforward velocity  (6-DOF) ─────────────────────
// Nakamura variable DLS applied before blending.
template <std::size_t ArmDof,
          typename ManipulatorT = sfc::ManipulatorFromYAML<ArmDof>,
          typename VehicleT    = sfc::VehicleBase>
inline HQPCascadedTask makeEeTaskWithVelocity(
    const sfc::UvmsSingleArm<ArmDof, ManipulatorT, VehicleT>& uvms,
    const sfc::Vector6&    vel_ref,
    const sfc::Vector3&    pos_ref,
    const sfc::Quaternion& q_ref,
    const sfc::Vector<6>&  gain,
    const sfc::Vector<6 + ArmDof>& Kq,
    const sfc::Vector<6>&  Kw,
    double sigma_threshold = 0.01,
    double lambda_max      = 0.001,
    double beta = 1.0,
    sfc::Vector<6 + ArmDof> zeta_star = sfc::Vector<6 + ArmDof>{0},
    std::string name = "ee_pose_withvelocity")
{
    sfc::Matrix<6, 6 + ArmDof> J = uvms.jacobian();

    const sfc::Vector3    pos_now = uvms.endEffectorPositionNed();
    const sfc::Quaternion q_now   = uvms.endEffectorQuaternionNed();
    const sfc::Quaternion q_err   = sfc::quatError(q_ref, q_now);

    Eigen::VectorXd b(6);
    b(0) = static_cast<double>(gain(0) * (pos_ref(0) - pos_now(0)) + vel_ref(0));
    b(1) = static_cast<double>(gain(1) * (pos_ref(1) - pos_now(1)) + vel_ref(1));
    b(2) = static_cast<double>(gain(2) * (pos_ref(2) - pos_now(2)) + vel_ref(2));
    b(3) = static_cast<double>(gain(3) * q_err.x + vel_ref(3));
    b(4) = static_cast<double>(gain(4) * q_err.y + vel_ref(4));
    b(5) = static_cast<double>(gain(5) * q_err.z + vel_ref(5));

    const Eigen::MatrixXd J_eig = toEigen(J);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_eig, Eigen::ComputeThinU);
    const Eigen::VectorXd& sigma = svd.singularValues();
    Eigen::VectorXd scale(sigma.size());
    for (int i = 0; i < sigma.size(); ++i) {
        const double s = sigma(i);
        double lam = 0.0;
        if (s < sigma_threshold) {
            const double r = s / sigma_threshold;
            lam = lambda_max * (1.0 - r * r);
        }
        scale(i) = (s * s) / (s * s + lam * lam);
    }
    b = svd.matrixU() * scale.asDiagonal() * (svd.matrixU().transpose() * b);
    b = beta * b + (1.0 - beta) * J_eig * toEigen(zeta_star);

    return HQPCascadedTask(J_eig, b, toEigenMatrix(Kq), toEigenMatrix(Kw), std::move(name));
}

// ── Nominal joint configuration  (ArmDof-DOF) ─────────────────────────────
template <std::size_t ArmDof,
          typename ManipulatorT = sfc::ManipulatorFromYAML<ArmDof>,
          typename VehicleT    = sfc::VehicleBase>
inline HQPCascadedTask makeNominalConfigTask(
    const sfc::UvmsSingleArm<ArmDof, ManipulatorT, VehicleT>& uvms,
    const sfc::Vector<ArmDof>& q_nominal,
    const sfc::Vector<ArmDof>& gain,
    const sfc::Vector<6 + ArmDof>& Kq,
    const sfc::Vector<ArmDof>& Kw,
    double beta = 1.0,
    sfc::Vector<6 + ArmDof> zeta_star = sfc::Vector<6 + ArmDof>{0},
    std::string name = "nominal_config")
{
    sfc::Matrix<ArmDof, 6 + ArmDof> J{};
    for (std::size_t i = 0; i < ArmDof; ++i) J(i, 6 + i) = sfc::Real(1);

    const sfc::Vector<ArmDof> q_now = uvms.manipulatorPosition();
    Eigen::VectorXd b(ArmDof);
    for (std::size_t i = 0; i < ArmDof; ++i)
        b(i) = static_cast<double>(gain(i) * (q_nominal(i) - q_now(i)));

    const Eigen::MatrixXd J_eig = toEigen(J);
    b = beta * b + (1.0 - beta) * J_eig * toEigen(zeta_star);
    return HQPCascadedTask(J_eig, b, toEigenMatrix(Kq), toEigenMatrix(Kw), std::move(name));
}

// ═══════════════════════════════════════════════════════════════════════════
// INEQUALITY TASKS   (lb ≤ J·ζ ≤ ub)
// CBF / hard bounds. Support the same blending as equality tasks:
//   lb_eff = beta·lb_task + (1-beta)·J·ζ*
//   ub_eff = beta·ub_task + (1-beta)·J·ζ*
// At beta=0 both bounds collapse to J·ζ* (equality w.r.t. zeta_star).
// At beta=1 original task bounds are restored.
// ═══════════════════════════════════════════════════════════════════════════

// ── System constraints: vehicle velocity + joint position/velocity bounds ──
template <std::size_t ArmDof,
          typename ManipulatorT = sfc::ManipulatorFromYAML<ArmDof>,
          typename VehicleT    = sfc::VehicleBase>
inline HQPCascadedTask makeSystemConstraintsTask(
    const sfc::UvmsSingleArm<ArmDof, ManipulatorT, VehicleT>& uvms,
    const sfc::Vector<6>& nu_min,
    const sfc::Vector<6>& nu_max,
    const sfc::Vector<ArmDof>& q_min,
    const sfc::Vector<ArmDof>& q_max,
    const sfc::Vector<ArmDof>& dq_min,
    const sfc::Vector<ArmDof>& dq_max,
    const sfc::Vector<6 + ArmDof>& Kq,
    const sfc::Vector<6 + ArmDof>& Kw,
    const sfc::Real& lambda_cbf = 1.0,
    double beta = 1.0,
    sfc::Vector<6 + ArmDof> zeta_star = sfc::Vector<6 + ArmDof>{0},
    std::string name = "system_constraint")
{
    sfc::Matrix<6 + ArmDof, 6 + ArmDof> J{};
    for (std::size_t i = 0; i < 6 + ArmDof; i++) J(i, i) = sfc::Real(1);

    const sfc::Vector<ArmDof> q_now = uvms.manipulatorPosition();
    sfc::Vector<ArmDof> dq_max_cbf{};
    sfc::Vector<ArmDof> dq_min_cbf{};
    for (std::size_t i = 0; i < ArmDof; ++i) {
        dq_max_cbf(i) = sfc::Real( checkBarrier(static_cast<double>(q_max(i)), static_cast<double>(q_now(i))) * static_cast<double>(lambda_cbf));
        dq_min_cbf(i) = sfc::Real(-checkBarrier(static_cast<double>(q_now(i)), static_cast<double>(q_min(i))) * static_cast<double>(lambda_cbf));
    }

    sfc::Vector<ArmDof> dq_min_{};
    sfc::Vector<ArmDof> dq_max_{};
    for (std::size_t i = 0; i < ArmDof; i++) {
        dq_min_(i) = (dq_min_cbf(i) > dq_min(i)) ? dq_min_cbf(i) : dq_min(i);
        dq_max_(i) = (dq_max_cbf(i) < dq_max(i)) ? dq_max_cbf(i) : dq_max(i);
    }

    sfc::Vector<6 + ArmDof> zeta_min{};
    sfc::Vector<6 + ArmDof> zeta_max{};
    for (std::size_t i = 0; i < 6; i++) {
        zeta_min(i) = nu_min(i);
        zeta_max(i) = nu_max(i);
    }
    for (std::size_t i = 0; i < ArmDof; i++) {
        zeta_min(6 + i) = dq_min_(i);
        zeta_max(6 + i) = dq_max_(i);
    }

    const Eigen::MatrixXd J_eig = toEigen(J);
    const Eigen::VectorXd Jzs = J_eig * toEigen(zeta_star);
    const Eigen::VectorXd lb = beta * toEigen(zeta_min) + (1.0 - beta) * Jzs;
    const Eigen::VectorXd ub = beta * toEigen(zeta_max) + (1.0 - beta) * Jzs;
    return HQPCascadedTask(J_eig, lb, ub, toEigenMatrix(Kq), toEigenMatrix(Kw), std::move(name));
}

// ── Joint position limits  (ArmDof-DOF) ───────────────────────────────────
template <std::size_t ArmDof,
          typename ManipulatorT = sfc::ManipulatorFromYAML<ArmDof>,
          typename VehicleT    = sfc::VehicleBase>
inline HQPCascadedTask makeJointLimitTask(
    const sfc::UvmsSingleArm<ArmDof, ManipulatorT, VehicleT>& uvms,
    const sfc::Vector<ArmDof>& q_min,
    const sfc::Vector<ArmDof>& q_max,
    const sfc::Vector<6 + ArmDof>& Kq,
    const sfc::Vector<ArmDof>& Kw,
    double beta = 1.0,
    sfc::Vector<6 + ArmDof> zeta_star = sfc::Vector<6 + ArmDof>{0},
    std::string name = "joint_limit")
{
    sfc::Matrix<ArmDof, 6 + ArmDof> J{};
    for (std::size_t i = 0; i < ArmDof; ++i) J(i, 6 + i) = sfc::Real(1);

    const Eigen::MatrixXd J_eig = toEigen(J);
    const Eigen::VectorXd Jzs = J_eig * toEigen(zeta_star);
    const Eigen::VectorXd lb = beta * toEigen(q_min) + (1.0 - beta) * Jzs;
    const Eigen::VectorXd ub = beta * toEigen(q_max) + (1.0 - beta) * Jzs;
    return HQPCascadedTask(J_eig, lb, ub, toEigenMatrix(Kq), toEigenMatrix(Kw), std::move(name));
}

// ── Self-collision avoidance  (3-DOF, x-axis CBF) ─────────────────────────
template <std::size_t ArmDof,
          typename ManipulatorT = sfc::ManipulatorFromYAML<ArmDof>,
          typename VehicleT    = sfc::VehicleBase>
inline HQPCascadedTask makeSelfCollisionTask(
    const sfc::UvmsSingleArm<ArmDof, ManipulatorT, VehicleT>& uvms,
    const sfc::Vector<6 + ArmDof>& Kq,
    const sfc::Vector<3>& Kw,
    const sfc::Real& x_min = 0.8,
    const sfc::Real& lambda_cbf = 1.0,
    double beta = 1.0,
    sfc::Vector<6 + ArmDof> zeta_star = sfc::Vector<6 + ArmDof>{0},
    std::string name = "self_collision")
{
    sfc::Matrix<6, 6 + ArmDof> J_j3_B = uvms.jacobianBodyFrameN(3);
    sfc::Matrix<6, 6 + ArmDof> J_j5_B = uvms.jacobianBodyFrameN(5);
    sfc::Matrix<6, 6 + ArmDof> J_ee_B = uvms.jacobianBodyFrame();

    sfc::Vector3 x_j3_B = uvms.forwardKinematicsBodyFrameN(3).translation();
    sfc::Vector3 x_j5_B = uvms.forwardKinematicsBodyFrameN(5).translation();
    sfc::Vector3 x_ee_B = uvms.forwardKinematicsBodyFrame().translation();

    sfc::Vector<3> h_min_task = sfc::Vector<3>{
        lambda_cbf * checkBarrier(x_j3_B(0), x_min),
        lambda_cbf * checkBarrier(x_j5_B(0), x_min),
        lambda_cbf * checkBarrier(x_ee_B(0), x_min)} * (-1.0);
    sfc::Vector<3> h_max_task = sfc::Vector<3>{10.0, 10.0, 10.0};

    sfc::Matrix<3, 6 + ArmDof> J_collision{};
    for (u_int i = 0; i < 6 + ArmDof; i++) {
        J_collision(0, i) = J_j3_B(0, i);
        J_collision(1, i) = J_j5_B(0, i);
        J_collision(2, i) = J_ee_B(0, i);
    }

    const Eigen::MatrixXd J_eig = toEigen<3, 6 + ArmDof>(J_collision);
    const Eigen::VectorXd Jzs = J_eig * toEigen(zeta_star);
    const Eigen::VectorXd lb = beta * toEigen(h_min_task) + (1.0 - beta) * Jzs;
    const Eigen::VectorXd ub = beta * toEigen(h_max_task) + (1.0 - beta) * Jzs;
    return HQPCascadedTask(J_eig, lb, ub, toEigenMatrix(Kq), toEigenMatrix(Kw), std::move(name));
}

// ── Manipulability  (1-DOF CBF) ───────────────────────────────────────────
template <std::size_t ArmDof,
          typename ManipulatorT = sfc::ManipulatorFromYAML<ArmDof>,
          typename VehicleT    = sfc::VehicleBase>
inline HQPCascadedTask makeManipulabilityTask(
    const sfc::UvmsSingleArm<ArmDof, ManipulatorT, VehicleT>& uvms,
    const sfc::Vector<6 + ArmDof>& Kq,
    const sfc::Vector<1>& Kw,
    const sfc::Real& man_min = 0.0001,
    const sfc::Real& lambda_cbf = 1.0,
    double beta = 1.0,
    sfc::Vector<6 + ArmDof> zeta_star = sfc::Vector<6 + ArmDof>{0},
    std::string name = "manipulability")
{
    sfc::Real man = uvms.manipulatorManipulability();
    sfc::Vector<ArmDof> J_man = uvms.manipulatorManipulabilityDerivative();
    sfc::Matrix<1, 6 + ArmDof> J_task{};
    for (u_int i = 0; i < ArmDof; i++) J_task(0, 6 + i) = J_man(i);

    sfc::Vector<1> h_min_task = sfc::Vector<1>{-1.0 * lambda_cbf * checkBarrier(man, man_min)};
    sfc::Vector<1> h_max_task = sfc::Vector<1>{100.0};

    const Eigen::MatrixXd J_eig = toEigen(J_task);
    const Eigen::VectorXd Jzs = J_eig * toEigen(zeta_star);
    const Eigen::VectorXd lb = beta * toEigen(h_min_task) + (1.0 - beta) * Jzs;
    const Eigen::VectorXd ub = beta * toEigen(h_max_task) + (1.0 - beta) * Jzs;
    return HQPCascadedTask(J_eig, lb, ub, toEigenMatrix(Kq), toEigenMatrix(Kw), std::move(name));
}

// ── Zero velocity  (6+ArmDof-DOF, lowest-priority regularization) ─────────
template <std::size_t ArmDof,
          typename ManipulatorT = sfc::ManipulatorFromYAML<ArmDof>,
          typename VehicleT    = sfc::VehicleBase>
inline HQPCascadedTask makeZeroVelocityTask(
    const sfc::UvmsSingleArm<ArmDof, ManipulatorT, VehicleT>& uvms,
    const sfc::Real& Kq = 1,
    const sfc::Real& Kw = 1,
    double beta = 1.0,
    sfc::Vector<6 + ArmDof> zeta_star = sfc::Vector<6 + ArmDof>{0},
    std::string name = "zero_velocity")
{
    sfc::Matrix<6 + ArmDof, 6 + ArmDof> J{};
    for (std::size_t i = 0; i < 6 + ArmDof; ++i) J(i, i) = sfc::Real(1);

    const Eigen::MatrixXd J_eig = toEigen(J);
    const Eigen::VectorXd Jzs = J_eig * toEigen(zeta_star);
    Eigen::VectorXd lb_task(6 + ArmDof);
    Eigen::VectorXd ub_task(6 + ArmDof);
    for (std::size_t i = 0; i < 6 + ArmDof; ++i) { lb_task(i) = -100.0; ub_task(i) = 100.0; }
    const Eigen::VectorXd lb = beta * lb_task + (1.0 - beta) * Jzs;
    const Eigen::VectorXd ub = beta * ub_task + (1.0 - beta) * Jzs;

    return HQPCascadedTask(J_eig, lb, ub, Kq, Kw, std::move(name));
}

// ═══════════════════════════════════════════════════════════════════════════
// DIAGNOSTICS
// ═══════════════════════════════════════════════════════════════════════════

template <std::size_t ArmDof,
          typename ManipulatorT = sfc::ManipulatorFromYAML<ArmDof>,
          typename VehicleT    = sfc::VehicleBase>
inline sfc::Vector<3> getSelfCollisionDiagnostics(
    const sfc::UvmsSingleArm<ArmDof, ManipulatorT, VehicleT>& uvms)
{
    sfc::Vector3 x_j3_B = uvms.forwardKinematicsBodyFrameN(3).translation();
    sfc::Vector3 x_j5_B = uvms.forwardKinematicsBodyFrameN(5).translation();
    sfc::Vector3 x_ee_B = uvms.forwardKinematicsBodyFrame().translation();
    return sfc::Vector<3>{x_j3_B(0), x_j5_B(0), x_ee_B(0)};
}

}  // namespace hqp
