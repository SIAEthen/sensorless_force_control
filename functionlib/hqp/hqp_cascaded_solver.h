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
#include <string>
#include <vector>

namespace hqp {

struct HQPCascadedTask {
    Eigen::MatrixXd J;   // (m x n_q)  task Jacobian
    Eigen::VectorXd lb;  // (m)        constraint lower bound
    Eigen::VectorXd ub;  // (m)        constraint upper bound
    Eigen::MatrixXd Qq;  // (n_q x n_q) weight on qdot     (default: eps*I)
    Eigen::MatrixXd Qw;  // (m x m)    weight on slack w    (default: I)
    int maxWSR = 200;
    std::string name;

    HQPCascadedTask() = default;

    // Equality task:  J*qdot = b  (lb = ub = b)
    HQPCascadedTask(Eigen::MatrixXd J_, Eigen::VectorXd b,
                    double qq_weight = 1e-6, double qw_weight = 1.0,
                    std::string name_ = "")
        : J(std::move(J_)),
          lb(b), ub(std::move(b)),
          Qq(qq_weight * Eigen::MatrixXd::Identity(J.cols(), J.cols())),
          Qw(qw_weight * Eigen::MatrixXd::Identity(J.rows(), J.rows())),
          name(std::move(name_)) {}

    // Inequality task:  lb <= J*qdot <= ub
    HQPCascadedTask(Eigen::MatrixXd J_, Eigen::VectorXd lb_, Eigen::VectorXd ub_,
                    double qq_weight = 1e-6, double qw_weight = 1.0,
                    std::string name_ = "")
        : J(std::move(J_)),
          lb(std::move(lb_)), ub(std::move(ub_)),
          Qq(qq_weight * Eigen::MatrixXd::Identity(J.cols(), J.cols())),
          Qw(qw_weight * Eigen::MatrixXd::Identity(J.rows(), J.rows())),
          name(std::move(name_)) {}
    
    // Inequality task with matrix gain
    // Inequality task:  lb <= J*qdot <= ub
    HQPCascadedTask(Eigen::MatrixXd J_, Eigen::VectorXd lb_, Eigen::VectorXd ub_,
                    Eigen::MatrixXd qq_weight, Eigen::MatrixXd qw_weight,
                    std::string name_ = "")
        : J(std::move(J_)),
          lb(std::move(lb_)), ub(std::move(ub_)),
          Qq(qq_weight),
          Qw(qw_weight),
          name(std::move(name_)) {}

    // Equality task with matrix gain
    // Equality task:  J*qdot = b  (lb = ub = b)
    HQPCascadedTask(Eigen::MatrixXd J_, Eigen::VectorXd b,
                    Eigen::MatrixXd qq_weight, Eigen::MatrixXd qw_weight,
                    std::string name_ = "")
        : J(std::move(J_)),
          lb(b), ub(std::move(b)),
          Qq(qq_weight),
          Qw(qw_weight),
          name(std::move(name_)) {}
};

struct HQPCascadedResult {
    Eigen::VectorXd qdot;
    std::vector<Eigen::VectorXd> slacks;  // optimal w_i per level
    bool success = false;
    std::vector<bool> level_success;
};

class HQPCascadedSolver {
public:
    explicit HQPCascadedSolver(int n_qdot);

    void clearTasks();
    void addTask(const HQPCascadedTask& task);
    void setQdotBounds(const Eigen::VectorXd& lb, const Eigen::VectorXd& ub);
    void clearQdotBounds();

    HQPCascadedResult solve() const;

    int nTasks() const { return static_cast<int>(tasks_.size()); }
    int nQdot()  const { return n_q_; }

private:
    int n_q_;
    std::vector<HQPCascadedTask> tasks_;
    Eigen::VectorXd qdot_lb_, qdot_ub_;
    bool has_qdot_bounds_ = false;
};

}  // namespace hqp
