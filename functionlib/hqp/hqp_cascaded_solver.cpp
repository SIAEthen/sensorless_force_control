#include "hqp/hqp_cascaded_solver.h"

#include <qpOASES.hpp>
#include <stdexcept>
#include <vector>

namespace hqp {

static void toRowMajor(const Eigen::MatrixXd& M, qpOASES::real_t* out) {
    const int rows = static_cast<int>(M.rows());
    const int cols = static_cast<int>(M.cols());
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            out[r * cols + c] = static_cast<qpOASES::real_t>(M(r, c));
}

static void toArray(const Eigen::VectorXd& v, qpOASES::real_t* out) {
    for (int k = 0; k < static_cast<int>(v.size()); ++k)
        out[k] = static_cast<qpOASES::real_t>(v(k));
}

HQPCascadedSolver::HQPCascadedSolver(int n_qdot) : n_q_(n_qdot) {
    if (n_qdot <= 0)
        throw std::invalid_argument("HQPCascadedSolver: n_qdot must be > 0");
}

void HQPCascadedSolver::clearTasks() { tasks_.clear(); }

void HQPCascadedSolver::addTask(const HQPCascadedTask& task) {
    if (task.J.cols() != n_q_)
        throw std::invalid_argument("addTask: J.cols() != n_qdot");
    if (task.lb.size() != task.J.rows() || task.ub.size() != task.J.rows())
        throw std::invalid_argument("addTask: lb/ub size mismatch");
    if (task.Qq.rows() != n_q_ || task.Qq.cols() != n_q_)
        throw std::invalid_argument("addTask: Qq must be (n_q x n_q)");
    if (task.Qw.rows() != task.J.rows() || task.Qw.cols() != task.J.rows())
        throw std::invalid_argument("addTask: Qw must be (m x m)");
    tasks_.push_back(task);
}

void HQPCascadedSolver::setQdotBounds(const Eigen::VectorXd& lb,
                                       const Eigen::VectorXd& ub) {
    if (lb.size() != n_q_ || ub.size() != n_q_)
        throw std::invalid_argument("setQdotBounds: size mismatch");
    qdot_lb_ = lb;
    qdot_ub_ = ub;
    has_qdot_bounds_ = true;
}

void HQPCascadedSolver::clearQdotBounds() { has_qdot_bounds_ = false; }

HQPCascadedResult HQPCascadedSolver::solve() const {
    HQPCascadedResult result;
    result.qdot = Eigen::VectorXd::Zero(n_q_);
    result.success = true;

    if (tasks_.empty()) return result;

    const double INF = 1e30;

    std::vector<Eigen::MatrixXd> J_prev;
    std::vector<Eigen::VectorXd> lb_tight, ub_tight;
    int n_prev_c = 0;

    for (int i = 0; i < static_cast<int>(tasks_.size()); ++i) {
        const HQPCascadedTask& task = tasks_[i];
        const int m_i = static_cast<int>(task.J.rows());
        const int n_z  = n_q_ + m_i;
        const int n_c  = n_prev_c + m_i;

        // H = diag(Qi, 2*Qwi)
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_z, n_z);
        H.topLeftCorner(n_q_, n_q_)   = task.Qq;
        H.bottomRightCorner(m_i, m_i) = 2.0 * task.Qw;

        // Constraint matrix A  (n_c x n_z)
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_c, n_z);
        Eigen::VectorXd lbA(n_c), ubA(n_c);

        int row = 0;
        for (int k = 0; k < i; ++k) {
            const int m_k = static_cast<int>(J_prev[k].rows());
            A.block(row, 0, m_k, n_q_) = J_prev[k];
            lbA.segment(row, m_k) = lb_tight[k];
            ubA.segment(row, m_k) = ub_tight[k];
            row += m_k;
        }
        A.block(row, 0,    m_i, n_q_) = task.J;
        A.block(row, n_q_, m_i, m_i)  = Eigen::MatrixXd::Identity(m_i, m_i);
        lbA.segment(row, m_i) = task.lb;
        ubA.segment(row, m_i) = task.ub;

        // Variable bounds
        Eigen::VectorXd lb_z = Eigen::VectorXd::Constant(n_z, -INF);
        Eigen::VectorXd ub_z = Eigen::VectorXd::Constant(n_z,  INF);
        if (has_qdot_bounds_) {
            lb_z.head(n_q_) = qdot_lb_;
            ub_z.head(n_q_) = qdot_ub_;
        }

        std::vector<qpOASES::real_t> H_rm(n_z * n_z), A_rm(n_c * n_z);
        std::vector<qpOASES::real_t> g_rm(n_z, 0.0);
        std::vector<qpOASES::real_t> lb_z_rm(n_z), ub_z_rm(n_z);
        std::vector<qpOASES::real_t> lbA_rm(n_c), ubA_rm(n_c);

        toRowMajor(H, H_rm.data());
        toRowMajor(A, A_rm.data());
        toArray(lb_z, lb_z_rm.data());
        toArray(ub_z, ub_z_rm.data());
        toArray(lbA,  lbA_rm.data());
        toArray(ubA,  ubA_rm.data());

        qpOASES::QProblem qp(n_z, n_c);
        qpOASES::Options opts;
        opts.setToDefault();
        opts.printLevel = qpOASES::PL_NONE;
        qp.setOptions(opts);

        int nWSR = task.maxWSR;
        qpOASES::returnValue ret = qp.init(
            H_rm.data(), g_rm.data(), A_rm.data(),
            lb_z_rm.data(), ub_z_rm.data(),
            lbA_rm.data(), ubA_rm.data(),
            nWSR);

        const bool ok = (ret == qpOASES::SUCCESSFUL_RETURN ||
                         ret == qpOASES::RET_MAX_NWSR_REACHED);
        result.level_success.push_back(ok);

        if (!ok) {
            result.success = false;
            result.slacks.push_back(Eigen::VectorXd::Zero(m_i));
            J_prev.push_back(task.J);
            lb_tight.push_back(task.lb);
            ub_tight.push_back(task.ub);
            n_prev_c += m_i;
            continue;
        }

        std::vector<qpOASES::real_t> z(n_z);
        qp.getPrimalSolution(z.data());

        for (int k = 0; k < n_q_; ++k)
            result.qdot(k) = static_cast<double>(z[k]);

        Eigen::VectorXd w_i(m_i);
        for (int k = 0; k < m_i; ++k)
            w_i(k) = static_cast<double>(z[n_q_ + k]);
        result.slacks.push_back(w_i);

        J_prev.push_back(task.J);
        lb_tight.push_back(task.lb - w_i);
        ub_tight.push_back(task.ub - w_i);
        n_prev_c += m_i;
    }

    return result;
}

}  // namespace hqp
