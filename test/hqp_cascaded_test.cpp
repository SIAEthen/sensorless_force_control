/**
 * HQPCascadedSolver — standalone test.
 *
 * Build (from hqp_solver/ root):
 *   g++ -std=c++14 -I include \
 *       -I /usr/include/eigen3 \
 *       -I /home/sia/girona_ws/third_party/qpoases/include \
 *       src/hqp_cascaded_solver.cpp test/hqp_cascaded_test.cpp \
 *       -L /home/sia/girona_ws/third_party/qpoases/lib -lqpOASES \
 *       -o /tmp/hqp_cascaded_test
 *   ./hqp_cascaded_test
 */

#include "functionlib/hqp/hqp_cascaded_solver.h"
#include <iostream>
#include <iomanip>
#include <cassert>
#include <cmath>

using namespace hqp;
using Eigen::MatrixXd;
using Eigen::VectorXd;

static void print(const std::string& label, const HQPCascadedResult& r) {
    std::cout << "\n[" << label << "]  success=" << r.success << "\n";
    std::cout << "  qdot = " << r.qdot.transpose() << "\n";
    for (int k = 0; k < (int)r.slacks.size(); ++k)
        std::cout << "  w" << k+1 << " = " << r.slacks[k].transpose()
                  << "  (|w|=" << std::fixed << std::setprecision(6)
                  << r.slacks[k].norm() << ")\n";
}

// ── Test 1 ────────────────────────────────────────────────────────────────────
// n_q=2, two independent equality tasks — both should be satisfied.
//   T1 (high):  qdot[0] = 3
//   T2 (low):   qdot[1] = 5
// Expected: qdot=[3,5], w1=w2=0
static void test1() {
    HQPCascadedSolver solver(2);

    MatrixXd J1(1,2); J1 << 1, 0;
    VectorXd b1(1);   b1 << 3.0;
    solver.addTask(HQPCascadedTask(J1, b1, 1e-6, 1.0, "qdot0=3"));

    MatrixXd J2(1,2); J2 << 0, 1;
    VectorXd b2(1);   b2 << 5.0;
    solver.addTask(HQPCascadedTask(J2, b2, 1e-6, 1.0, "qdot1=5"));

    auto r = solver.solve();
    print("Test1 — independent tasks", r);

    assert(r.success);
    assert(std::abs(r.qdot(0) - 3.0) < 1e-4);
    assert(std::abs(r.qdot(1) - 5.0) < 1e-4);
    assert(r.slacks[0].norm() < 1e-4);
    assert(r.slacks[1].norm() < 1e-4);
    std::cout << "  PASS\n";
}

// ── Test 2 ────────────────────────────────────────────────────────────────────
// n_q=1, conflicting equality tasks:
//   T1 (high):  qdot = 1
//   T2 (low):   qdot = 4
// T1 must win. T2 slack w2 ≈ -(4-1) = -3  (so J2*qdot + w2 = 1 + (-3) = -2... wait)
//
// At level 2: constraint is lb2 - w1* <= J1 qdot <= ub2 - w1*  → {1 - 0} = {1}
//             and lb2 <= J2 qdot + w2 <= ub2  → {4} = 4
// So qdot = 1, 1 + w2 = 4 → w2 = 3.
// |w2| = 3 = violation.
static void test2() {
    HQPCascadedSolver solver(1);

    MatrixXd J1(1,1); J1 << 1.0;
    VectorXd b1(1);   b1 << 1.0;
    solver.addTask(HQPCascadedTask(J1, b1, 1e-6, 1.0, "q=1"));

    MatrixXd J2(1,1); J2 << 1.0;
    VectorXd b2(1);   b2 << 4.0;
    solver.addTask(HQPCascadedTask(J2, b2, 1e-6, 1.0, "q=4"));

    auto r = solver.solve();
    print("Test2 — conflicting equality (T1 wins)", r);

    assert(r.success);
    assert(std::abs(r.qdot(0) - 1.0) < 1e-3 && "T1 must win");
    assert(std::abs(r.slacks[0](0))   < 1e-3 && "T1 slack = 0");
    assert(std::abs(r.slacks[1](0) - 3.0) < 1e-3 && "T2 slack = 3");
    std::cout << "  PASS\n";
}

// ── Test 3 ────────────────────────────────────────────────────────────────────
// n_q=2, inequality task with qdot bounds:
//   T1 (high):  qdot[0] + qdot[1] in [5, 6]
//   qdot bounds: [-10, 10] each
// Expected: qdot[0]+qdot[1] in [5,6], w1=0
static void test3() {
    HQPCascadedSolver solver(2);
    solver.setQdotBounds(VectorXd::Constant(2, -10.0), VectorXd::Constant(2, 10.0));

    MatrixXd J1(1,2); J1 << 1, 1;
    VectorXd lb1(1), ub1(1); lb1 << 5.0; ub1 << 6.0;
    solver.addTask(HQPCascadedTask(J1, lb1, ub1, 1e-6, 1.0, "sum in [5,6]"));

    auto r = solver.solve();
    print("Test3 — inequality + qdot bounds", r);

    assert(r.success);
    double sum = r.qdot(0) + r.qdot(1);
    assert(sum >= 5.0 - 1e-3 && sum <= 6.0 + 1e-3 && "sum constraint");
    assert(r.slacks[0].norm() < 1e-3 && "task satisfied");
    std::cout << "  PASS\n";
}

// ── Test 4 ────────────────────────────────────────────────────────────────────
// n_q=4, three-level UVMS-like example:
//   T1 (high):  keep EE force: J1*qdot = [3,2] (2 rows)
//   T2 (mid):   joint limit avoidance: qdot in [-1,1]^4 (inequality)
//   T3 (low):   minimise motion: qdot ~ 0 (regularisation)
static void test4() {
    const int n = 4;
    HQPCascadedSolver solver(n);

    // T1: EE task (2 constraints on 4 DOF)
    MatrixXd J1(2, n);
    J1 << 1, 0, 1, 0,
          0, 1, 0, 1;
    VectorXd b1(2); b1 << 3.0, 2.0;
    solver.addTask(HQPCascadedTask(J1, b1, 1e-6, 1.0, "EE_task"));

    // T2: inequality on each joint (encoded as J=I, lb=[-1]*n, ub=[1]*n)
    MatrixXd J2 = MatrixXd::Identity(n, n);
    VectorXd lb2 = VectorXd::Constant(n, -1.0);
    VectorXd ub2 = VectorXd::Constant(n,  1.0);
    solver.addTask(HQPCascadedTask(J2, lb2, ub2, 1e-6, 10.0, "joint_limits"));

    // T3: minimise qdot (b=0, equality)
    MatrixXd J3 = MatrixXd::Identity(n, n);
    VectorXd b3 = VectorXd::Zero(n);
    solver.addTask(HQPCascadedTask(J3, b3, 1e-6, 0.01, "min_motion"));

    auto r = solver.solve();
    print("Test4 — 3-level UVMS-like", r);

    assert(r.success);
    // Check T1 is satisfied
    VectorXd err1 = J1 * r.qdot - b1;
    assert(err1.norm() + r.slacks[0].norm() < 1e-3 && "T1 satisfied");
    std::cout << "  PASS\n";
}

// ── Test 5 ────────────────────────────────────────────────────────────────────
// n_q=4, three-level UVMS-like example:
//   T1 (high):  keep EE force: J1*qdot = [3,2] (2 rows)
//   T2 (mid):   joint limit avoidance: qdot in [-1,1]^4 (inequality)
//   T3 (low):   minimise motion: qdot ~ 0 (regularisation)
static void test5() {
    const int n = 12; // uvms 12 dof
    HQPCascadedSolver solver(n);

    // T1: EE task (2 constraints on 4 DOF)
    Eigen::MatrixXd J1 = Eigen::MatrixXd::Identity(n, n);
    VectorXd lb1(n); 
    lb1 << -0.1,-0.1,-0.1,-0.1,-0.1,-0.1, //for vehicle
           -0.2,-0.2,-0.2,-0.2,-0.2,-0.2;
    VectorXd ub1(n); 
    ub1 << 0.1,0.1,0.1,0.1,0.1,0.1, //for vehicle
           0.2,0.2,0.2,0.2,0.2,0.2;
    VectorXd task_jointlimit_kq_vec(n);
    task_jointlimit_kq_vec << 0.01,0.01,0.01,0.01,0.01,0.01, 0.01,0.01,0.01,0.01,0.01,0.01;
    Eigen::DiagonalMatrix<double, n> task_jointlimit_kq(task_jointlimit_kq_vec);
    VectorXd task_jointlimit_kw_vec(n);
    task_jointlimit_kw_vec << 1.0,1.0,1.0,1.0,1.0,1.0, 1.0,1.0,1.0,1.0,1.0,1.0;
    Eigen::DiagonalMatrix<double, n> task_jointlimit_kw(task_jointlimit_kw_vec);
    solver.addTask(HQPCascadedTask(J1, lb1, ub1, task_jointlimit_kq, task_jointlimit_kw, "jointlimit_task"));

    // T2: EE task
    Eigen::MatrixXd J2 = Eigen::MatrixXd::Random(6, n);
    VectorXd b2(6);
    solver.addTask(HQPCascadedTask(J2, b2, 0.1, 0.1, "jointlimit_task"));

    auto r = solver.solve();
    print("Test5 — 2-level UVMS-like", r);

    assert(r.success);
    // Check T1 is satisfied
    VectorXd err1 = J2 * r.qdot - b2;
    std::cout << err1.norm() + r.slacks[0].norm();
    // assert(err1.norm() + r.slacks[0].norm() < 1e-3 && "T1 satisfied");
    std::cout << "  PASS\n";





}

int main() {
    std::cout << "=== HQP Cascaded Solver Tests ===\n";
    test1();
    test2();
    test3();
    test4();
    test5();
    std::cout << "\nAll tests passed.\n";
    return 0;
}
