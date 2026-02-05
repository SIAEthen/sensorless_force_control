// Task-priority solver step for 6DOF vehicle + ArmDof manipulator.
#ifndef SFC_TASK_PRIORITY_CONTROL_TASK_PRIORITY_SOLVER_H_
#define SFC_TASK_PRIORITY_CONTROL_TASK_PRIORITY_SOLVER_H_

#include <cstddef>
#include <stdexcept>

#include "config.h"
#include "utilts/linear_algebra.h"
#include "utilts/matrix.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

template <std::size_t Dof, std::size_t TaskDim>
inline Vector<Dof> taskPrioritySolveStep(
    const Vector<TaskDim>& sigma_dot,
    const Matrix<TaskDim, Dof>& J,
    Matrix<Dof, Dof>& N,
    Vector<Dof>& zeta,
    Real damping = static_cast<Real>(1e-3)) {
  if (!sigma_dot.isFinite() || !J.isFinite() || !N.isFinite() || !zeta.isFinite()) {
    throw std::runtime_error("taskPrioritySolveStep: non-finite value");
  }
  if (!isFinite(damping)) {
    throw std::runtime_error("taskPrioritySolveStep: non-finite damping");
  }
  if (damping < zero()) {
    throw std::runtime_error("taskPrioritySolveStep: negative damping");
  }

  const Matrix<TaskDim, Dof> JN = J * N;
  const Matrix<Dof, TaskDim> JN_pinv = pseudoInverseDls(JN, damping);
  const Vector<TaskDim> residual = sigma_dot - (J * zeta);
  const Vector<Dof> delta = N * (JN_pinv * residual);

  zeta = zeta + delta;
  N = N * (identity<Dof>() - (JN_pinv * JN));

  return zeta;
}

}  // namespace sfc

#endif  // SFC_TASK_PRIORITY_CONTROL_TASK_PRIORITY_SOLVER_H_
