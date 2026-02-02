#ifndef SFC_KINEMATIC_CONTROL_TASK_PRIORITY_CONTROL_H_
#define SFC_KINEMATIC_CONTROL_TASK_PRIORITY_CONTROL_H_

#include <cstddef>
#include <stdexcept>

#include "config.h"
#include "utilts/linear_algebra.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

template <std::size_t Dof>
class TaskPriorityControl {
public:
  Vector<Dof> solvePrimary(const Matrix<6, Dof>& jacobian,
                           const Vector6& task_twist,
                           Real damping = static_cast<Real>(1e-3)) const {
    if (!jacobian.isFinite() || !task_twist.isFinite() || !isFinite(damping)) {
      throw std::runtime_error("TaskPriorityControl::solvePrimary: non-finite value");
    }
    if (damping < zero()) {
      throw std::runtime_error("TaskPriorityControl::solvePrimary: negative damping");
    }
    const Matrix<Dof, 6> j_pinv = pseudoInverseDls(jacobian, damping);
    return matVec(j_pinv, task_twist);
  }

  Vector<Dof> solveWithNullspace(const Matrix<6, Dof>& jacobian,
                                 const Vector6& task_twist,
                                 const Vector<Dof>& secondary,
                                 Real damping = static_cast<Real>(1e-3)) const {
    if (!jacobian.isFinite() || !task_twist.isFinite() ||
        !secondary.isFinite() || !isFinite(damping)) {
      throw std::runtime_error("TaskPriorityControl::solveWithNullspace: non-finite value");
    }
    if (damping < zero()) {
      throw std::runtime_error("TaskPriorityControl::solveWithNullspace: negative damping");
    }
    const Matrix<Dof, 6> j_pinv = pseudoInverseDls(jacobian, damping);
    const Vector<Dof> primary = matVec(j_pinv, task_twist);
    const Matrix<Dof, Dof> n = identity<Dof>() - (j_pinv * jacobian);
    const Vector<Dof> secondary_proj = matVec(n, secondary);
    return primary + secondary_proj;
  }
};

}  // namespace sfc

#endif  // SFC_KINEMATIC_CONTROL_TASK_PRIORITY_CONTROL_H_
