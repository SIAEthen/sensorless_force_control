#ifndef SFC_THRUST_ALLOCATION_THRUSTER_ALLOCATOR_H_
#define SFC_THRUST_ALLOCATION_THRUSTER_ALLOCATOR_H_

#include <cstddef>
#include <stdexcept>

#include "config.h"
#include "utilts/linear_algebra.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

template <std::size_t Thrusters>
class ThrusterAllocatorBase {
public:
  void setAllocationMatrix(const Matrix<6, Thrusters>& allocation) {
    allocation_ = allocation;
  }

  void setLimits(const Vector<Thrusters>& min_force,
                 const Vector<Thrusters>& max_force) {
    min_force_ = min_force;
    max_force_ = max_force;
  }

  Vector<Thrusters> allocate(const Vector6& desired_wrench,
                             Real damping = static_cast<Real>(1e-3)) const {
    if (!desired_wrench.isFinite() || !allocation_.isFinite() || !isFinite(damping)) {
      throw std::runtime_error("ThrusterAllocator::allocate: non-finite value");
    }
    if (damping < zero()) {
      throw std::runtime_error("ThrusterAllocator::allocate: negative damping");
    }

    const Matrix<Thrusters, 6> pinv = pseudoInverseDls(allocation_, damping);
    Vector<Thrusters> forces = matVec(pinv, desired_wrench);
    return clampVector(forces, min_force_, max_force_);
  }

private:
  Matrix<6, Thrusters> allocation_{};
  Vector<Thrusters> min_force_{};
  Vector<Thrusters> max_force_{};
};

}  // namespace sfc

#endif  // SFC_THRUST_ALLOCATION_THRUSTER_ALLOCATOR_H_
