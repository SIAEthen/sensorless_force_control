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
  virtual ~ThrusterAllocatorBase() = default;

  virtual void setAllocationMatrix(const Matrix<6, Thrusters>& allocation) = 0;

  virtual void setLimits(const Vector<Thrusters>& min_force,
                         const Vector<Thrusters>& max_force) = 0;

  virtual Vector<Thrusters> allocate(const Vector6& desired_wrench,
                                     Real damping = static_cast<Real>(1e-3)) const = 0;

protected:
  Matrix<6, Thrusters> allocation_{};
  Vector<Thrusters> min_force_{};
  Vector<Thrusters> max_force_{};
};

}  // namespace sfc

#endif  // SFC_THRUST_ALLOCATION_THRUSTER_ALLOCATOR_H_
