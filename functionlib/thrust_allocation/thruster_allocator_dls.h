#ifndef SFC_THRUST_ALLOCATION_THRUSTER_ALLOCATOR_DLS_H_
#define SFC_THRUST_ALLOCATION_THRUSTER_ALLOCATOR_DLS_H_

#include "thruster_allocator_base.h"
// #include "functionlib/utilts/print.h"
namespace sfc {

template <std::size_t Thrusters>
class ThrusterAllocatorDls : public ThrusterAllocatorBase<Thrusters> {
 public:
  void setAllocationMatrix(const Matrix<6, Thrusters>& allocation) override {
    this->allocation_ = allocation;
  }

  void setLimits(const Vector<Thrusters>& min_force,
                 const Vector<Thrusters>& max_force) override {
    this->min_force_ = min_force;
    this->max_force_ = max_force;
  }

  Vector<Thrusters> allocate(const Vector6& desired_wrench,
                             Real damping = static_cast<Real>(1e-3)) const override {
    if (!desired_wrench.isFinite() || !this->allocation_.isFinite() || !isFinite(damping)) {
      throw std::runtime_error("ThrusterAllocatorDls::allocate: non-finite value");
    }
    if (damping < zero()) {
      throw std::runtime_error("ThrusterAllocatorDls::allocate: negative damping");
    }

    const Matrix<Thrusters, 6> pinv = pseudoInverseDls(this->allocation_, damping);
    Vector<Thrusters> forces = pinv * desired_wrench;
    // print(pinv,std::cout,"pinv tcm");
    return clampVector(forces, this->min_force_, this->max_force_);
  }
  Vector6 computeWrench(const Vector<Thrusters>& force ) const {
    return this->allocation_*force;
  }
};

}  // namespace sfc

#endif  // SFC_THRUST_ALLOCATION_THRUSTER_ALLOCATOR_DLS_H_
