#ifndef SENSORLESS_FORCE_CONTROL_THRUSTER_ALLOCATOR_QPOASES_H_
#define SENSORLESS_FORCE_CONTROL_THRUSTER_ALLOCATOR_QPOASES_H_

#include <cstddef>
#include <stdexcept>

#include <qpOASES.hpp>

#include "functionlib/config.h"
#include "functionlib/utilts/matrix.h"
#include "functionlib/utilts/vector.h"

class ThrusterAllocatorQpoases {
 public:
  ThrusterAllocatorQpoases();

  void setAllocationMatrix(const sfc::Matrix<6, 6>& allocation_matrix);
  void setLimits(const sfc::Vector6& min_force, const sfc::Vector6& max_force);
  void setWrenchWeights(const sfc::Vector6& wrench_weights);
  void setWorkingPointWeights(const sfc::Vector6& working_point_weights);
  void setSmoothnessWeights(const sfc::Vector6& smoothness_weights);
  void setNormalizedInputScale(const sfc::Vector6& normalized_input_scale);
  void setDesiredNormalizedInput(const sfc::Vector6& desired_normalized_input);
  void setMaxWorkingSetIterations(int max_n_wsr);
  void reset();

  sfc::Vector6 allocate(const sfc::Vector6& desired_wrench);
  sfc::Vector6 allocate(const sfc::Vector6& desired_wrench, sfc::Real damping);
  sfc::Vector6 computeWrench(const sfc::Vector6& thrusts) const;
  const sfc::Vector6& lastForce() const { return last_force_; }

 private:
  void validateInputs() const;
  void buildDenseQp(const sfc::Vector6& desired_wrench,
                    sfc::Real damping,
                    qpOASES::real_t hessian[36],
                    qpOASES::real_t gradient[6],
                    qpOASES::real_t lower_bound[6],
                    qpOASES::real_t upper_bound[6]) const;
  sfc::Vector6 clampToBounds(const sfc::Vector6& force) const;
  void validateNonnegativeVector(const sfc::Vector6& value, const char* name) const;

  sfc::Matrix<6, 6> allocation_matrix_{};
  sfc::Vector6 min_force_{-100.0, -100.0, -100.0, -100.0, -100.0, -100.0};
  sfc::Vector6 max_force_{100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
  sfc::Vector6 wrench_weights_{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  sfc::Vector6 working_point_weights_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  sfc::Vector6 smoothness_weights_{1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};
  sfc::Vector6 normalized_input_scale_{100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
  sfc::Vector6 desired_normalized_input_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  sfc::Vector6 last_force_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  int max_n_wsr_{32};
  sfc::Real last_damping_{static_cast<sfc::Real>(-1.0)};
  bool initialized_{false};

  qpOASES::QProblemB qp_;
};

#endif  // SENSORLESS_FORCE_CONTROL_THRUSTER_ALLOCATOR_QPOASES_H_
