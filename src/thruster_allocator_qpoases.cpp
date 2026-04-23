#include "thruster_allocator_qpoases.h"

#include <algorithm>

using qpOASES::HST_POSDEF;
using qpOASES::Options;
using qpOASES::SUCCESSFUL_RETURN;
using qpOASES::int_t;
using qpOASES::real_t;

namespace {

sfc::Vector6 squaredElements(const sfc::Vector6& value) {
  sfc::Vector6 out{};
  for (std::size_t i = 0; i < 6; ++i) {
    out(i) = value(i) * value(i);
  }
  return out;
}

sfc::Vector6 reciprocalElements(const sfc::Vector6& value) {
  sfc::Vector6 out{};
  for (std::size_t i = 0; i < 6; ++i) {
    out(i) = static_cast<sfc::Real>(1.0) / value(i);
  }
  return out;
}

sfc::Matrix<6, 6> diagonalMatrix(const sfc::Vector6& diagonal) {
  sfc::Matrix<6, 6> out{};
  for (std::size_t i = 0; i < 6; ++i) {
    out(i, i) = diagonal(i);
  }
  return out;
}

template <std::size_t Rows, std::size_t Cols>
sfc::Matrix<Rows, Cols> scaleMatrix(const sfc::Matrix<Rows, Cols>& matrix,
                                    sfc::Real scalar) {
  sfc::Matrix<Rows, Cols> out{};
  for (std::size_t row = 0; row < Rows; ++row) {
    for (std::size_t col = 0; col < Cols; ++col) {
      out(row, col) = matrix(row, col) * scalar;
    }
  }
  return out;
}

}  // namespace

ThrusterAllocatorQpoases::ThrusterAllocatorQpoases()
    : qp_(6, HST_POSDEF) {
  Options options;
  options.setToFast();
  options.printLevel = qpOASES::PL_NONE;
  qp_.setOptions(options);
}

void ThrusterAllocatorQpoases::setAllocationMatrix(
    const sfc::Matrix<6, 6>& allocation_matrix) {
  allocation_matrix_ = allocation_matrix;
  initialized_ = false;
}

void ThrusterAllocatorQpoases::setLimits(const sfc::Vector6& min_force,
                                         const sfc::Vector6& max_force) {
  for (std::size_t i = 0; i < 6; ++i) {
    if (min_force(i) > max_force(i)) {
      throw std::runtime_error("ThrusterAllocatorQpoases::setLimits: invalid bound order");
    }
  }
  min_force_ = min_force;
  max_force_ = max_force;
  last_force_ = clampToBounds(last_force_);
  initialized_ = false;
}

void ThrusterAllocatorQpoases::setWrenchWeights(const sfc::Vector6& wrench_weights) {
  validateNonnegativeVector(wrench_weights, "ThrusterAllocatorQpoases::setWrenchWeights");
  wrench_weights_ = wrench_weights;
  initialized_ = false;
}

void ThrusterAllocatorQpoases::setWorkingPointWeights(const sfc::Vector6& working_point_weights) {
  validateNonnegativeVector(working_point_weights,
                            "ThrusterAllocatorQpoases::setWorkingPointWeights");
  working_point_weights_ = working_point_weights;
  initialized_ = false;
}

void ThrusterAllocatorQpoases::setSmoothnessWeights(const sfc::Vector6& smoothness_weights) {
  validateNonnegativeVector(smoothness_weights,
                            "ThrusterAllocatorQpoases::setSmoothnessWeights");
  smoothness_weights_ = smoothness_weights;
  initialized_ = false;
}

void ThrusterAllocatorQpoases::setNormalizedInputScale(
    const sfc::Vector6& normalized_input_scale) {
  for (std::size_t i = 0; i < 6; ++i) {
    if (!(normalized_input_scale(i) > static_cast<sfc::Real>(0.0))) {
      throw std::runtime_error(
          "ThrusterAllocatorQpoases::setNormalizedInputScale: require positive scale");
    }
  }
  normalized_input_scale_ = normalized_input_scale;
  initialized_ = false;
}

void ThrusterAllocatorQpoases::setDesiredNormalizedInput(
    const sfc::Vector6& desired_normalized_input) {
  if (!desired_normalized_input.isFinite()) {
    throw std::runtime_error(
        "ThrusterAllocatorQpoases::setDesiredNormalizedInput: non-finite value");
  }
  desired_normalized_input_ = desired_normalized_input;
  initialized_ = false;
}

void ThrusterAllocatorQpoases::setMaxWorkingSetIterations(int max_n_wsr) {
  if (max_n_wsr <= 0) {
    throw std::runtime_error(
        "ThrusterAllocatorQpoases::setMaxWorkingSetIterations: require > 0");
  }
  max_n_wsr_ = max_n_wsr;
}

void ThrusterAllocatorQpoases::reset() {
  qp_.reset();
  initialized_ = false;
  last_damping_ = static_cast<sfc::Real>(-1.0);
  last_force_ = clampToBounds(last_force_);
}

sfc::Vector6 ThrusterAllocatorQpoases::allocate(const sfc::Vector6& desired_wrench) {
  return allocate(desired_wrench, static_cast<sfc::Real>(0.0));
}

sfc::Vector6 ThrusterAllocatorQpoases::allocate(const sfc::Vector6& desired_wrench,
                                                sfc::Real damping) {
  if (damping < static_cast<sfc::Real>(0.0)) {
    throw std::runtime_error(
        "ThrusterAllocatorQpoases::allocate: require damping >= 0");
  }
  validateInputs();
  if (!desired_wrench.isFinite()) {
    throw std::runtime_error(
        "ThrusterAllocatorQpoases::allocate: desired wrench contains non-finite value");
  }

  real_t hessian[36]{};
  real_t gradient[6]{};
  real_t lower_bound[6]{};
  real_t upper_bound[6]{};
  buildDenseQp(desired_wrench, damping, hessian, gradient, lower_bound, upper_bound);

  const bool hessian_changed = (!initialized_) || (damping != last_damping_);
  int_t n_wsr = static_cast<int_t>(max_n_wsr_);
  qpOASES::returnValue status = SUCCESSFUL_RETURN;
  if (hessian_changed) {
    qp_.reset();
    initialized_ = false;
    status = qp_.init(hessian, gradient, lower_bound, upper_bound, n_wsr);
    initialized_ = (status == SUCCESSFUL_RETURN);
  } else {
    status = qp_.hotstart(gradient, lower_bound, upper_bound, n_wsr);
    if (status != SUCCESSFUL_RETURN) {
      qp_.reset();
      initialized_ = false;
      n_wsr = static_cast<int_t>(2 * max_n_wsr_);
      status = qp_.init(hessian, gradient, lower_bound, upper_bound, n_wsr);
      initialized_ = (status == SUCCESSFUL_RETURN);
    }
  }

  if (status != SUCCESSFUL_RETURN) {
    last_force_ = clampToBounds(last_force_);
    return last_force_;
  }

  real_t solution[6]{};
  qp_.getPrimalSolution(solution);
  for (std::size_t i = 0; i < 6; ++i) {
    last_force_(i) = static_cast<sfc::Real>(solution[i]);
  }
  last_force_ = clampToBounds(last_force_);
  last_damping_ = damping;
  return last_force_;
}

sfc::Vector6 ThrusterAllocatorQpoases::computeWrench(const sfc::Vector6& thrusts) const {
  return allocation_matrix_ * thrusts;
}

void ThrusterAllocatorQpoases::validateInputs() const {
  if (!allocation_matrix_.isFinite()) {
    throw std::runtime_error(
        "ThrusterAllocatorQpoases: allocation matrix contains non-finite value");
  }
  if (!min_force_.isFinite() || !max_force_.isFinite() || !wrench_weights_.isFinite() ||
      !working_point_weights_.isFinite() || !smoothness_weights_.isFinite() ||
      !normalized_input_scale_.isFinite() || !desired_normalized_input_.isFinite() ||
      !last_force_.isFinite()) {
    throw std::runtime_error("ThrusterAllocatorQpoases: vector contains non-finite value");
  }
}

void ThrusterAllocatorQpoases::buildDenseQp(const sfc::Vector6& desired_wrench,
                                            sfc::Real damping,
                                            real_t hessian[36],
                                            real_t gradient[6],
                                            real_t lower_bound[6],
                                            real_t upper_bound[6]) const {
  const auto b_transpose = allocation_matrix_.transpose();

  const auto w_tau_sq = squaredElements(wrench_weights_);
  const auto w_mu_sq = squaredElements(working_point_weights_);
  const auto w_delta_sq = squaredElements(smoothness_weights_);
  const auto u_scale_inv = reciprocalElements(normalized_input_scale_);

  const auto W_tau_sq = diagonalMatrix(w_tau_sq);
  const auto W_mu_sq = diagonalMatrix(w_mu_sq);
  const auto W_delta_sq = diagonalMatrix(w_delta_sq);
  const auto D_inv = diagonalMatrix(u_scale_inv);
  const auto I = sfc::identity<6>();

  const auto H_tau = b_transpose * W_tau_sq * allocation_matrix_;
  const auto H_mu = D_inv * W_mu_sq * D_inv;
  const auto H_delta = W_delta_sq;
  const auto H_damping = scaleMatrix(I, damping);
  const auto H = H_tau + H_mu + H_delta + H_damping;

  const auto g_tau = (b_transpose * (W_tau_sq * desired_wrench)) * static_cast<sfc::Real>(-1.0);
  const auto g_mu = (D_inv * (W_mu_sq * desired_normalized_input_)) * static_cast<sfc::Real>(-1.0);
  const auto g_delta = (W_delta_sq * last_force_) * static_cast<sfc::Real>(-1.0);
  const auto g_damping = last_force_ * static_cast<sfc::Real>(-damping);
  const auto g = g_tau + g_mu + g_delta + g_damping;

  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t col = 0; col < 6; ++col) {
      hessian[row * 6 + col] = static_cast<real_t>(H(row, col));
    }
    gradient[row] = static_cast<real_t>(g(row));
    lower_bound[row] = static_cast<real_t>(min_force_(row));
    upper_bound[row] = static_cast<real_t>(max_force_(row));
  }
}

sfc::Vector6 ThrusterAllocatorQpoases::clampToBounds(const sfc::Vector6& force) const {
  sfc::Vector6 clamped{};
  for (std::size_t i = 0; i < 6; ++i) {
    clamped(i) = std::min(std::max(force(i), min_force_(i)), max_force_(i));
  }
  return clamped;
}

void ThrusterAllocatorQpoases::validateNonnegativeVector(const sfc::Vector6& value,
                                                         const char* name) const {
  if (!value.isFinite()) {
    throw std::runtime_error(std::string(name) + ": non-finite value");
  }
  for (std::size_t i = 0; i < 6; ++i) {
    if (value(i) < static_cast<sfc::Real>(0.0)) {
      throw std::runtime_error(std::string(name) + ": negative entry");
    }
  }
}
