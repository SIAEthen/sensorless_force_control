#ifndef SFC_FILTER_LOW_PASS_FILTER_H_
#define SFC_FILTER_LOW_PASS_FILTER_H_

#include <cstddef>
#include <stdexcept>

#include "filter/filter_base.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

template <std::size_t Size>
class FirstOrderLowPassFilter : public FilterBase<Size> {
public:
  explicit FirstOrderLowPassFilter(Real cutoff_hz = static_cast<Real>(1.0))
      : cutoff_hz_(cutoff_hz) {}

  void setCutoffHz(Real cutoff_hz) {
    if (!isFinite(cutoff_hz) || cutoff_hz <= zero()) {
      throw std::runtime_error("FirstOrderLowPassFilter::setCutoffHz: invalid cutoff");
    }
    cutoff_hz_ = cutoff_hz;
  }

  Vector<Size> update(const Vector<Size>& input, Real dt) {
    if (!input.isFinite() || !isFinite(dt)) {
      throw std::runtime_error("FirstOrderLowPassFilter::update: non-finite value");
    }
    if (dt <= zero()) {
      throw std::runtime_error("FirstOrderLowPassFilter::update: non-positive dt");
    }

    if (!this->has_state_) {
      this->state_ = input;
      this->has_state_ = true;
      return this->state_;
    }

    const Real tau = one() / (static_cast<Real>(2.0) * kPi * cutoff_hz_);
    const Real alpha = dt / (tau + dt);

    for (std::size_t i = 0; i < Size; ++i) {
      this->state_(i) = this->state_(i) + alpha * (input(i) - this->state_(i));
    }
    return this->state_;
  }

private:
  Real cutoff_hz_{static_cast<Real>(1.0)};
};

template <std::size_t Size>
class SecondOrderLowPassFilter : public FilterBase<Size> {
public:
  explicit SecondOrderLowPassFilter(Real cutoff_hz = static_cast<Real>(1.0),
                                    Real q = static_cast<Real>(0.70710678))
      : cutoff_hz_(cutoff_hz), q_(q) {}

  void setCutoffHz(Real cutoff_hz) {
    if (!isFinite(cutoff_hz) || cutoff_hz <= zero()) {
      throw std::runtime_error("SecondOrderLowPassFilter::setCutoffHz: invalid cutoff");
    }
    cutoff_hz_ = cutoff_hz;
  }

  void setQ(Real q) {
    if (!isFinite(q) || q <= zero()) {
      throw std::runtime_error("SecondOrderLowPassFilter::setQ: invalid Q");
    }
    q_ = q;
  }

  void reset(const Vector<Size>& value = Vector<Size>{}) override {
    FilterBase<Size>::reset(value);
    x1_ = value;
    x2_ = value;
    y1_ = value;
    y2_ = value;
  }

  Vector<Size> update(const Vector<Size>& input, Real dt) override {
    if (!input.isFinite() || !isFinite(dt)) {
      throw std::runtime_error("SecondOrderLowPassFilter::update: non-finite value");
    }
    if (dt <= zero()) {
      throw std::runtime_error("SecondOrderLowPassFilter::update: non-positive dt");
    }

    if (!this->has_state_) {
      reset(input);
      return this->state_;
    }

    const Real omega = static_cast<Real>(2.0) * kPi * cutoff_hz_;
    const Real k = std::tan(omega * dt * static_cast<Real>(0.5));
    const Real norm = one() / (one() + k / q_ + k * k);
    const Real b0 = k * k * norm;
    const Real b1 = static_cast<Real>(2.0) * b0;
    const Real b2 = b0;
    const Real a1 = static_cast<Real>(2.0) * (k * k - one()) * norm;
    const Real a2 = (one() - k / q_ + k * k) * norm;

    for (std::size_t i = 0; i < Size; ++i) {
      const Real x0 = input(i);
      const Real y0 = b0 * x0 + b1 * x1_(i) + b2 * x2_(i)
                      - a1 * y1_(i) - a2 * y2_(i);
      x2_(i) = x1_(i);
      x1_(i) = x0;
      y2_(i) = y1_(i);
      y1_(i) = y0;
      this->state_(i) = y0;
    }
    return this->state_;
  }

private:
  Real cutoff_hz_{static_cast<Real>(1.0)};
  Real q_{static_cast<Real>(0.70710678)};
  Vector<Size> x1_{};
  Vector<Size> x2_{};
  Vector<Size> y1_{};
  Vector<Size> y2_{};
};

}  // namespace sfc

#endif  // SFC_FILTER_LOW_PASS_FILTER_H_
