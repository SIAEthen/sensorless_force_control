#ifndef SFC_FILTER_FILTER_BASE_H_
#define SFC_FILTER_FILTER_BASE_H_

#include <cstddef>
#include <stdexcept>

#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

template <std::size_t Size>
class FilterBase {
public:
  virtual ~FilterBase() = default;

  // Reset initializes the internal filter state. After reset, the next update
  // continues filtering from this state without a transient jump.
  virtual void reset(const Vector<Size>& value = Vector<Size>{}) {
    state_ = value;
    has_state_ = true;
  }

  virtual Vector<Size> update(const Vector<Size>& input, Real dt) = 0;

  const Vector<Size>& state() const { return state_; }
  bool hasState() const { return has_state_; }

protected:
  Vector<Size> state_{};
  bool has_state_{false};
};

}  // namespace sfc

#endif  // SFC_FILTER_FILTER_BASE_H_
