#ifndef SFC_UTILTS_PRINT_H_
#define SFC_UTILTS_PRINT_H_

#include <ostream>

#include "matrix.h"
#include "rotation.h"
#include "vector.h"

namespace sfc {

template <std::size_t Rows, std::size_t Cols>
inline void print(const Matrix<Rows, Cols>& m, std::ostream& os, const char* name) {
  printMatrix(m, os, name);
}

inline void print(const RotationMatrix& r, std::ostream& os, const char* name) {
  printMatrix(r, os, name);
}

inline void print(const HomogeneousMatrix& t, std::ostream& os, const char* name) {
  printMatrix(t, os, name);
}

inline void print(const Quaternion& q, std::ostream& os, const char* name) {
  printQuaternion(q, os, name);
}

template <std::size_t Size>
inline void print(const Vector<Size>& v, std::ostream& os, const char* name) {
  os << name << " (" << Size << ")\n";
  for (std::size_t i = 0; i < Size; ++i) {
    os << v(i);
    if (i + 1 < Size) {
      os << " ";
    }
  }
  os << "\n";
}

}  // namespace sfc

#endif  // SFC_UTILTS_PRINT_H_
