#ifndef SFC_UTILTS_ROTATION_H_
#define SFC_UTILTS_ROTATION_H_

#include <cmath>
#include <cstddef>
#include <ostream>
#include <stdexcept>

#include "config.h"
#include "utilts/matrix.h"
#include "utilts/real.h"
#include "utilts/vector.h"

namespace sfc {

struct Quaternion;
struct RotationMatrix;
struct HomogeneousMatrix;
Vector3 RotationMatrix2RPY(const RotationMatrix& r);
Vector3 Quaternion2RPY(const Quaternion& q);
Vector3 rpyFromRotationMatrix(const RotationMatrix& r);
Vector3 rpyFromQuaternion(const Quaternion& q);

struct RotationMatrix {
  Matrix3 m{};

  sfc::Real operator()(std::size_t row, std::size_t col) const;
  sfc::Real& operator()(std::size_t row, std::size_t col);
  RotationMatrix operator*(const RotationMatrix& other) const;
  Vector3 operator*(const Vector3& v) const;
  bool isFinite() const;

  static RotationMatrix identity();
  static RotationMatrix fromRPY(sfc::Real roll, sfc::Real pitch, sfc::Real yaw);
  static RotationMatrix fromRPY(const Vector3& rpy);
  static RotationMatrix fromQuaternion(const Quaternion& q);
  

  RotationMatrix transpose() const;
};

struct HomogeneousMatrix {
  Matrix4 m{};

  sfc::Real operator()(std::size_t row, std::size_t col) const;
  sfc::Real& operator()(std::size_t row, std::size_t col);
  HomogeneousMatrix operator*(const HomogeneousMatrix& other) const;
  Vector<4> operator*(const Vector<4>& v) const;
  Vector3 operator*(const Vector3& v) const;
  bool isFinite() const;

  static HomogeneousMatrix identity();
  static HomogeneousMatrix fromRotationTranslation(const RotationMatrix& r,
                                                    const Vector3& p);
  

  RotationMatrix rotation() const;
  Vector3 translation() const;
  Vector3 zAxis() const;
};

struct Quaternion {
  sfc::Real w{1.0};
  sfc::Real x{0.0};
  sfc::Real y{0.0};
  sfc::Real z{0.0};

  static Quaternion identity();
  static Quaternion fromRPY(sfc::Real roll, sfc::Real pitch, sfc::Real yaw);
  static Quaternion fromRPY(const Vector3& rpy);
  static Quaternion fromRotationMatrix(const RotationMatrix& r);

  Quaternion operator+(const Quaternion& other) const;
  Quaternion operator-(const Quaternion& other) const;
  Quaternion operator*(const Quaternion& other) const;
  sfc::Real norm() const;
  Quaternion normalized() const;
  Quaternion conjugate() const;
  Quaternion inverse() const;
};

inline void printQuaternion(const Quaternion& q, std::ostream& os, const char* name) {
  os << name << ": [w=" << q.w
     << ", x=" << q.x
     << ", y=" << q.y
     << ", z=" << q.z << "]\n";
}

inline void printMatrix(const RotationMatrix& r, std::ostream& os, const char* name) {
  printMatrix(r.m, os, name);
}

inline void printMatrix(const HomogeneousMatrix& t, std::ostream& os, const char* name) {
  printMatrix(t.m, os, name);
}


inline sfc::Real RotationMatrix::operator()(std::size_t row, std::size_t col) const {
  return m(row, col);
}

inline sfc::Real& RotationMatrix::operator()(std::size_t row, std::size_t col) {
  return m(row, col);
}

inline bool RotationMatrix::isFinite() const { return m.isFinite(); }

inline RotationMatrix RotationMatrix::identity() {
  RotationMatrix r{};
  r(0, 0) = 1.0;
  r(1, 1) = 1.0;
  r(2, 2) = 1.0;
  return r;
}

inline RotationMatrix RotationMatrix::fromRPY(sfc::Real roll, sfc::Real pitch, sfc::Real yaw) {
  const sfc::Real cr = std::cos(roll);
  const sfc::Real sr = std::sin(roll);
  const sfc::Real cp = std::cos(pitch);
  const sfc::Real sp = std::sin(pitch);
  const sfc::Real cy = std::cos(yaw);
  const sfc::Real sy = std::sin(yaw);

  RotationMatrix r{};
  r(0, 0) = cy * cp;
  r(0, 1) = cy * sp * sr - sy * cr;
  r(0, 2) = cy * sp * cr + sy * sr;
  r(1, 0) = sy * cp;
  r(1, 1) = sy * sp * sr + cy * cr;
  r(1, 2) = sy * sp * cr - cy * sr;
  r(2, 0) = -sp;
  r(2, 1) = cp * sr;
  r(2, 2) = cp * cr;
  return r;
}

inline RotationMatrix RotationMatrix::fromRPY(const Vector3& rpy) {
  return fromRPY(rpy(0), rpy(1), rpy(2));
}

inline RotationMatrix RotationMatrix::fromQuaternion(const Quaternion& q) {
  if (!std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) ||
      !std::isfinite(q.z)) {
    throw std::runtime_error("Quaternion contains non-finite value");
  }
  const Quaternion n = q.normalized();
  const sfc::Real xx = n.x * n.x;
  const sfc::Real yy = n.y * n.y;
  const sfc::Real zz = n.z * n.z;
  const sfc::Real xy = n.x * n.y;
  const sfc::Real xz = n.x * n.z;
  const sfc::Real yz = n.y * n.z;
  const sfc::Real wx = n.w * n.x;
  const sfc::Real wy = n.w * n.y;
  const sfc::Real wz = n.w * n.z;

  RotationMatrix r{};
  r(0, 0) = static_cast<sfc::Real>(1.0) - static_cast<sfc::Real>(2.0) * (yy + zz);
  r(0, 1) = static_cast<sfc::Real>(2.0) * (xy - wz);
  r(0, 2) = static_cast<sfc::Real>(2.0) * (xz + wy);
  r(1, 0) = static_cast<sfc::Real>(2.0) * (xy + wz);
  r(1, 1) = static_cast<sfc::Real>(1.0) - static_cast<sfc::Real>(2.0) * (xx + zz);
  r(1, 2) = static_cast<sfc::Real>(2.0) * (yz - wx);
  r(2, 0) = static_cast<sfc::Real>(2.0) * (xz - wy);
  r(2, 1) = static_cast<sfc::Real>(2.0) * (yz + wx);
  r(2, 2) = static_cast<sfc::Real>(1.0) - static_cast<sfc::Real>(2.0) * (xx + yy);
  return r;
}

inline RotationMatrix RotationMatrix::operator*(const RotationMatrix& other) const {
  RotationMatrix result{};
  result.m = m * other.m;
  return result;
}

inline Vector3 RotationMatrix::operator*(const Vector3& v) const {
  Vector3 r{};
  r(0) = m(0, 0) * v(0) + m(0, 1) * v(1) + m(0, 2) * v(2);
  r(1) = m(1, 0) * v(0) + m(1, 1) * v(1) + m(1, 2) * v(2);
  r(2) = m(2, 0) * v(0) + m(2, 1) * v(1) + m(2, 2) * v(2);
  return r;
}

inline RotationMatrix RotationMatrix::transpose() const {
  RotationMatrix r{};
  r.m = m.transpose();
  return r;
}

inline sfc::Real HomogeneousMatrix::operator()(std::size_t row, std::size_t col) const {
  return m(row, col);
}

inline sfc::Real& HomogeneousMatrix::operator()(std::size_t row, std::size_t col) {
  return m(row, col);
}

inline bool HomogeneousMatrix::isFinite() const { return m.isFinite(); }

inline HomogeneousMatrix HomogeneousMatrix::identity() {
  HomogeneousMatrix t{};
  t(0, 0) = 1.0;
  t(1, 1) = 1.0;
  t(2, 2) = 1.0;
  t(3, 3) = 1.0;
  return t;
}

inline HomogeneousMatrix HomogeneousMatrix::fromRotationTranslation(const RotationMatrix& r,
                                                             const Vector3& p) {
  if (!r.isFinite()) {
    throw std::runtime_error("RotationMatrix contains non-finite value");
  }
  if (!p.isFinite()) {
    throw std::runtime_error("Translation contains non-finite value");
  }
  HomogeneousMatrix t = identity();
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      t(row, col) = r(row, col);
    }
    t(row, 3) = p(row);
  }
  return t;
}

inline HomogeneousMatrix HomogeneousMatrix::operator*(const HomogeneousMatrix& other) const {
  HomogeneousMatrix result{};
  result.m = m * other.m;
  return result;
}

inline Vector<4> HomogeneousMatrix::operator*(const Vector<4>& v) const {
  Vector<4> r{};
  r(0) = m(0, 0) * v(0) + m(0, 1) * v(1) + m(0, 2) * v(2) + m(0, 3) * v(3);
  r(1) = m(1, 0) * v(0) + m(1, 1) * v(1) + m(1, 2) * v(2) + m(1, 3) * v(3);
  r(2) = m(2, 0) * v(0) + m(2, 1) * v(1) + m(2, 2) * v(2) + m(2, 3) * v(3);
  r(3) = m(3, 0) * v(0) + m(3, 1) * v(1) + m(3, 2) * v(2) + m(3, 3) * v(3);
  return r;
}

inline Vector3 HomogeneousMatrix::operator*(const Vector3& v) const {
  Vector<4> v4{v(0), v(1), v(2), static_cast<sfc::Real>(1.0)};
  Vector<4> r4 = (*this) * v4;
  return Vector3{r4(0), r4(1), r4(2)};
}

inline RotationMatrix HomogeneousMatrix::rotation() const {
  if (!isFinite()) {
    throw std::runtime_error("HomogeneousMatrix contains non-finite value");
  }
  RotationMatrix r{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      r(row, col) = m(row, col);
    }
  }
  return r;
}

inline Vector3 HomogeneousMatrix::translation() const {
  if (!isFinite()) {
    throw std::runtime_error("HomogeneousMatrix contains non-finite value");
  }
  Vector3 p{};
  p(0) = m(0, 3);
  p(1) = m(1, 3);
  p(2) = m(2, 3);
  return p;
}

inline Vector3 HomogeneousMatrix::zAxis() const {
  if (!isFinite()) {
    throw std::runtime_error("HomogeneousMatrix contains non-finite value");
  }
  Vector3 z{};
  z(0) = m(0, 2);
  z(1) = m(1, 2);
  z(2) = m(2, 2);
  return z;
}

inline Quaternion Quaternion::identity() { return Quaternion{1.0, 0.0, 0.0, 0.0}; }

inline Quaternion Quaternion::fromRPY(sfc::Real roll, sfc::Real pitch, sfc::Real yaw) {
  const sfc::Real half_roll = roll * static_cast<sfc::Real>(0.5);
  const sfc::Real half_pitch = pitch * static_cast<sfc::Real>(0.5);
  const sfc::Real half_yaw = yaw * static_cast<sfc::Real>(0.5);

  const sfc::Real cr = std::cos(half_roll);
  const sfc::Real sr = std::sin(half_roll);
  const sfc::Real cp = std::cos(half_pitch);
  const sfc::Real sp = std::sin(half_pitch);
  const sfc::Real cy = std::cos(half_yaw);
  const sfc::Real sy = std::sin(half_yaw);

  Quaternion q{
      cr * cp * cy + sr * sp * sy,
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy};
  return q.normalized();
}

inline Quaternion Quaternion::fromRPY(const Vector3& rpy) {
  return fromRPY(rpy(0), rpy(1), rpy(2));
}


inline Quaternion Quaternion::fromRotationMatrix(const RotationMatrix& r) {
  if (!r.isFinite()) {
    throw std::runtime_error("RotationMatrix contains non-finite value");
  }
  const sfc::Real t = r(0, 0) + r(1, 1) + r(2, 2);
  Quaternion q{};
  if (t > static_cast<sfc::Real>(0.0)) {
    const sfc::Real s = std::sqrt(t + static_cast<sfc::Real>(1.0)) *
                        static_cast<sfc::Real>(2.0);
    q.w = static_cast<sfc::Real>(0.25) * s;
    q.x = (r(2, 1) - r(1, 2)) / s;
    q.y = (r(0, 2) - r(2, 0)) / s;
    q.z = (r(1, 0) - r(0, 1)) / s;
  } else if (r(0, 0) > r(1, 1) && r(0, 0) > r(2, 2)) {
    const sfc::Real s = std::sqrt(static_cast<sfc::Real>(1.0) + r(0, 0) -
                                  r(1, 1) - r(2, 2)) *
                        static_cast<sfc::Real>(2.0);
    q.w = (r(2, 1) - r(1, 2)) / s;
    q.x = static_cast<sfc::Real>(0.25) * s;
    q.y = (r(0, 1) + r(1, 0)) / s;
    q.z = (r(0, 2) + r(2, 0)) / s;
  } else if (r(1, 1) > r(2, 2)) {
    const sfc::Real s = std::sqrt(static_cast<sfc::Real>(1.0) + r(1, 1) -
                                  r(0, 0) - r(2, 2)) *
                        static_cast<sfc::Real>(2.0);
    q.w = (r(0, 2) - r(2, 0)) / s;
    q.x = (r(0, 1) + r(1, 0)) / s;
    q.y = static_cast<sfc::Real>(0.25) * s;
    q.z = (r(1, 2) + r(2, 1)) / s;
  } else {
    const sfc::Real s = std::sqrt(static_cast<sfc::Real>(1.0) + r(2, 2) -
                                  r(0, 0) - r(1, 1)) *
                        static_cast<sfc::Real>(2.0);
    q.w = (r(1, 0) - r(0, 1)) / s;
    q.x = (r(0, 2) + r(2, 0)) / s;
    q.y = (r(1, 2) + r(2, 1)) / s;
    q.z = static_cast<sfc::Real>(0.25) * s;
  }
  return q.normalized();
}

inline Quaternion Quaternion::operator+(const Quaternion& other) const {
  if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      !std::isfinite(other.w) || !std::isfinite(other.x) ||
      !std::isfinite(other.y) || !std::isfinite(other.z)) {
    throw std::runtime_error("Quaternion contains non-finite value");
  }
  return Quaternion{w + other.w, x + other.x, y + other.y, z + other.z};
}

inline Quaternion Quaternion::operator-(const Quaternion& other) const {
  if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      !std::isfinite(other.w) || !std::isfinite(other.x) ||
      !std::isfinite(other.y) || !std::isfinite(other.z)) {
    throw std::runtime_error("Quaternion contains non-finite value");
  }
  return Quaternion{w - other.w, x - other.x, y - other.y, z - other.z};
}

inline Quaternion Quaternion::operator*(const Quaternion& other) const {
  if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      !std::isfinite(other.w) || !std::isfinite(other.x) ||
      !std::isfinite(other.y) || !std::isfinite(other.z)) {
    throw std::runtime_error("Quaternion contains non-finite value");
  }
  return Quaternion{
      w * other.w - x * other.x - y * other.y - z * other.z,
      w * other.x + x * other.w + y * other.z - z * other.y,
      w * other.y - x * other.z + y * other.w + z * other.x,
      w * other.z + x * other.y - y * other.x + z * other.w};
}

inline sfc::Real Quaternion::norm() const {
  if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    throw std::runtime_error("Quaternion contains non-finite value");
  }
  return std::sqrt(w * w + x * x + y * y + z * z);
}

inline Quaternion Quaternion::normalized() const {
  const sfc::Real n = norm();
  if (n == static_cast<sfc::Real>(0.0)) {
    throw std::runtime_error("Quaternion normalization failed: zero norm");
  }
  return Quaternion{w / n, x / n, y / n, z / n};
}

inline Quaternion Quaternion::conjugate() const {
  if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    throw std::runtime_error("Quaternion contains non-finite value");
  }
  return Quaternion{w, -x, -y, -z};
}

inline Quaternion Quaternion::inverse() const {
  // For unit quaternions, inverse equals conjugate.
  return conjugate();
}

inline Vector3 RotationMatrix2RPY(const RotationMatrix& r) {
  if (!r.isFinite()) {
    throw std::runtime_error("RotationMatrix contains non-finite value");
  }

  const sfc::Real pitch_raw = -r(2, 0);
  const sfc::Real pitch = std::asin(sfc::clampReal(pitch_raw,
                                                   static_cast<sfc::Real>(-1.0),
                                                   static_cast<sfc::Real>(1.0)));
  const sfc::Real roll = std::atan2(r(2, 1), r(2, 2));
  const sfc::Real yaw = std::atan2(r(1, 0), r(0, 0));

  if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw)) {
    throw std::runtime_error("RotationMatrix2RPY produced non-finite value");
  }

  Vector3 rpy{};
  rpy(0) = roll;
  rpy(1) = pitch;
  rpy(2) = yaw;
  return rpy;
}

inline Vector3 Quaternion2RPY(const Quaternion& q) {
  const RotationMatrix r = RotationMatrix::fromQuaternion(q);
  return RotationMatrix2RPY(r);
}

inline Vector3 rpyFromRotationMatrix(const RotationMatrix& r) {
  return RotationMatrix2RPY(r);
}

inline Vector3 rpyFromQuaternion(const Quaternion& q) {
  return Quaternion2RPY(q);
}

}  // namespace sfc

#endif  // SFC_UTILTS_ROTATION_H_
