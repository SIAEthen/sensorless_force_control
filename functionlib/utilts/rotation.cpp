#include "utilts/rotation.h"

sfc::Real RotationMatrix::operator()(std::size_t row, std::size_t col) const {
  return m(row, col);
}

sfc::Real& RotationMatrix::operator()(std::size_t row, std::size_t col) {
  return m(row, col);
}

bool RotationMatrix::isFinite() const { return m.isFinite(); }

RotationMatrix RotationMatrix::identity() {
  RotationMatrix r{};
  r(0, 0) = 1.0;
  r(1, 1) = 1.0;
  r(2, 2) = 1.0;
  return r;
}

RotationMatrix RotationMatrix::fromRPY(sfc::Real roll, sfc::Real pitch, sfc::Real yaw) {
  RotationMatrix r{};
  r.m = rotationMatrixFromRPY(roll, pitch, yaw);
  return r;
}

RotationMatrix RotationMatrix::fromRPY(const RPY& rpy) {
  return fromRPY(rpy.roll, rpy.pitch, rpy.yaw);
}

RotationMatrix RotationMatrix::fromQuaternion(const Quaternion& q) {
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

RotationMatrix RotationMatrix::operator*(const RotationMatrix& other) const {
  RotationMatrix result{};
  result.m = m * other.m;
  return result;
}

RotationMatrix RotationMatrix::transpose() const {
  RotationMatrix r{};
  r.m = m.transpose();
  return r;
}

sfc::Real HomogeneousMatrix::operator()(std::size_t row, std::size_t col) const {
  return m(row, col);
}

sfc::Real& HomogeneousMatrix::operator()(std::size_t row, std::size_t col) {
  return m(row, col);
}

bool HomogeneousMatrix::isFinite() const { return m.isFinite(); }

HomogeneousMatrix HomogeneousMatrix::identity() {
  HomogeneousMatrix t{};
  t(0, 0) = 1.0;
  t(1, 1) = 1.0;
  t(2, 2) = 1.0;
  t(3, 3) = 1.0;
  return t;
}

HomogeneousMatrix HomogeneousMatrix::fromRotationTranslation(const RotationMatrix& r,
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

HomogeneousMatrix HomogeneousMatrix::operator*(const HomogeneousMatrix& other) const {
  HomogeneousMatrix result{};
  result.m = m * other.m;
  return result;
}

RotationMatrix HomogeneousMatrix::rotation() const {
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

Vector3 HomogeneousMatrix::translation() const {
  if (!isFinite()) {
    throw std::runtime_error("HomogeneousMatrix contains non-finite value");
  }
  Vector3 p{};
  p(0) = m(0, 3);
  p(1) = m(1, 3);
  p(2) = m(2, 3);
  return p;
}

Vector3 HomogeneousMatrix::zAxis() const {
  if (!isFinite()) {
    throw std::runtime_error("HomogeneousMatrix contains non-finite value");
  }
  Vector3 z{};
  z(0) = m(0, 2);
  z(1) = m(1, 2);
  z(2) = m(2, 2);
  return z;
}

Quaternion Quaternion::identity() { return Quaternion{1.0, 0.0, 0.0, 0.0}; }

Quaternion Quaternion::fromRPY(sfc::Real roll, sfc::Real pitch, sfc::Real yaw) {
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

Quaternion Quaternion::fromRPY(const RPY& rpy) {
  return fromRPY(rpy.roll, rpy.pitch, rpy.yaw);
}

Quaternion Quaternion::fromRotationMatrix(const RotationMatrix& r) {
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

Quaternion Quaternion::operator+(const Quaternion& other) const {
  if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      !std::isfinite(other.w) || !std::isfinite(other.x) ||
      !std::isfinite(other.y) || !std::isfinite(other.z)) {
    throw std::runtime_error("Quaternion contains non-finite value");
  }
  return Quaternion{w + other.w, x + other.x, y + other.y, z + other.z};
}

Quaternion Quaternion::operator-(const Quaternion& other) const {
  if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      !std::isfinite(other.w) || !std::isfinite(other.x) ||
      !std::isfinite(other.y) || !std::isfinite(other.z)) {
    throw std::runtime_error("Quaternion contains non-finite value");
  }
  return Quaternion{w - other.w, x - other.x, y - other.y, z - other.z};
}

Quaternion Quaternion::operator*(const Quaternion& other) const {
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

sfc::Real Quaternion::norm() const {
  if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    throw std::runtime_error("Quaternion contains non-finite value");
  }
  return std::sqrt(w * w + x * x + y * y + z * z);
}

Quaternion Quaternion::normalized() const {
  const sfc::Real n = norm();
  if (n == static_cast<sfc::Real>(0.0)) {
    throw std::runtime_error("Quaternion normalization failed: zero norm");
  }
  return Quaternion{w / n, x / n, y / n, z / n};
}

Quaternion Quaternion::conjugate() const {
  if (!std::isfinite(w) || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    throw std::runtime_error("Quaternion contains non-finite value");
  }
  return Quaternion{w, -x, -y, -z};
}

Quaternion Quaternion::inverse() const {
  // For unit quaternions, inverse equals conjugate.
  return conjugate();
}

Matrix3 rotationMatrixFromRPY(sfc::Real roll, sfc::Real pitch, sfc::Real yaw) {
  const sfc::Real cr = std::cos(roll);
  const sfc::Real sr = std::sin(roll);
  const sfc::Real cp = std::cos(pitch);
  const sfc::Real sp = std::sin(pitch);
  const sfc::Real cy = std::cos(yaw);
  const sfc::Real sy = std::sin(yaw);

  Matrix3 r{};
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

Vector3 RotationMatrix2RPY(const RotationMatrix& r) {
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

Vector3 Quaternion2RPY(const Quaternion& q) {
  const RotationMatrix r = RotationMatrix::fromQuaternion(q);
  return RotationMatrix2RPY(r);
}

RPY RPY::fromRotationMatrix(const RotationMatrix& r) {
  const Vector3 rpy = RotationMatrix2RPY(r);
  return RPY{rpy(0), rpy(1), rpy(2)};
}

RPY RPY::fromQuaternion(const Quaternion& q) {
  const Vector3 rpy = Quaternion2RPY(q);
  return RPY{rpy(0), rpy(1), rpy(2)};
}
