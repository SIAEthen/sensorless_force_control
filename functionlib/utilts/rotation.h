#ifndef SFC_UTILTS_ROTATION_H_
#define SFC_UTILTS_ROTATION_H_

#include <cstddef>

#include "config.h"
#include "utilts/matrix.h"
#include "utilts/real.h"
#include "utilts/vector.h"

struct Quaternion;
struct RotationMatrix;
struct HomogeneousMatrix;
struct RPY;

Matrix3 rotationMatrixFromRPY(sfc::Real roll, sfc::Real pitch, sfc::Real yaw);
Vector3 RotationMatrix2RPY(const RotationMatrix& r);
Vector3 Quaternion2RPY(const Quaternion& q);

struct RotationMatrix {
  Matrix3 m{};

  sfc::Real operator()(std::size_t row, std::size_t col) const;
  sfc::Real& operator()(std::size_t row, std::size_t col);
  RotationMatrix operator*(const RotationMatrix& other) const;
  bool isFinite() const;

  static RotationMatrix identity();
  static RotationMatrix fromRPY(sfc::Real roll, sfc::Real pitch, sfc::Real yaw);
  static RotationMatrix fromRPY(const RPY& rpy);
  static RotationMatrix fromQuaternion(const Quaternion& q);
  

  RotationMatrix transpose() const;
};

struct HomogeneousMatrix {
  Matrix4 m{};

  sfc::Real operator()(std::size_t row, std::size_t col) const;
  sfc::Real& operator()(std::size_t row, std::size_t col);
  HomogeneousMatrix operator*(const HomogeneousMatrix& other) const;
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
  static Quaternion fromRPY(const RPY& rpy);
  static Quaternion fromRotationMatrix(const RotationMatrix& r);

  Quaternion operator+(const Quaternion& other) const;
  Quaternion operator-(const Quaternion& other) const;
  Quaternion operator*(const Quaternion& other) const;
  sfc::Real norm() const;
  Quaternion normalized() const;
  Quaternion conjugate() const;
  Quaternion inverse() const;
};

struct RPY {
  sfc::Real roll{0.0};
  sfc::Real pitch{0.0};
  sfc::Real yaw{0.0};

  RPY() = default;
  RPY(sfc::Real r, sfc::Real p, sfc::Real y) : roll(r), pitch(p), yaw(y) {}

  static RPY fromRotationMatrix(const RotationMatrix& r);
  static RPY fromQuaternion(const Quaternion& q);
};

#endif  // SFC_UTILTS_ROTATION_H_
