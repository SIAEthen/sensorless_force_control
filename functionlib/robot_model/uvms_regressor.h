#ifndef SFC_UVMS_REGRESSOR_H_
#define SFC_UVMS_REGRESSOR_H_

#include "functionlib/utilts/matrix.h"
#include "functionlib/utilts/rotation.h"
#include "functionlib/utilts/vector.h"

namespace sfc {

inline Matrix<6, 4> RegressorStupid(const Matrix3& r) {
  const Vector3 z{0.0, 0.0, 1.0};
  const Vector3 rz = r.transpose() * z;

  Matrix<3, 1> minus_rz{};
  minus_rz(0, 0) = -rz(0);
  minus_rz(1, 0) = -rz(1);
  minus_rz(2, 0) = -rz(2);

  const auto top = hstack(minus_rz, zeros<3, 3>());
  const auto bottom = hstack(zeros<3, 1>(), skew(rz));
  return vstack(top, bottom);
}

inline Matrix<6, 4> RegressorStupid(const RotationMatrix& r) {
  return RegressorStupid(r.m);
}

// MATLAB-compatible naming.
inline Matrix<6, 4> Regressor_stupid(const Matrix3& r) {
  return RegressorStupid(r);
}

// MATLAB-compatible naming.
inline Matrix<6, 4> Regressor_stupid(const RotationMatrix& r) {
  return RegressorStupid(r);
}

inline Matrix<6, 6> UMat(const Matrix3& r_b_a, const Vector3& r_ab_a) {
  const Matrix3 s_r = skew(r_ab_a);
  const auto top = hstack(r_b_a, zeros<3, 3>());
  const auto bottom = hstack(s_r * r_b_a, r_b_a);
  return vstack(top, bottom);
}

inline Matrix<6, 6> UMat(const RotationMatrix& r_b_a, const Vector3& r_ab_a) {
  return UMat(r_b_a.m, r_ab_a);
}

// MATLAB-compatible naming.
inline Matrix<6, 6> U_mat(const Matrix3& r_b_a, const Vector3& r_ab_a) {
  return UMat(r_b_a, r_ab_a);
}

// MATLAB-compatible naming.
inline Matrix<6, 6> U_mat(const RotationMatrix& r_b_a, const Vector3& r_ab_a) {
  return UMat(r_b_a, r_ab_a);
}

template <std::size_t Cols>
inline void assignBlock6x4(Matrix<6, Cols>& dst,
                           std::size_t start_col,
                           const Matrix<6, 4>& src) {
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t col = 0; col < 4; ++col) {
      dst(row, start_col + col) = src(row, col);
    }
  }
}

template <typename ArmT>
inline Matrix<6, 4 * (ArmT::kDof + 1)> regressor_uvms(const RotationMatrix& r_b_inertia,
                                                       const ArmT& arm,
                                                       const HomogeneousMatrix& t_0_b) {
  constexpr std::size_t n = ArmT::kDof;
  Matrix<6, 4 * (n + 1)> y{};

  const RotationMatrix r_0_b = t_0_b.rotation();
  const Vector3 p_b0_b = t_0_b.translation();
  const RotationMatrix r_0_inertia = r_b_inertia * r_0_b;

  assignBlock6x4(y, 0, Regressor_stupid(r_b_inertia));

  const Matrix<6, 6> u_0_b = U_mat(r_0_b, p_b0_b);
  const auto t_list = arm.jointTransforms();
  for (std::size_t i = 0; i < n; ++i) {
    const HomogeneousMatrix& t_i_0 = t_list[i];
    const RotationMatrix r_i_0 = t_i_0.rotation();
    const Vector3 p_0i_0 = t_i_0.translation();
    const RotationMatrix r_i_inertia = r_0_inertia * r_i_0;

    const Matrix<6, 6> u_i_0 = U_mat(r_i_0, p_0i_0);
    const Matrix<6, 4> block = (u_0_b * u_i_0) * Regressor_stupid(r_i_inertia);
    assignBlock6x4(y, 4 * (i + 1), block);
  }
  return y;
}

template <typename ArmT>
inline Matrix<6, 4 * (ArmT::kDof + 1)> regressor_uvms(const Vector3& rpy,
                                                       const ArmT& arm,
                                                       const HomogeneousMatrix& t_0_b) {
  const RotationMatrix r_b_inertia = RotationMatrix::fromRPY(rpy);
  return regressor_uvms(r_b_inertia, arm, t_0_b);
}


template <typename ArmT>
inline Matrix<6, 4 * (ArmT::kDof + 1)> regressor_girona1000(const Vector3& rpy,
                                                             const ArmT& arm,
                                                             const HomogeneousMatrix& t_0_b) {
  return regressor_uvms(rpy, arm, t_0_b);
}

template <typename ArmT>
inline Matrix<6, 4 * (ArmT::kDof + 1)> regressor_girona1000(const RotationMatrix& r_b_inertia,
                                                             const ArmT& arm,
                                                             const HomogeneousMatrix& t_0_b) {
  return regressor_uvms(r_b_inertia, arm, t_0_b);
}

template <typename ArmT>
inline Vector6 get_girona_model(
    const RotationMatrix& r_b_inertia,
    const ArmT& arm,
    const HomogeneousMatrix& t_0_b,
    const Vector<4 * (ArmT::kDof + 1)>& theta) {
  const Matrix<6, 4 * (ArmT::kDof + 1)> y = regressor_uvms(r_b_inertia, arm, t_0_b);
  return y * theta;
}

template <typename ArmT>
inline Vector6 get_girona_model(
    const Vector3& rpy,
    const ArmT& arm,
    const HomogeneousMatrix& t_0_b,
    const Vector<4 * (ArmT::kDof + 1)>& theta) {
  const Matrix<6, 4 * (ArmT::kDof + 1)> y = regressor_uvms(rpy, arm, t_0_b);
  return y * theta;
}

}  // namespace sfc

#endif  // SFC_UVMS_REGRESSOR_H_
