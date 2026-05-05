#pragma once
#include <cmath>
#include <utility>
#include "matrix.h"
#include "linear_algebra.h"

namespace sfc {

// LU-based determinanterminant for compile-time N×N matrix.
template <std::size_t N>
sfc::Real determinant(const sfc::Matrix<N, N>& a) {
    sfc::Matrix<N, N> m = a;
    sfc::Real d = sfc::Real(1.0);
    for (std::size_t i = 0; i < N; ++i) {
        std::size_t pivot = i;
        sfc::Real max_val = std::fabs(m(i, i));
        for (std::size_t row = i + 1; row < N; ++row) {
            const sfc::Real val = std::fabs(m(row, i));
            if (val > max_val) { max_val = val; pivot = row; }
        }
        if (pivot != i) {
            for (std::size_t col = 0; col < N; ++col)
                std::swap(m(i, col), m(pivot, col));
            d = -d;
        }
        if (m(i, i) == sfc::Real(0.0)) return sfc::Real(0.0);
        d *= m(i, i);
        const sfc::Real inv_diag = sfc::Real(1.0) / m(i, i);
        for (std::size_t row = i + 1; row < N; ++row) {
            const sfc::Real factor = m(row, i) * inv_diag;
            for (std::size_t col = i + 1; col < N; ++col)
                m(row, col) -= factor * m(i, col);
        }
    }
    return d;
}

// Manipulability measure: w = sqrt(determinant(J * J^T))
template <std::size_t R, std::size_t C>
sfc::Real cal_manipulability(const sfc::Matrix<R, C>& Jacobian) {
    const sfc::Matrix<R, R> JJT = Jacobian * Jacobian.transpose();
    const sfc::Real d = determinant(JJT);
    if (d <= sfc::Real(0.0)) return sfc::Real(0.0);
    return std::sqrt(d);
}

}  // namespace sfc
