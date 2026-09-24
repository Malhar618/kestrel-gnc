#pragma once

#include <array>
#include <cstddef>

#include "gnc/math/vec3.hpp"

namespace gnc {

/// A 3x3 matrix stored row-major in a std::array. Used for direction cosine
/// matrices (DCMs) and inertia tensors.
template <typename T>
struct Mat3 {
  std::array<T, 9> m{};

  constexpr T& operator()(std::size_t row, std::size_t col) { return m[3 * row + col]; }
  constexpr const T& operator()(std::size_t row, std::size_t col) const { return m[3 * row + col]; }

  static constexpr Mat3 identity() {
    // clang-format off
    return {{T(1), T(0), T(0),
             T(0), T(1), T(0),
             T(0), T(0), T(1)}};
    // clang-format on
  }
};

using Mat3f = Mat3<float>;
using Mat3d = Mat3<double>;

template <typename T>
constexpr Vec3<T> operator*(const Mat3<T>& a, const Vec3<T>& v) {
  return {a(0, 0) * v.x + a(0, 1) * v.y + a(0, 2) * v.z,
          a(1, 0) * v.x + a(1, 1) * v.y + a(1, 2) * v.z,
          a(2, 0) * v.x + a(2, 1) * v.y + a(2, 2) * v.z};
}

template <typename T>
constexpr Mat3<T> operator*(const Mat3<T>& a, const Mat3<T>& b) {
  Mat3<T> out{};
  for (std::size_t r = 0; r < 3; ++r) {
    for (std::size_t c = 0; c < 3; ++c) {
      out(r, c) = a(r, 0) * b(0, c) + a(r, 1) * b(1, c) + a(r, 2) * b(2, c);
    }
  }
  return out;
}

template <typename T>
constexpr Mat3<T> transposed(const Mat3<T>& a) {
  Mat3<T> out{};
  for (std::size_t r = 0; r < 3; ++r) {
    for (std::size_t c = 0; c < 3; ++c) {
      out(r, c) = a(c, r);
    }
  }
  return out;
}

template <typename T>
constexpr T determinant(const Mat3<T>& a) {
  return a(0, 0) * (a(1, 1) * a(2, 2) - a(1, 2) * a(2, 1)) -
         a(0, 1) * (a(1, 0) * a(2, 2) - a(1, 2) * a(2, 0)) +
         a(0, 2) * (a(1, 0) * a(2, 1) - a(1, 1) * a(2, 0));
}

/// Inverse via the adjugate. Precondition: determinant(a) != 0.
template <typename T>
constexpr Mat3<T> inverse(const Mat3<T>& a) {
  const T inv_det = T(1) / determinant(a);
  Mat3<T> out{};
  out(0, 0) = (a(1, 1) * a(2, 2) - a(1, 2) * a(2, 1)) * inv_det;
  out(0, 1) = (a(0, 2) * a(2, 1) - a(0, 1) * a(2, 2)) * inv_det;
  out(0, 2) = (a(0, 1) * a(1, 2) - a(0, 2) * a(1, 1)) * inv_det;
  out(1, 0) = (a(1, 2) * a(2, 0) - a(1, 0) * a(2, 2)) * inv_det;
  out(1, 1) = (a(0, 0) * a(2, 2) - a(0, 2) * a(2, 0)) * inv_det;
  out(1, 2) = (a(0, 2) * a(1, 0) - a(0, 0) * a(1, 2)) * inv_det;
  out(2, 0) = (a(1, 0) * a(2, 1) - a(1, 1) * a(2, 0)) * inv_det;
  out(2, 1) = (a(0, 1) * a(2, 0) - a(0, 0) * a(2, 1)) * inv_det;
  out(2, 2) = (a(0, 0) * a(1, 1) - a(0, 1) * a(1, 0)) * inv_det;
  return out;
}

/// Diagonal matrix with d on the diagonal.
template <typename T>
constexpr Mat3<T> diag(const Vec3<T>& d) {
  // clang-format off
  return {{d.x,  T(0), T(0),
           T(0), d.y,  T(0),
           T(0), T(0), d.z}};
  // clang-format on
}

/// Cross-product matrix: skew(a) * b == cross(a, b).
template <typename T>
constexpr Mat3<T> skew(const Vec3<T>& a) {
  // clang-format off
  return {{ T(0), -a.z,   a.y,
            a.z,   T(0), -a.x,
           -a.y,   a.x,   T(0)}};
  // clang-format on
}

}  // namespace gnc
