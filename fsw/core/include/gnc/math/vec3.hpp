#pragma once

#include <cmath>

namespace gnc {

/// A 3-vector. Plain aggregate: fixed size, no heap, trivially copyable.
/// Flight software uses Vec3f; the simulator uses Vec3d.
template <typename T>
struct Vec3 {
  T x{};
  T y{};
  T z{};
};

using Vec3f = Vec3<float>;
using Vec3d = Vec3<double>;

template <typename T>
constexpr Vec3<T> operator+(const Vec3<T>& a, const Vec3<T>& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

template <typename T>
constexpr Vec3<T> operator-(const Vec3<T>& a, const Vec3<T>& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

template <typename T>
constexpr Vec3<T> operator-(const Vec3<T>& a) {
  return {-a.x, -a.y, -a.z};
}

template <typename T>
constexpr Vec3<T> operator*(T s, const Vec3<T>& a) {
  return {s * a.x, s * a.y, s * a.z};
}

template <typename T>
constexpr Vec3<T> operator*(const Vec3<T>& a, T s) {
  return s * a;
}

template <typename T>
constexpr Vec3<T> operator/(const Vec3<T>& a, T s) {
  return {a.x / s, a.y / s, a.z / s};
}

template <typename T>
constexpr T dot(const Vec3<T>& a, const Vec3<T>& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Right-handed cross product: cross(x_hat, y_hat) == z_hat.
template <typename T>
constexpr Vec3<T> cross(const Vec3<T>& a, const Vec3<T>& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

template <typename T>
T norm(const Vec3<T>& a) {
  return std::sqrt(dot(a, a));
}

/// Unit vector along a. Precondition: a is not the zero vector.
template <typename T>
Vec3<T> normalized(const Vec3<T>& a) {
  return a / norm(a);
}

}  // namespace gnc
