#pragma once

#include <cmath>

#include "gnc/math/mat3.hpp"
#include "gnc/math/vec3.hpp"

namespace gnc {

/// Unit quaternion for attitude. Conventions (see docs/conventions.md):
///   - Hamilton product, scalar-first storage [w, x, y, z].
///   - q_nb rotates body-frame vectors into the NED frame: v_n = q ⊗ v_b ⊗ q*.
///   - q and -q describe the same rotation (double cover).
template <typename T>
struct Quat {
  T w{1};
  T x{};
  T y{};
  T z{};

  static constexpr Quat identity() { return {T(1), T(0), T(0), T(0)}; }
  constexpr Vec3<T> vec() const { return {x, y, z}; }
};

using Quatf = Quat<float>;
using Quatd = Quat<double>;

/// Hamilton product. Composition reads right to left: q_nb = q_na ⊗ q_ab.
template <typename T>
constexpr Quat<T> operator*(const Quat<T>& a, const Quat<T>& b) {
  // clang-format off
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
          a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
  // clang-format on
}

// 4-vector arithmetic. Integrators need these; the results are not rotations
// until they are normalized again.
template <typename T>
constexpr Quat<T> operator+(const Quat<T>& a, const Quat<T>& b) {
  return {a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z};
}

template <typename T>
constexpr Quat<T> operator*(T s, const Quat<T>& q) {
  return {s * q.w, s * q.x, s * q.y, s * q.z};
}

template <typename T>
constexpr T dot(const Quat<T>& a, const Quat<T>& b) {
  return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

template <typename T>
T norm(const Quat<T>& q) {
  return std::sqrt(dot(q, q));
}

template <typename T>
Quat<T> normalized(const Quat<T>& q) {
  return (T(1) / norm(q)) * q;
}

/// Inverse of a unit quaternion.
template <typename T>
constexpr Quat<T> conjugate(const Quat<T>& q) {
  return {q.w, -q.x, -q.y, -q.z};
}

/// Picks the sign with w >= 0, so equal rotations compare equal.
template <typename T>
constexpr Quat<T> canonical(const Quat<T>& q) {
  return q.w < T(0) ? T(-1) * q : q;
}

/// Rotates v by q: returns q ⊗ [0, v] ⊗ q*, expanded so it costs two cross products.
template <typename T>
constexpr Vec3<T> rotate(const Quat<T>& q, const Vec3<T>& v) {
  const Vec3<T> u = q.vec();
  const Vec3<T> t = T(2) * cross(u, v);
  return v + q.w * t + cross(u, t);
}

/// Rotation of `angle` radians about the unit vector `axis` (right-hand rule).
template <typename T>
Quat<T> from_axis_angle(const Vec3<T>& axis, T angle) {
  const T half = angle / T(2);
  const Vec3<T> u = std::sin(half) * axis;
  return {std::cos(half), u.x, u.y, u.z};
}

/// Exponential map: rotation vector (axis * angle, rad) -> unit quaternion.
template <typename T>
Quat<T> from_rotation_vector(const Vec3<T>& rv) {
  const T angle = norm(rv);
  // sin(angle/2)/angle, with its Taylor series near zero to avoid 0/0.
  const T k = angle > T(1e-6) ? std::sin(angle / T(2)) / angle : T(0.5) - angle * angle / T(48);
  const Vec3<T> u = k * rv;
  return {std::cos(angle / T(2)), u.x, u.y, u.z};
}

/// Logarithm map: unit quaternion -> rotation vector with angle in [0, pi].
template <typename T>
Vec3<T> to_rotation_vector(const Quat<T>& q_in) {
  const Quat<T> q = canonical(q_in);
  const Vec3<T> u = q.vec();
  const T s = norm(u);
  const T k = s > T(1e-6) ? T(2) * std::atan2(s, q.w) / s : T(2) / q.w;
  return k * u;
}

/// Smallest angle (rad) that rotates a onto b.
template <typename T>
T angle_between(const Quat<T>& a, const Quat<T>& b) {
  return norm(to_rotation_vector(conjugate(a) * b));
}

/// Direction cosine matrix C_nb with v_n = C_nb * v_b (same rotation as q_nb).
template <typename T>
constexpr Mat3<T> to_dcm(const Quat<T>& q) {
  const T xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
  const T xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
  const T wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
  // clang-format off
  return {{T(1) - T(2) * (yy + zz), T(2) * (xy - wz), T(2) * (xz + wy),
           T(2) * (xy + wz), T(1) - T(2) * (xx + zz), T(2) * (yz - wx),
           T(2) * (xz - wy), T(2) * (yz + wx), T(1) - T(2) * (xx + yy)}};
  // clang-format on
}

/// DCM -> quaternion (Shepperd's method): solve for whichever of w, x, y, z is
/// largest first, so we never divide by a number near zero.
template <typename T>
Quat<T> from_dcm(const Mat3<T>& c) {
  const T tr = c(0, 0) + c(1, 1) + c(2, 2);
  Quat<T> q;
  if (tr > c(0, 0) && tr > c(1, 1) && tr > c(2, 2)) {
    const T s = T(2) * std::sqrt(T(1) + tr);  // s = 4w
    q = {s / T(4), (c(2, 1) - c(1, 2)) / s, (c(0, 2) - c(2, 0)) / s, (c(1, 0) - c(0, 1)) / s};
  } else if (c(0, 0) > c(1, 1) && c(0, 0) > c(2, 2)) {
    const T s = T(2) * std::sqrt(T(1) + c(0, 0) - c(1, 1) - c(2, 2));  // s = 4x
    q = {(c(2, 1) - c(1, 2)) / s, s / T(4), (c(0, 1) + c(1, 0)) / s, (c(0, 2) + c(2, 0)) / s};
  } else if (c(1, 1) > c(2, 2)) {
    const T s = T(2) * std::sqrt(T(1) + c(1, 1) - c(0, 0) - c(2, 2));  // s = 4y
    q = {(c(0, 2) - c(2, 0)) / s, (c(0, 1) + c(1, 0)) / s, s / T(4), (c(1, 2) + c(2, 1)) / s};
  } else {
    const T s = T(2) * std::sqrt(T(1) + c(2, 2) - c(0, 0) - c(1, 1));  // s = 4z
    q = {(c(1, 0) - c(0, 1)) / s, (c(0, 2) + c(2, 0)) / s, (c(1, 2) + c(2, 1)) / s, s / T(4)};
  }
  return normalized(q);
}

/// Attitude kinematics: dq/dt = 1/2 q ⊗ [0, omega_b], with omega_b the body
/// angular rate expressed in the body frame (what a gyro measures).
template <typename T>
constexpr Quat<T> derivative(const Quat<T>& q, const Vec3<T>& omega_b) {
  return T(0.5) * (q * Quat<T>{T(0), omega_b.x, omega_b.y, omega_b.z});
}

/// Propagates q through dt seconds of constant body rate. Exact for constant
/// omega_b, which is why the EKF uses it instead of Euler-stepping derivative().
template <typename T>
Quat<T> integrate(const Quat<T>& q, const Vec3<T>& omega_b, T dt) {
  return normalized(q * from_rotation_vector(dt * omega_b));
}

}  // namespace gnc
