#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include "gnc/math/quat.hpp"

namespace gnc {

/// Aerospace 3-2-1 Euler angles (rad): yaw about z, then pitch about the new y,
/// then roll about the new x. For logging, plots and human input only. The
/// flight code works in quaternions because Euler angles hit gimbal lock at
/// pitch = ±90 deg.
template <typename T>
struct Euler321 {
  T roll{};
  T pitch{};
  T yaw{};
};

using Euler321f = Euler321<float>;
using Euler321d = Euler321<double>;

/// q_nb = q_z(yaw) ⊗ q_y(pitch) ⊗ q_x(roll).
template <typename T>
Quat<T> from_euler321(const Euler321<T>& e) {
  const Quat<T> qz = from_axis_angle(Vec3<T>{T(0), T(0), T(1)}, e.yaw);
  const Quat<T> qy = from_axis_angle(Vec3<T>{T(0), T(1), T(0)}, e.pitch);
  const Quat<T> qx = from_axis_angle(Vec3<T>{T(1), T(0), T(0)}, e.roll);
  return qz * qy * qx;
}

/// At gimbal lock only yaw - roll (pitch = +90 deg) or yaw + roll (pitch = -90 deg)
/// is observable. There we report roll = 0 and put the whole rotation in yaw.
template <typename T>
Euler321<T> to_euler321(const Quat<T>& q) {
  const T sin_pitch = std::clamp(T(2) * (q.w * q.y - q.x * q.z), T(-1), T(1));
  if (std::abs(sin_pitch) >= T(1) - std::numeric_limits<T>::epsilon()) {
    const T half_pi = std::numbers::pi_v<T> / T(2);
    return {T(0), std::copysign(half_pi, sin_pitch),
            std::atan2(T(2) * (q.w * q.z - q.x * q.y), T(1) - T(2) * (q.x * q.x + q.z * q.z))};
  }
  return {std::atan2(T(2) * (q.w * q.x + q.y * q.z), T(1) - T(2) * (q.x * q.x + q.y * q.y)),
          std::asin(sin_pitch),
          std::atan2(T(2) * (q.w * q.z + q.x * q.y), T(1) - T(2) * (q.y * q.y + q.z * q.z))};
}

}  // namespace gnc
