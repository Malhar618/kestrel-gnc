#pragma once

#include "gnc/math/vec3.hpp"

namespace sim {

/// Standard gravity (m/s^2).
inline constexpr double kStandardGravity = 9.80665;

/// The world the vehicle flies in.
struct Environment {
  double gravity_mps2 = kStandardGravity;  // acts along +z (down) in NED
  gnc::Vec3d wind_ned_mps{};               // velocity of the air mass, NED
};

}  // namespace sim
