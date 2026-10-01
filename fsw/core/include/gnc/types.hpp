#pragma once

#include <array>
#include <cstddef>

#include "gnc/angles.hpp"
#include "gnc/math/quat.hpp"
#include "gnc/math/vec3.hpp"

namespace gnc {

inline constexpr std::size_t kNumRotors = 4;

/// What the flight software believes about the vehicle: truth in M2, the EKF from M3 on.
struct VehicleState {
  Vec3f pos_ned_m{};
  Vec3f vel_ned_mps{};
  Quatf q_nb = Quatf::identity();
  Vec3f omega_b_radps{};
};

/// Normalized motor commands in PX4 motor order: 0 = stopped, 1 = full speed.
struct MotorOutputs {
  std::array<real, kNumRotors> u{};
};

}  // namespace gnc
