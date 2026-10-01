#pragma once

#include <array>

#include "gnc/angles.hpp"
#include "gnc/math/mat3.hpp"
#include "gnc/math/vec3.hpp"
#include "gnc/types.hpp"

namespace gnc {

/// The flight software's model of the vehicle. It is deliberately separate from the
/// simulator's truth parameters: flight code never reads sim/, and Monte Carlo runs will
/// make the two disagree on purpose.
struct QuadModel {
  real mass_kg = 0;
  Mat3f inertia_b_kgm2 = Mat3f::identity();
  std::array<Vec3f, kNumRotors> rotor_pos_b_m{};  // hub positions, PX4 Quad X order
  std::array<int, kNumRotors> rotor_spin{};       // +1 CCW, -1 CW seen from above
  real thrust_coeff = 0;                          // T = thrust_coeff * w^2
  real torque_coeff = 0;                          // Q = torque_coeff * w^2
  real rotor_speed_max_radps = 0;                 // rotor speed at a command of 1

  real max_rotor_thrust_N() const {
    return thrust_coeff * rotor_speed_max_radps * rotor_speed_max_radps;
  }
};

/// Model of the default 1.5 kg, 450 mm class quad (matches sim::default_quad_params()).
QuadModel default_quad_model();

}  // namespace gnc
