#pragma once

#include <array>

#include "gnc/angles.hpp"
#include "gnc/control/quad_model.hpp"
#include "gnc/math/vec3.hpp"
#include "gnc/types.hpp"

namespace gnc {

/// Collective thrust (along -z body, N) and torque about the CG in body axes (N m).
struct ThrustTorque {
  real thrust_N = 0;
  Vec3f torque_b_Nm{};
};

struct MixerResult {
  MotorOutputs outputs{};
  std::array<real, kNumRotors> rotor_thrust_N{};
  ThrustTorque achieved{};  // what the rotors deliver after any saturation handling
  bool saturated = false;   // true if any part of the request was reduced
};

/// Control allocation: turns a thrust/torque request into rotor thrusts and then into
/// normalized motor commands. When the rotors can't deliver the request, it gives up
/// yaw first, then collective thrust, and roll/pitch last, since tilt is what keeps the
/// vehicle upright and pointed where it needs to go.
class Mixer {
 public:
  explicit Mixer(const QuadModel& model);

  /// False if the rotor geometry can't produce independent thrust and torques.
  bool valid() const { return valid_; }

  MixerResult mix(const ThrustTorque& request) const;

  /// Thrust and torque produced by a set of rotor thrusts (the allocation matrix itself).
  ThrustTorque wrench_of(const std::array<real, kNumRotors>& rotor_thrust_N) const;

 private:
  QuadModel model_;
  std::array<real, 16> alloc_{};      // [T, tx, ty, tz] = alloc * f, row-major 4x4
  std::array<real, 16> alloc_inv_{};  // f = alloc_inv * [T, tx, ty, tz]
  bool valid_ = false;
};

}  // namespace gnc
