#pragma once

#include "gnc/angles.hpp"
#include "gnc/math/quat.hpp"
#include "gnc/math/vec3.hpp"

namespace gnc {

struct AttitudeGains {
  Vec3f kp{};              // body rate per radian of attitude error (1/s)
  Vec3f rate_max_radps{};  // per-axis limit on the commanded body rate
};

/// Body-rate setpoint that turns q_nb toward q_sp. The error is the rotation from the
/// current attitude to the setpoint, conj(q_nb) ⊗ q_sp, which is expressed in current
/// body axes (right multiplication = body axes), so it maps straight onto body rates.
/// The log map takes the short way around, so a 180-degree error still has a direction.
Vec3f attitude_rate_setpoint(const Quatf& q_nb, const Quatf& q_sp, const AttitudeGains& gains);

struct ThrustAttitude {
  Quatf q_sp = Quatf::identity();  // attitude that points the thrust along the request
  real thrust_N = 0;               // collective thrust to command now
  bool tilt_limited = false;       // true if the horizontal request was cut back
};

/// Turns a desired NED acceleration and heading into an attitude and collective thrust.
/// The rotors push along -z body, so the thrust vector F = m (a_sp - g_n) must point
/// along -z_b: z_b = -F / |F|. Heading fixes the rest of the frame. The horizontal part
/// is limited so the tilt never exceeds tilt_max_rad, and the thrust is F projected on the
/// current -z body axis, so a vehicle that is still turning doesn't overshoot vertically.
/// tilt_max_rad is clamped to [0, 89 deg].
ThrustAttitude thrust_to_attitude(const Vec3f& acc_sp_ned_mps2, real yaw_sp_rad, const Quatf& q_nb,
                                  real mass_kg, real gravity_mps2, real tilt_max_rad);

}  // namespace gnc
