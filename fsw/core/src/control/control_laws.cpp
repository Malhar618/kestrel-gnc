#include "gnc/control/control_laws.hpp"

#include <algorithm>
#include <cmath>

#include "gnc/math/mat3.hpp"

namespace gnc {

Vec3f attitude_rate_setpoint(const Quatf& q_nb, const Quatf& q_sp, const AttitudeGains& gains) {
  const Vec3f error_b = to_rotation_vector(conjugate(q_nb) * q_sp);
  const Vec3f& lim = gains.rate_max_radps;
  return {std::clamp(gains.kp.x * error_b.x, -lim.x, lim.x),
          std::clamp(gains.kp.y * error_b.y, -lim.y, lim.y),
          std::clamp(gains.kp.z * error_b.z, -lim.z, lim.z)};
}

ThrustAttitude thrust_to_attitude(const Vec3f& acc_sp_ned_mps2, real yaw_sp_rad, const Quatf& q_nb,
                                  real mass_kg, real gravity_mps2, real tilt_max_rad) {
  ThrustAttitude out;
  Vec3f force = mass_kg * (acc_sp_ned_mps2 - Vec3f{0, 0, gravity_mps2});

  // Always keep some upward thrust (10% of weight), so the attitude stays defined.
  const real min_up = real(0.1) * mass_kg * gravity_mps2;
  if (-force.z < min_up) force.z = -min_up;

  // Limit tilt: |horizontal| <= up * tan(tilt_max).
  const real up = -force.z;
  const real horizontal = std::hypot(force.x, force.y);
  const real horizontal_max = up * std::tan(tilt_max_rad);
  if (horizontal > horizontal_max) {
    const real scale = horizontal_max / horizontal;
    force.x *= scale;
    force.y *= scale;
    out.tilt_limited = true;
  }

  // Desired body axes in NED: z along -F, y perpendicular to z and the heading, x = y x z.
  const Vec3f z_b = -(force / norm(force));
  const Vec3f heading{std::cos(yaw_sp_rad), std::sin(yaw_sp_rad), 0};
  const Vec3f y_b = normalized(cross(z_b, heading));
  const Vec3f x_b = cross(y_b, z_b);
  Mat3f c_nb{};  // columns are the body axes expressed in NED
  c_nb(0, 0) = x_b.x;
  c_nb(1, 0) = x_b.y;
  c_nb(2, 0) = x_b.z;
  c_nb(0, 1) = y_b.x;
  c_nb(1, 1) = y_b.y;
  c_nb(2, 1) = y_b.z;
  c_nb(0, 2) = z_b.x;
  c_nb(1, 2) = z_b.y;
  c_nb(2, 2) = z_b.z;
  out.q_sp = from_dcm(c_nb);

  const Vec3f z_b_now = rotate(q_nb, Vec3f{0, 0, 1});
  out.thrust_N = std::max(dot(force, -z_b_now), real(0));
  return out;
}

}  // namespace gnc
