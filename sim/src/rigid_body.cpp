#include "sim/rigid_body.hpp"

#include "gnc/math/rk4.hpp"

namespace sim {

using gnc::Mat3d;
using gnc::Vec3d;

RigidBodyParams make_rigid_body(double mass_kg, const Mat3d& inertia_b_kgm2) {
  return {mass_kg, inertia_b_kgm2, gnc::inverse(inertia_b_kgm2)};
}

RigidBodyState operator+(const RigidBodyState& a, const RigidBodyState& b) {
  return {a.pos_ned_m + b.pos_ned_m, a.vel_ned_mps + b.vel_ned_mps, a.q_nb + b.q_nb,
          a.omega_b_radps + b.omega_b_radps};
}

RigidBodyState operator*(double s, const RigidBodyState& x) {
  return {s * x.pos_ned_m, s * x.vel_ned_mps, s * x.q_nb, s * x.omega_b_radps};
}

RigidBodyState rigid_body_derivative(const RigidBodyParams& p, const RigidBodyState& x,
                                     const Vec3d& force_b_N, const Vec3d& torque_b_Nm,
                                     const Vec3d& gravity_ned_mps2) {
  const Vec3d& w = x.omega_b_radps;
  const Vec3d h_b = p.inertia_b_kgm2 * w;
  return {x.vel_ned_mps, gnc::rotate(x.q_nb, force_b_N) / p.mass_kg + gravity_ned_mps2,
          gnc::derivative(x.q_nb, w), p.inertia_inv_b * (torque_b_Nm - gnc::cross(w, h_b))};
}

RigidBodyState rigid_body_step(const RigidBodyParams& p, const RigidBodyState& x,
                               const Vec3d& force_b_N, const Vec3d& torque_b_Nm,
                               const Vec3d& gravity_ned_mps2, double dt_s) {
  const auto f = [&](double /*t*/, const RigidBodyState& s) {
    return rigid_body_derivative(p, s, force_b_N, torque_b_Nm, gravity_ned_mps2);
  };
  RigidBodyState next = gnc::rk4_step(f, 0.0, x, dt_s);
  next.q_nb = gnc::normalized(next.q_nb);
  return next;
}

double rotational_kinetic_energy(const RigidBodyParams& p, const RigidBodyState& x) {
  return 0.5 * gnc::dot(x.omega_b_radps, p.inertia_b_kgm2 * x.omega_b_radps);
}

Vec3d angular_momentum_ned(const RigidBodyParams& p, const RigidBodyState& x) {
  return gnc::rotate(x.q_nb, p.inertia_b_kgm2 * x.omega_b_radps);
}

}  // namespace sim
