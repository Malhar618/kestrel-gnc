#pragma once

#include "gnc/math/mat3.hpp"
#include "gnc/math/quat.hpp"
#include "gnc/math/vec3.hpp"

namespace sim {

/// Mass properties about the center of gravity, in body axes. Use make_rigid_body()
/// so the stored inverse always matches the inertia.
struct RigidBodyParams {
  double mass_kg = 1.0;
  gnc::Mat3d inertia_b_kgm2 = gnc::Mat3d::identity();
  gnc::Mat3d inertia_inv_b = gnc::Mat3d::identity();
};

RigidBodyParams make_rigid_body(double mass_kg, const gnc::Mat3d& inertia_b_kgm2);

/// 6-DOF state of the center of gravity: NED position and velocity, attitude q_nb,
/// and body angular rate in body axes.
struct RigidBodyState {
  gnc::Vec3d pos_ned_m{};
  gnc::Vec3d vel_ned_mps{};
  gnc::Quatd q_nb = gnc::Quatd::identity();
  gnc::Vec3d omega_b_radps{};
};

// Vector-space operations so gnc::rk4_step() can integrate the state. A time
// derivative reuses this struct; its q_nb field then holds dq/dt, not a rotation.
RigidBodyState operator+(const RigidBodyState& a, const RigidBodyState& b);
RigidBodyState operator*(double s, const RigidBodyState& x);

/// Newton-Euler equations with body-frame force and torque (about the CG):
///   d(pos)/dt   = vel
///   d(vel)/dt   = C_nb * F_b / m + g_n
///   d(q_nb)/dt  = 1/2 q_nb ⊗ [0, omega_b]
///   d(omega)/dt = J^-1 (tau_b - omega_b x (J omega_b))
RigidBodyState rigid_body_derivative(const RigidBodyParams& p, const RigidBodyState& x,
                                     const gnc::Vec3d& force_b_N, const gnc::Vec3d& torque_b_Nm,
                                     const gnc::Vec3d& gravity_ned_mps2);

/// One RK4 step with the body-frame force and torque held constant, then q_nb is
/// renormalized.
RigidBodyState rigid_body_step(const RigidBodyParams& p, const RigidBodyState& x,
                               const gnc::Vec3d& force_b_N, const gnc::Vec3d& torque_b_Nm,
                               const gnc::Vec3d& gravity_ned_mps2, double dt_s);

/// 1/2 omega^T J omega (J).
double rotational_kinetic_energy(const RigidBodyParams& p, const RigidBodyState& x);

/// Angular momentum about the CG expressed in NED, C_nb J omega_b (kg m^2/s).
/// Constant when no torque acts.
gnc::Vec3d angular_momentum_ned(const RigidBodyParams& p, const RigidBodyState& x);

}  // namespace sim
