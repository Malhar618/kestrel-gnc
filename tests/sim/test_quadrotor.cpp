#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

#include "gnc/math/mat3.hpp"
#include "gnc/math/quat.hpp"
#include "sim/quadrotor.hpp"
#include "test_support.hpp"

namespace sim::test {
namespace {

using gnc::Quatd;
using gnc::Vec3d;
using gnc::test::vec_near;

constexpr double kDt = 1e-3;

QuadState run(const QuadParams& p, QuadState x, const MotorCommand& cmd, const Environment& env,
              int steps) {
  for (int i = 0; i < steps; ++i) x = quad_step(p, x, cmd, env, kDt);
  return x;
}

// Angular acceleration at hover after scaling each rotor's speed.
Vec3d angular_accel(const QuadParams& p, const Environment& env,
                    const std::array<double, kNumRotors>& speed_scale) {
  QuadState x = hover_state(p, env);
  for (std::size_t i = 0; i < kNumRotors; ++i) x.rotor_speed_radps[i] *= speed_scale[i];
  return quad_derivative(p, x, hover_command(p, env), env).body.omega_b_radps;
}

TEST(Quadrotor, RotorLayoutMatchesPx4QuadX) {
  const auto r = quad_x_rotors(0.225);
  // Motor 1 front-right CCW, 2 rear-left CCW, 3 front-left CW, 4 rear-right CW.
  EXPECT_TRUE(r[0].pos_b_m.x > 0.0 && r[0].pos_b_m.y > 0.0 && r[0].spin == 1);
  EXPECT_TRUE(r[1].pos_b_m.x < 0.0 && r[1].pos_b_m.y < 0.0 && r[1].spin == 1);
  EXPECT_TRUE(r[2].pos_b_m.x > 0.0 && r[2].pos_b_m.y < 0.0 && r[2].spin == -1);
  EXPECT_TRUE(r[3].pos_b_m.x < 0.0 && r[3].pos_b_m.y > 0.0 && r[3].spin == -1);
  for (const Rotor& rotor : r) EXPECT_NEAR(gnc::norm(rotor.pos_b_m), 0.225, 1e-12);
}

TEST(Quadrotor, HoverTrimCarriesExactlyTheWeight) {
  const QuadParams p = default_quad_params();
  const Environment env{};
  const double weight = p.body.mass_kg * env.gravity_mps2;
  const double speed = hover_rotor_speed(p, env);
  EXPECT_NEAR(p.thrust_coeff * speed * speed / (weight / 4.0), 1.0, 1e-12);

  const Wrench w = quad_wrench(p, hover_state(p, env), env);
  EXPECT_TRUE(vec_near(w.force_b_N, Vec3d{0.0, 0.0, -weight}, 1e-9));
  EXPECT_TRUE(vec_near(w.torque_b_Nm, Vec3d{}, 1e-12));

  // Hover leaves headroom to maneuver without saturating.
  const double u = hover_command(p, env).u[0];
  EXPECT_GT(u, 0.3);
  EXPECT_LT(u, 0.8);
}

TEST(Quadrotor, HoverTrimHoldsPositionAndAttitudeFor10Seconds) {
  const QuadParams p = default_quad_params();
  const Environment env{};
  const QuadState x =
      run(p, hover_state(p, env, Vec3d{0.0, 0.0, -10.0}), hover_command(p, env), env, 10000);
  EXPECT_TRUE(vec_near(x.body.pos_ned_m, Vec3d{0.0, 0.0, -10.0}, 1e-9));
  EXPECT_LT(gnc::angle_between(x.body.q_nb, Quatd::identity()), 1e-12);
}

TEST(Quadrotor, MotorLagReaches63PercentAfterOneTimeConstant) {
  const QuadParams p = default_quad_params();
  const Environment env{};
  const QuadState x0 = hover_state(p, env);
  MotorCommand cmd = hover_command(p, env);
  for (double& u : cmd.u) u *= 1.2;
  const int steps = static_cast<int>(std::lround(p.motor_time_constant_s / kDt));
  const QuadState x = run(p, x0, cmd, env, steps);
  const double start = x0.rotor_speed_radps[0];
  const double target = cmd.u[0] * p.rotor_speed_max_radps;
  for (double speed : x.rotor_speed_radps) {
    EXPECT_NEAR((speed - start) / (target - start), 1.0 - std::exp(-1.0), 1e-6);
  }
}

TEST(Quadrotor, DifferentialThrustFollowsPx4Conventions) {
  const QuadParams p = default_quad_params();
  const Environment env{};

  // Front rotors (1, 3) faster: nose pitches up, nothing else.
  const Vec3d front = angular_accel(p, env, {1.05, 1.0, 1.05, 1.0});
  EXPECT_GT(front.y, 0.0);
  EXPECT_NEAR(front.x, 0.0, 1e-9);
  EXPECT_NEAR(front.z, 0.0, 1e-9);
  // Quantitatively: two extra thrusts at lever arm L/sqrt(2), divided by Jyy.
  const double weight = p.body.mass_kg * env.gravity_mps2;
  const double extra_thrust = weight / 4.0 * (1.05 * 1.05 - 1.0);
  const double lever = p.rotors[0].pos_b_m.x;
  EXPECT_NEAR(front.y, 2.0 * extra_thrust * lever / p.body.inertia_b_kgm2(1, 1), 1e-9);

  // Right rotors (1, 4) faster: right side rises, so the vehicle rolls left (negative roll).
  const Vec3d right = angular_accel(p, env, {1.05, 1.0, 1.0, 1.05});
  EXPECT_LT(right.x, 0.0);
  EXPECT_NEAR(right.x, -2.0 * extra_thrust * p.rotors[0].pos_b_m.y / p.body.inertia_b_kgm2(0, 0),
              1e-9);
  EXPECT_NEAR(right.y, 0.0, 1e-9);
  EXPECT_NEAR(right.z, 0.0, 1e-9);

  // CCW rotors (1, 2) faster: their reaction torque yaws the nose right (positive yaw).
  const Vec3d ccw = angular_accel(p, env, {1.05, 1.05, 1.0, 1.0});
  EXPECT_GT(ccw.z, 0.0);
  // Quantitatively: each rotor's reaction torque is (k_Q / k_T) times its thrust.
  const double extra_torque = p.torque_coeff / p.thrust_coeff * extra_thrust;
  EXPECT_NEAR(ccw.z, 2.0 * extra_torque / p.body.inertia_b_kgm2(2, 2), 1e-9);
  EXPECT_NEAR(ccw.x, 0.0, 1e-9);
  EXPECT_NEAR(ccw.y, 0.0, 1e-9);
}

TEST(Quadrotor, RotorsOffFallsAtGWithoutDrag) {
  QuadParams p = default_quad_params();
  p.drag_coeff_b = {};
  const Environment env{};
  QuadState x;
  x.body.pos_ned_m = {0.0, 0.0, -100.0};
  x = run(p, x, MotorCommand{}, env, 3000);
  EXPECT_NEAR(x.body.pos_ned_m.z, -100.0 + 0.5 * env.gravity_mps2 * 9.0, 1e-9);
}

TEST(Quadrotor, DragGivesTerminalVelocity) {
  const QuadParams p = default_quad_params();
  const Environment env{};
  const QuadState x = run(p, QuadState{}, MotorCommand{}, env, 60000);  // 60 s, ~14 time constants
  const double terminal = p.body.mass_kg * env.gravity_mps2 / p.drag_coeff_b.z;
  EXPECT_NEAR(x.body.vel_ned_mps.z / terminal, 1.0, 1e-5);
}

TEST(Quadrotor, WindDragIsDownwindWhenYawed) {
  // Facing east in a north wind: drag must still push north. At identity attitude a
  // backwards rotation looks identical to the right one, so this test yaws first.
  const QuadParams p = default_quad_params();
  Environment env{};
  env.wind_ned_mps = {3.0, 0.0, 0.0};
  QuadState x = hover_state(p, env, Vec3d{0.0, 0.0, -10.0});
  x.body.q_nb = gnc::from_axis_angle(Vec3d{0.0, 0.0, 1.0}, std::numbers::pi / 2.0);
  const Vec3d a = quad_derivative(p, x, hover_command(p, env), env).body.vel_ned_mps;
  EXPECT_NEAR(a.x, p.drag_coeff_b.x * 3.0 / p.body.mass_kg, 1e-12);
  EXPECT_NEAR(a.y, 0.0, 1e-12);
}

TEST(Quadrotor, AnisotropicDragIsAppliedInBodyAxes) {
  // Pitched -0.5 rad, moving north at 5 m/s, rotors stopped: the wrench is pure drag,
  // F_n = -C_nb D C_nb^T v_n with D = diag(0.25, 0.25, 0.35).
  const QuadParams p = default_quad_params();
  const Environment env{};
  QuadState x;
  x.body.q_nb = gnc::from_axis_angle(Vec3d{0.0, 1.0, 0.0}, -0.5);
  x.body.vel_ned_mps = {5.0, 0.0, 0.0};
  const Vec3d drag_ned = gnc::rotate(x.body.q_nb, quad_wrench(p, x, env).force_b_N);
  const gnc::Mat3d c = gnc::to_dcm(x.body.q_nb);
  const Vec3d expected =
      -(c * (gnc::diag(p.drag_coeff_b) * (gnc::transposed(c) * x.body.vel_ned_mps)));
  EXPECT_TRUE(vec_near(drag_ned, expected, 1e-12));
  EXPECT_TRUE(vec_near(drag_ned, Vec3d{-1.364924, 0.0, 0.210368}, 1e-6));  // worked by hand
}

TEST(Quadrotor, WindCarriesTheHoveringVehicleDownwind) {
  const QuadParams p = default_quad_params();
  Environment env{};
  env.wind_ned_mps = {3.0, 0.0, 0.0};  // 3 m/s wind blowing toward north
  const QuadState x0 = hover_state(p, env, Vec3d{0.0, 0.0, -10.0});

  // Initially the air moves past at 3 m/s: drag pushes north at D_x * 3 / m.
  const Vec3d a0 = quad_derivative(p, x0, hover_command(p, env), env).body.vel_ned_mps;
  EXPECT_NEAR(a0.x, p.drag_coeff_b.x * 3.0 / p.body.mass_kg, 1e-12);

  // After 10 time constants (m / D_x = 6 s) it drifts along with the air mass.
  const QuadState x = run(p, x0, hover_command(p, env), env, 60000);
  EXPECT_NEAR(x.body.vel_ned_mps.x, 3.0, 1e-3);
  EXPECT_NEAR(x.body.vel_ned_mps.y, 0.0, 1e-9);
  EXPECT_NEAR(x.body.pos_ned_m.z, -10.0, 1e-6);
  EXPECT_LT(gnc::angle_between(x.body.q_nb, Quatd::identity()), 1e-9);  // drag acts at the CG
}

TEST(Quadrotor, CommandsClampToZeroAndOne) {
  const QuadParams p = default_quad_params();
  const Environment env{};
  // Out-of-range commands must behave bit-for-bit like their clamped values, including
  // during the transient (one motor time constant), not just at steady state.
  const QuadState x0 = hover_state(p, env);
  const QuadState raw = run(p, x0, MotorCommand{{2.0, -1.0, 1.0, 0.0}}, env, 30);
  const QuadState clamped = run(p, x0, MotorCommand{{1.0, 0.0, 1.0, 0.0}}, env, 30);
  for (std::size_t i = 0; i < kNumRotors; ++i) {
    EXPECT_EQ(raw.rotor_speed_radps[i], clamped.rotor_speed_radps[i]);
  }
  // A negative command targets zero speed: no spin-down faster than the motor lag allows.
  const QuadState d = quad_derivative(p, QuadState{}, MotorCommand{{-1.0, -1.0, -1.0, -1.0}}, env);
  for (double rate : d.rotor_speed_radps) EXPECT_EQ(rate, 0.0);
}

TEST(Quadrotor, CoarseStepNeverReversesARotor) {
  // With dt = 3 tau, RK4 overshoots a spin-up to -412.5 rad/s; quad_step must clamp at zero.
  const QuadParams p = default_quad_params();
  const Environment env{};
  const QuadState x = quad_step(p, QuadState{}, MotorCommand{{1.0, 1.0, 1.0, 1.0}}, env,
                                3.0 * p.motor_time_constant_s);
  for (double speed : x.rotor_speed_radps) EXPECT_GE(speed, 0.0);
}

TEST(Quadrotor, StepReturnsUnitQuaternion) {
  const QuadParams p = default_quad_params();
  const Environment env{};
  QuadState x = hover_state(p, env);
  x.body.omega_b_radps = {0.0, 0.0, 50.0};
  x = quad_step(p, x, hover_command(p, env), env, 0.01);
  EXPECT_NEAR(gnc::norm(x.body.q_nb), 1.0, 1e-12);
}

TEST(Quadrotor, SameInputsGiveBitIdenticalRuns) {
  // Monte Carlo and regression tests depend on exact reproducibility.
  const QuadParams p = default_quad_params();
  const Environment env{};
  MotorCommand cmd = hover_command(p, env);
  cmd.u[0] *= 1.02;
  const QuadState a = run(p, hover_state(p, env), cmd, env, 2000);
  const QuadState b = run(p, hover_state(p, env), cmd, env, 2000);
  EXPECT_EQ(a.body.pos_ned_m.x, b.body.pos_ned_m.x);
  EXPECT_EQ(a.body.pos_ned_m.y, b.body.pos_ned_m.y);
  EXPECT_EQ(a.body.pos_ned_m.z, b.body.pos_ned_m.z);
  EXPECT_EQ(a.body.q_nb.w, b.body.q_nb.w);
  EXPECT_EQ(a.body.q_nb.x, b.body.q_nb.x);
  EXPECT_EQ(a.body.q_nb.y, b.body.q_nb.y);
  EXPECT_EQ(a.body.q_nb.z, b.body.q_nb.z);
}

}  // namespace
}  // namespace sim::test
