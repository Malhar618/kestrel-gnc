#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>

#include "gnc/control/control_laws.hpp"
#include "gnc/control/mixer.hpp"
#include "gnc/control/pid.hpp"
#include "gnc/control/quad_controller.hpp"
#include "gnc/control/quad_model.hpp"
#include "gnc/math/euler.hpp"
#include "test_support.hpp"

namespace gnc::test {
namespace {

constexpr float kG = 9.80665f;
constexpr float kPiF = std::numbers::pi_v<float>;

// Relative check for single-precision results.
::testing::AssertionResult close(float actual, float expected, float rel, float abs_floor = 1e-6f) {
  const float tolerance = std::max(abs_floor, rel * std::abs(expected));
  if (std::abs(actual - expected) <= tolerance) return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure()
         << actual << " vs " << expected << " (tolerance " << tolerance << ")";
}

bool within_rotor_range(const MixerResult& r, float f_max) {
  for (float f : r.rotor_thrust_N) {
    if (f < 0.0f || f > f_max) return false;
  }
  return true;
}

// --- Mixer ---

TEST(Mixer, HoverSplitsEvenlyAtTheHoverCommand) {
  const QuadModel m = default_quad_model();
  const Mixer mixer(m);
  ASSERT_TRUE(mixer.valid());
  const MixerResult r = mixer.mix({m.mass_kg * kG, {}});
  const float per_rotor = m.mass_kg * kG / 4.0f;
  const float u_hover = std::sqrt(per_rotor / m.thrust_coeff) / m.rotor_speed_max_radps;
  for (std::size_t i = 0; i < kNumRotors; ++i) {
    EXPECT_TRUE(close(r.rotor_thrust_N[i], per_rotor, 1e-5f));
    EXPECT_TRUE(close(r.outputs.u[i], u_hover, 1e-5f));
  }
  EXPECT_FALSE(r.saturated);
}

TEST(Mixer, DeliversAnyRequestWithinRange) {
  const QuadModel m = default_quad_model();
  const Mixer mixer(m);
  const ThrustTorque requests[] = {
      {14.7f, {0.2f, -0.15f, 0.03f}}, {20.0f, {-0.4f, 0.1f, -0.05f}}, {8.0f, {0.0f, 0.3f, 0.0f}}};
  for (const ThrustTorque& req : requests) {
    const MixerResult r = mixer.mix(req);
    EXPECT_FALSE(r.saturated);
    EXPECT_TRUE(close(r.achieved.thrust_N, req.thrust_N, 1e-5f));
    EXPECT_TRUE(vec_near(r.achieved.torque_b_Nm, req.torque_b_Nm, 1e-5f));
  }
}

TEST(Mixer, TorqueRequestsFollowPx4MotorLayout) {
  const QuadModel m = default_quad_model();
  const Mixer mixer(m);
  const float t = m.mass_kg * kG;
  // Pitch up: front rotors (1, 3) push harder than rear rotors (2, 4).
  const MixerResult pitch = mixer.mix({t, {0.0f, 0.2f, 0.0f}});
  EXPECT_GT(pitch.rotor_thrust_N[0], pitch.rotor_thrust_N[1]);
  EXPECT_GT(pitch.rotor_thrust_N[2], pitch.rotor_thrust_N[3]);
  // Roll right: the left rotors (2, 3) push harder so the right side drops.
  const MixerResult roll = mixer.mix({t, {0.2f, 0.0f, 0.0f}});
  EXPECT_GT(roll.rotor_thrust_N[1], roll.rotor_thrust_N[0]);
  EXPECT_GT(roll.rotor_thrust_N[2], roll.rotor_thrust_N[3]);
  // Yaw right: the CCW rotors (1, 2) push harder; their reaction torque turns the nose right.
  const MixerResult yaw = mixer.mix({t, {0.0f, 0.0f, 0.02f}});
  EXPECT_GT(yaw.rotor_thrust_N[0], yaw.rotor_thrust_N[2]);
  EXPECT_GT(yaw.rotor_thrust_N[1], yaw.rotor_thrust_N[3]);
}

TEST(Mixer, YawGivesWayBeforeRollPitchAndThrust) {
  const QuadModel m = default_quad_model();
  const Mixer mixer(m);
  const ThrustTorque req{m.mass_kg * kG, {0.3f, -0.2f, 5.0f}};  // far more yaw than possible
  const MixerResult r = mixer.mix(req);
  EXPECT_TRUE(r.saturated);
  EXPECT_TRUE(within_rotor_range(r, m.max_rotor_thrust_N()));
  EXPECT_TRUE(close(r.achieved.thrust_N, req.thrust_N, 1e-5f));
  EXPECT_TRUE(close(r.achieved.torque_b_Nm.x, 0.3f, 1e-4f));
  EXPECT_TRUE(close(r.achieved.torque_b_Nm.y, -0.2f, 1e-4f));
  EXPECT_GT(r.achieved.torque_b_Nm.z, 0.0f);  // as much yaw as fits, same direction
  EXPECT_LT(r.achieved.torque_b_Nm.z, 5.0f);
}

TEST(Mixer, OversizedRollPitchIsScaledKeepingItsDirection) {
  const QuadModel m = default_quad_model();
  const Mixer mixer(m);
  const MixerResult r = mixer.mix({m.mass_kg * kG, {10.0f, 5.0f, 0.0f}});
  EXPECT_TRUE(r.saturated);
  EXPECT_TRUE(within_rotor_range(r, m.max_rotor_thrust_N()));
  EXPECT_GT(r.achieved.torque_b_Nm.x, 0.0f);
  EXPECT_TRUE(close(r.achieved.torque_b_Nm.x / r.achieved.torque_b_Nm.y, 2.0f, 1e-4f));
}

TEST(Mixer, ExcessThrustIsCutButTorqueIsKept) {
  const QuadModel m = default_quad_model();
  const Mixer mixer(m);
  const float too_much = 4.0f * m.max_rotor_thrust_N() * 1.2f;
  const MixerResult r = mixer.mix({too_much, {0.2f, 0.0f, 0.0f}});
  EXPECT_TRUE(r.saturated);
  EXPECT_TRUE(within_rotor_range(r, m.max_rotor_thrust_N()));
  EXPECT_LT(r.achieved.thrust_N, too_much);
  EXPECT_TRUE(close(r.achieved.torque_b_Nm.x, 0.2f, 1e-4f));
}

TEST(Mixer, NegativeThrustRequestStopsAtZero) {
  const QuadModel m = default_quad_model();
  const MixerResult r = Mixer(m).mix({-5.0f, {}});
  for (float u : r.outputs.u) EXPECT_GE(u, 0.0f);
  EXPECT_TRUE(within_rotor_range(r, m.max_rotor_thrust_N()));
}

TEST(Mixer, DegenerateGeometryIsRejected) {
  QuadModel m = default_quad_model();
  m.rotor_pos_b_m = {};  // every rotor at the CG: no roll or pitch authority
  const Mixer mixer(m);
  EXPECT_FALSE(mixer.valid());
  const MixerResult r = mixer.mix({10.0f, {}});
  EXPECT_TRUE(r.saturated);
  for (float u : r.outputs.u) EXPECT_EQ(u, 0.0f);
}

// --- PID ---

PidGains3 gains(Vec3f kp, Vec3f ki, Vec3f kd, Vec3f limit, float cutoff = 0.0f) {
  PidGains3 g;
  g.kp = kp;
  g.ki = ki;
  g.kd = kd;
  g.integrator_limit = limit;
  g.derivative_cutoff_hz = cutoff;
  return g;
}

TEST(Pid3, ProportionalActsPerAxis) {
  Pid3 pid(gains({2, 3, 4}, {}, {}, {}));
  EXPECT_TRUE(vec_near(pid.update({1, -1, 0.5f}, {}, 0.01f, false), Vec3f{2, -3, 2}, 1e-6f));
}

TEST(Pid3, IntegratorAccumulatesAndClamps) {
  Pid3 pid(gains({}, {1, 1, 1}, {}, {10, 10, 0.3f}));
  Vec3f out{};
  for (int i = 0; i < 5; ++i) out = pid.update({1, -1, 1}, {}, 0.1f, false);
  EXPECT_TRUE(vec_near(out, Vec3f{0.5f, -0.5f, 0.3f}, 1e-6f));  // z hit its 0.3 limit
}

TEST(Pid3, FrozenIntegratorHoldsItsValue) {
  Pid3 pid(gains({}, {1, 1, 1}, {}, {10, 10, 10}));
  pid.update({1, 1, 1}, {}, 0.1f, false);
  const Vec3f before = pid.integral();
  for (int i = 0; i < 10; ++i) pid.update({1, 1, 1}, {}, 0.1f, true);
  EXPECT_TRUE(vec_near(pid.integral(), before, 0.0f));
}

TEST(Pid3, DerivativeActsOnMeasurementSoSetpointStepsDontKick) {
  Pid3 pid(gains({1, 1, 1}, {}, {1, 1, 1}, {}));
  pid.update({}, {}, 0.1f, false);
  // The setpoint jumps (error 5) but the measurement doesn't move: no derivative kick.
  EXPECT_TRUE(vec_near(pid.update({5, 5, 5}, {}, 0.1f, false), Vec3f{5, 5, 5}, 1e-6f));
  // The measurement rises 0.1 in 0.1 s: derivative 1, which opposes the motion.
  EXPECT_TRUE(
      vec_near(pid.update({5, 5, 5}, {0.1f, 0.1f, 0.1f}, 0.1f, false), Vec3f{4, 4, 4}, 1e-5f));
}

TEST(Pid3, DerivativeFilterIsFirstOrder) {
  const float cutoff = 10.0f, dt = 0.001f;
  Pid3 pid(gains({}, {}, {1, 1, 1}, {}, cutoff));
  pid.update({}, {}, dt, false);
  Vec3f out{};
  const int steps = 20;
  for (int i = 1; i <= steps; ++i) {
    const float meas = static_cast<float>(i) * dt;  // slope 1
    out = pid.update({}, {meas, meas, meas}, dt, false);
  }
  const float tau = 1.0f / (2.0f * kPiF * cutoff);
  const float alpha = dt / (dt + tau);
  const float expected = -(1.0f - std::pow(1.0f - alpha, static_cast<float>(steps)));
  EXPECT_TRUE(close(out.x, expected, 1e-3f));
}

TEST(Pid3, ResetClearsEverything) {
  Pid3 pid(gains({}, {1, 1, 1}, {1, 1, 1}, {10, 10, 10}));
  pid.update({1, 1, 1}, {1, 1, 1}, 0.1f, false);
  pid.update({1, 1, 1}, {2, 2, 2}, 0.1f, false);
  pid.reset();
  EXPECT_TRUE(vec_near(pid.integral(), Vec3f{}, 0.0f));
  // First update after reset has no previous measurement, so no derivative term.
  EXPECT_TRUE(vec_near(pid.update({}, {5, 5, 5}, 0.1f, false), Vec3f{}, 1e-6f));
}

// --- Attitude -> body rate ---

AttitudeGains att_gains() { return {{6, 6, 4}, {3.8f, 3.8f, 1.6f}}; }

TEST(AttitudeControl, ZeroErrorGivesZeroRate) {
  const Quatf q = from_euler321(Euler321f{0.2f, -0.1f, 1.0f});
  EXPECT_TRUE(vec_near(attitude_rate_setpoint(q, q, att_gains()), Vec3f{}, 1e-6f));
}

TEST(AttitudeControl, SmallRollErrorGivesProportionalRollRate) {
  const Quatf sp = from_euler321(Euler321f{0.1f, 0, 0});
  EXPECT_TRUE(vec_near(attitude_rate_setpoint(Quatf::identity(), sp, att_gains()),
                       Vec3f{0.6f, 0, 0}, 1e-5f));
}

TEST(AttitudeControl, ErrorIsExpressedInBodyAxes) {
  // Yawed to face east, the setpoint adds 0.1 rad of roll. That's a body-x (roll) rate,
  // even though the vehicle's roll axis now points east rather than north.
  const Quatf q = from_euler321(Euler321f{0, 0, kPiF / 2});
  const Quatf sp = from_euler321(Euler321f{0.1f, 0, kPiF / 2});
  EXPECT_TRUE(vec_near(attitude_rate_setpoint(q, sp, att_gains()), Vec3f{0.6f, 0, 0}, 1e-5f));
}

TEST(AttitudeControl, LargeErrorsAreRateLimited) {
  const Quatf sp = from_euler321(Euler321f{1.0f, 0, 0});  // 6 rad/s requested, 3.8 allowed
  EXPECT_TRUE(close(attitude_rate_setpoint(Quatf::identity(), sp, att_gains()).x, 3.8f, 1e-6f));
}

TEST(AttitudeControl, HalfTurnYawErrorPicksOneDirection) {
  // Exactly 180 deg of yaw error has no shorter way round; either sign of the setpoint
  // quaternion must give the same, saturated yaw rate.
  const Quatf sp{0, 0, 0, 1};
  const Vec3f a = attitude_rate_setpoint(Quatf::identity(), sp, att_gains());
  const Vec3f b = attitude_rate_setpoint(Quatf::identity(), -1.0f * sp, att_gains());
  EXPECT_TRUE(vec_near(a, Vec3f{0, 0, 1.6f}, 1e-6f));
  EXPECT_TRUE(vec_near(a, b, 0.0f));
}

// --- Acceleration -> attitude + thrust ---

constexpr float kMass = 1.5f;
constexpr float kTilt = 35.0f * kPiF / 180.0f;

TEST(ThrustToAttitude, ZeroAccelerationIsLevelHoverThrust) {
  const ThrustAttitude ta = thrust_to_attitude({}, 0, Quatf::identity(), kMass, kG, kTilt);
  EXPECT_TRUE(same_rotation(ta.q_sp, Quatf::identity(), 1e-6f));
  EXPECT_TRUE(close(ta.thrust_N, kMass * kG, 1e-6f));
  EXPECT_FALSE(ta.tilt_limited);
}

TEST(ThrustToAttitude, AcceleratingNorthPitchesNoseDown) {
  const ThrustAttitude ta = thrust_to_attitude({2, 0, 0}, 0, Quatf::identity(), kMass, kG, kTilt);
  const Euler321f e = to_euler321(ta.q_sp);
  EXPECT_TRUE(close(e.pitch, -std::atan(2.0f / kG), 1e-5f));
  KESTREL_EXPECT_NEAR(e.roll, 0.0f, 1e-6f);
  KESTREL_EXPECT_NEAR(e.yaw, 0.0f, 1e-6f);
}

TEST(ThrustToAttitude, AcceleratingEastRollsRight) {
  const ThrustAttitude ta = thrust_to_attitude({0, 2, 0}, 0, Quatf::identity(), kMass, kG, kTilt);
  const Euler321f e = to_euler321(ta.q_sp);
  EXPECT_TRUE(close(e.roll, std::atan(2.0f / kG), 1e-5f));
  KESTREL_EXPECT_NEAR(e.pitch, 0.0f, 1e-6f);
}

TEST(ThrustToAttitude, HeadingPointsTheNose) {
  const ThrustAttitude ta =
      thrust_to_attitude({}, kPiF / 2, Quatf::identity(), kMass, kG, kTilt);  // face east
  EXPECT_TRUE(vec_near(rotate(ta.q_sp, Vec3f{1, 0, 0}), Vec3f{0, 1, 0}, 1e-6f));
}

TEST(ThrustToAttitude, TiltIsLimited) {
  const ThrustAttitude ta = thrust_to_attitude({100, 0, 0}, 0, Quatf::identity(), kMass, kG, kTilt);
  EXPECT_TRUE(ta.tilt_limited);
  EXPECT_TRUE(close(to_euler321(ta.q_sp).pitch, -kTilt, 1e-5f));
}

TEST(ThrustToAttitude, FallingFasterThanGravityKeepsMinimumThrust) {
  const ThrustAttitude ta =
      thrust_to_attitude({0, 0, 2 * kG}, 0, Quatf::identity(), kMass, kG, kTilt);
  EXPECT_TRUE(close(ta.thrust_N, 0.1f * kMass * kG, 1e-5f));
  EXPECT_TRUE(same_rotation(ta.q_sp, Quatf::identity(), 1e-6f));
}

TEST(ThrustToAttitude, ThrustIsProjectedOnTheCurrentThrustAxis) {
  // Still rolled 30 deg while asking to hover: only the projection on the current
  // thrust axis is commanded, so the vehicle doesn't balloon upward while it levels.
  const Quatf rolled = from_euler321(Euler321f{kPiF / 6, 0, 0});
  const ThrustAttitude ta = thrust_to_attitude({}, 0, rolled, kMass, kG, kTilt);
  EXPECT_TRUE(close(ta.thrust_N, kMass * kG * std::cos(kPiF / 6), 1e-5f));
}

// --- Whole controller ---

TEST(QuadController, AtTheSetpointItCommandsHover) {
  const QuadModel m = default_quad_model();
  QuadController c(m, default_controller_gains());
  VehicleState x;
  x.pos_ned_m = {1, 2, -10};
  PositionSetpoint sp;
  sp.pos_ned_m = x.pos_ned_m;
  const MotorOutputs out = c.step(x, sp, 0.004f);
  const float u_hover = std::sqrt(m.mass_kg * kG / 4.0f / m.thrust_coeff) / m.rotor_speed_max_radps;
  for (float u : out.u) EXPECT_TRUE(close(u, u_hover, 1e-5f));
  EXPECT_FALSE(c.status().saturated);
}

TEST(QuadController, SaturationFreezesTheRateIntegrator) {
  const QuadModel m = default_quad_model();
  QuadController c(m, default_controller_gains());
  const VehicleState level{};
  const Quatf rolled = from_euler321(Euler321f{0.5f, 0, 0});
  // Ask for more thrust than the rotors have: every step saturates after the first.
  c.step_attitude(level, rolled, 60.0f, 0.004f);
  ASSERT_TRUE(c.status().saturated);
  const Vec3f frozen = c.status().rate_integral;
  for (int i = 0; i < 50; ++i) c.step_attitude(level, rolled, 60.0f, 0.004f);
  EXPECT_TRUE(vec_near(c.status().rate_integral, frozen, 0.0f));
  // Back within limits, the integrator runs again.
  c.step_attitude(level, rolled, m.mass_kg * kG, 0.004f);
  c.step_attitude(level, rolled, m.mass_kg * kG, 0.004f);
  EXPECT_GT(norm(c.status().rate_integral - frozen), 0.0f);
}

}  // namespace
}  // namespace gnc::test
