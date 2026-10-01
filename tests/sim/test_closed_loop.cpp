#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <numbers>
#include <vector>

#include "gnc/control/mixer.hpp"
#include "gnc/control/quad_controller.hpp"
#include "gnc/control/quad_model.hpp"
#include "gnc/math/euler.hpp"
#include "sim/fsw_bridge.hpp"
#include "sim/quadrotor.hpp"
#include "test_support.hpp"

namespace sim::test {
namespace {

using gnc::Vec3d;
using gnc::test::same_rotation;
using gnc::test::vec_near;

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr float kDegF = std::numbers::pi_v<float> / 180.0f;

// What the controller is asked for at time t: a position (and heading), or an attitude
// with a collective thrust.
struct Target {
  bool attitude_mode = false;
  gnc::PositionSetpoint pos{};
  gnc::Quatf q_sp = gnc::Quatf::identity();
  float thrust_N = 0.0f;
};

Target position(float n, float e, float d, float yaw_deg = 0.0f) {
  Target t;
  t.pos.pos_ned_m = {n, e, d};
  t.pos.yaw_rad = yaw_deg * kDegF;
  return t;
}

struct Sample {
  double t_s;
  QuadState x;
  bool saturated;
};

// Flies the flight-software controller (250 Hz, float) against the plant (1 kHz, double).
std::vector<Sample> fly(const QuadParams& p, const Environment& env, QuadState x,
                        const std::function<Target(double)>& target, double duration_s,
                        const gnc::QuadControllerGains& gains = gnc::default_controller_gains()) {
  gnc::QuadController controller(gnc::default_quad_model(), gains);
  std::vector<Sample> log;
  MotorCommand cmd{};
  const int steps = static_cast<int>(std::lround(duration_s * 1000.0));
  for (int k = 0; k <= steps; ++k) {
    const double t = k * 1e-3;
    if (k % 4 == 0) {
      const Target tg = target(t);
      const gnc::VehicleState s = to_fsw_state(x);
      cmd = to_motor_command(tg.attitude_mode
                                 ? controller.step_attitude(s, tg.q_sp, tg.thrust_N, 0.004f)
                                 : controller.step(s, tg.pos, 0.004f));
    }
    log.push_back({t, x, controller.status().saturated});
    x = quad_step(p, x, cmd, env, 1e-3);
  }
  return log;
}

// Time after t0 at which |error| last left the band (0 if it never did).
double settling_time(const std::vector<Sample>& log, double t0,
                     const std::function<double(const QuadState&)>& error, double band) {
  double last_out = t0;
  for (const Sample& s : log) {
    if (s.t_s >= t0 && std::abs(error(s.x)) > band) last_out = s.t_s;
  }
  return last_out - t0;
}

double tilt_deg(const QuadState& x) {
  const gnc::Quatd& q = x.body.q_nb;
  return std::acos(std::clamp(1.0 - 2.0 * (q.x * q.x + q.y * q.y), -1.0, 1.0)) / kDeg;
}

double roll_deg(const QuadState& x) { return gnc::to_euler321(x.body.q_nb).roll / kDeg; }
double yaw_deg(const QuadState& x) { return gnc::to_euler321(x.body.q_nb).yaw / kDeg; }

Target roll_step(double t, float roll_deg_after) {
  const float roll = t < 0.5 ? 0.0f : roll_deg_after * kDegF;
  Target tg;
  tg.attitude_mode = true;
  tg.q_sp = gnc::from_euler321(gnc::Euler321f{roll, 0, 0});
  tg.thrust_N = 1.5f * 9.80665f / std::cos(roll);
  return tg;
}

// A plant that disagrees with the controller's model: heavier inertia, slower motors,
// weaker props.
QuadParams mismatched_plant() {
  QuadParams p = default_quad_params();
  p.body = make_rigid_body(p.body.mass_kg, gnc::diag(Vec3d{0.026, 0.026, 0.0455}));
  p.motor_time_constant_s *= 1.5;
  p.thrust_coeff *= 0.9;
  return p;
}

// --- The flight model and the mixer against the plant ---

TEST(ClosedLoop, FlightModelMatchesTheSimulatorDefaults) {
  // The flight model is a separate copy on purpose (fsw/ never reads sim/); by default
  // the two must describe the same vehicle to float precision.
  const gnc::QuadModel m = gnc::default_quad_model();
  const QuadParams p = default_quad_params();
  const auto d = [](float v) { return static_cast<double>(v); };
  EXPECT_NEAR(d(m.mass_kg), p.body.mass_kg, 1e-6);
  for (std::size_t i = 0; i < 9; ++i) {
    EXPECT_NEAR(d(m.inertia_b_kgm2.m[i]), p.body.inertia_b_kgm2.m[i], 1e-8);
  }
  for (std::size_t i = 0; i < kNumRotors; ++i) {
    EXPECT_NEAR(d(m.rotor_pos_b_m[i].x), p.rotors[i].pos_b_m.x, 1e-6);
    EXPECT_NEAR(d(m.rotor_pos_b_m[i].y), p.rotors[i].pos_b_m.y, 1e-6);
    EXPECT_NEAR(d(m.rotor_pos_b_m[i].z), p.rotors[i].pos_b_m.z, 1e-6);
    EXPECT_EQ(m.rotor_spin[i], p.rotors[i].spin);
  }
  EXPECT_NEAR(d(m.thrust_coeff) / p.thrust_coeff, 1.0, 1e-6);
  EXPECT_NEAR(d(m.torque_coeff) / p.torque_coeff, 1.0, 1e-6);
  EXPECT_NEAR(d(m.rotor_speed_max_radps), p.rotor_speed_max_radps, 1e-3);
}

TEST(ClosedLoop, MixerCommandsProduceTheRequestedWrenchOnThePlant) {
  // Mixer (flight code) -> steady rotor speeds -> plant's own wrench model. The two
  // allocation models are written independently, so this catches a sign or layout slip.
  const gnc::Mixer mixer(gnc::default_quad_model());
  const QuadParams p = default_quad_params();
  const gnc::ThrustTorque req{16.0f, {0.2f, -0.15f, 0.03f}};
  const gnc::MixerResult r = mixer.mix(req);
  ASSERT_FALSE(r.saturated);
  QuadState x;
  for (std::size_t i = 0; i < kNumRotors; ++i) {
    x.rotor_speed_radps[i] = static_cast<double>(r.outputs.u[i]) * p.rotor_speed_max_radps;
  }
  const Wrench w = quad_wrench(p, x, Environment{});
  EXPECT_NEAR(-w.force_b_N.z, 16.0, 16.0 * 1e-5);
  EXPECT_TRUE(vec_near(w.torque_b_Nm, Vec3d{0.2, -0.15, 0.03}, 1e-5));
}

TEST(FswBridge, ConvertsPrecisionNotFrames) {
  // Truth (double) -> flight state (float): same numbers, same frames, nothing reordered.
  QuadState x;
  x.body.pos_ned_m = {1.0, -2.0, -10.0};
  x.body.vel_ned_mps = {0.5, 0.25, -0.125};
  x.body.q_nb = gnc::from_euler321(gnc::Euler321d{0.1, -0.2, 1.5});
  x.body.omega_b_radps = {0.3, -0.2, 0.1};
  const gnc::VehicleState s = to_fsw_state(x);
  EXPECT_TRUE(vec_near(s.pos_ned_m, gnc::Vec3f{1.0f, -2.0f, -10.0f}, 0.0f));
  EXPECT_TRUE(vec_near(s.vel_ned_mps, gnc::Vec3f{0.5f, 0.25f, -0.125f}, 0.0f));
  EXPECT_TRUE(vec_near(s.omega_b_radps, gnc::Vec3f{0.3f, -0.2f, 0.1f}, 0.0f));
  EXPECT_TRUE(same_rotation(s.q_nb, gnc::from_euler321(gnc::Euler321f{0.1f, -0.2f, 1.5f}),
                            gnc::test::tol<float>()));

  gnc::MotorOutputs out;
  out.u = {0.1f, 0.2f, 0.3f, 0.4f};
  const MotorCommand cmd = to_motor_command(out);
  for (std::size_t i = 0; i < kNumRotors; ++i) {
    EXPECT_EQ(cmd.u[i], static_cast<double>(out.u[i]));  // same rotor order
  }
}

// --- Closed-loop specs ---

TEST(ClosedLoop, RecoversToHoverFromAnOffsetTiltedStart) {
  const QuadParams p = default_quad_params();
  const Environment env{};
  QuadState x0 = hover_state(p, env, Vec3d{2.0, -1.5, -8.0});
  x0.body.q_nb = gnc::from_euler321(gnc::Euler321d{10 * kDeg, -5 * kDeg, 30 * kDeg});
  const auto log = fly(p, env, x0, [](double) { return position(0, 0, -10); }, 10.0);
  for (const Sample& s : log) {
    if (s.t_s < 6.0) continue;
    EXPECT_LT(gnc::norm(s.x.body.pos_ned_m - Vec3d{0, 0, -10}), 0.05) << "t = " << s.t_s;
    EXPECT_LT(tilt_deg(s.x), 1.0);
  }
  EXPECT_NEAR(yaw_deg(log.back().x), 0.0, 0.1);
}

TEST(ClosedLoop, OneMetreStepSettlesWithoutOvershoot) {
  const QuadParams p = default_quad_params();
  const Environment env{};
  const auto log = fly(
      p, env, hover_state(p, env, Vec3d{0, 0, -10}),
      [](double t) { return position(t < 1.0 ? 0.0f : 1.0f, 0, -10); }, 6.0);
  double peak = 0.0, cross = 0.0, altitude = 0.0, tilt = 0.0;
  for (const Sample& s : log) {
    peak = std::max(peak, s.x.body.pos_ned_m.x);
    cross = std::max(cross, std::abs(s.x.body.pos_ned_m.y));
    altitude = std::max(altitude, std::abs(s.x.body.pos_ned_m.z + 10.0));
    tilt = std::max(tilt, tilt_deg(s.x));
  }
  EXPECT_LT(peak, 1.10);  // < 10% overshoot
  EXPECT_LT(settling_time(
                log, 1.0, [](const QuadState& x) { return x.body.pos_ned_m.x - 1.0; }, 0.02),
            3.0);  // within 2 cm in under 3 s
  EXPECT_LT(cross, 0.01);
  EXPECT_LT(altitude, 0.05);
  EXPECT_LT(tilt, 35.5);
}

TEST(ClosedLoop, OneMetreStepFacingEast) {
  // Same step flown with the nose pointing east: a north move is now a roll, not a pitch.
  // Catches a world/body mix-up that a yaw-0 flight cannot see.
  const QuadParams p = default_quad_params();
  const Environment env{};
  QuadState x0 = hover_state(p, env, Vec3d{0, 0, -10});
  x0.body.q_nb = gnc::from_euler321(gnc::Euler321d{0, 0, 90 * kDeg});
  const auto log =
      fly(p, env, x0, [](double t) { return position(t < 1.0 ? 0.0f : 1.0f, 0, -10, 90.0f); }, 6.0);
  double peak = 0.0, cross = 0.0, tilt = 0.0;
  for (const Sample& s : log) {
    peak = std::max(peak, s.x.body.pos_ned_m.x);
    cross = std::max(cross, std::abs(s.x.body.pos_ned_m.y));
    tilt = std::max(tilt, tilt_deg(s.x));
  }
  EXPECT_LT(peak, 1.10);
  EXPECT_LT(settling_time(
                log, 1.0, [](const QuadState& x) { return x.body.pos_ned_m.x - 1.0; }, 0.02),
            3.0);
  EXPECT_LT(cross, 0.01);
  EXPECT_LT(tilt, 35.5);
}

TEST(ClosedLoop, TenDegreeRollStepSettlesInHalfASecond) {
  const QuadParams p = default_quad_params();
  const Environment env{};
  const auto log = fly(
      p, env, hover_state(p, env, Vec3d{0, 0, -10}), [](double t) { return roll_step(t, 10.0f); },
      2.0);
  double peak = 0.0;
  for (const Sample& s : log) peak = std::max(peak, roll_deg(s.x));
  EXPECT_LT(peak, 10.5);  // < 5% overshoot
  EXPECT_LT(
      settling_time(log, 0.5, [](const QuadState& x) { return roll_deg(x) - 10.0; }, 0.2), 0.5);
}

TEST(ClosedLoop, RateDerivativeSpeedsUpTheRollStep) {
  // What kd is for: on the nominal plant it shortens the settle. (It is not what makes the
  // loop robust to a mismatched plant; the rate/attitude gain ratio does that.)
  const QuadParams p = default_quad_params();
  const Environment env{};
  const auto settle = [&](const gnc::QuadControllerGains& g) {
    const auto log = fly(
        p, env, hover_state(p, env, Vec3d{0, 0, -10}), [](double t) { return roll_step(t, 10.0f); },
        2.0, g);
    return settling_time(log, 0.5, [](const QuadState& x) { return roll_deg(x) - 10.0; }, 0.2);
  };
  gnc::QuadControllerGains no_kd = gnc::default_controller_gains();
  no_kd.rate.kd = {};
  EXPECT_LT(settle(gnc::default_controller_gains()), settle(no_kd) - 0.05);
}

TEST(ClosedLoop, YawStepTurnsInPlace) {
  const QuadParams p = default_quad_params();
  const Environment env{};
  const auto log = fly(
      p, env, hover_state(p, env, Vec3d{0, 0, -10}),
      [](double t) { return position(0, 0, -10, t < 1.0 ? 0.0f : 90.0f); }, 6.0);
  EXPECT_LT(
      settling_time(log, 1.0, [](const QuadState& x) { return yaw_deg(x) - 90.0; }, 1.0), 3.0);
  for (const Sample& s : log) {
    EXPECT_LT(gnc::norm(s.x.body.pos_ned_m - Vec3d{0, 0, -10}), 0.03);
    EXPECT_LT(tilt_deg(s.x), 1.0);
  }
}

TEST(ClosedLoop, IntegralActionRejectsSteadyWind) {
  const QuadParams p = default_quad_params();
  Environment env{};
  env.wind_ned_mps = {3.0, 0.0, 0.0};
  const auto log = fly(
      p, env, hover_state(p, env, Vec3d{0, 0, -10}), [](double) { return position(0, 0, -10); },
      15.0);
  double excursion = 0.0;
  for (const Sample& s : log) {
    const double err = gnc::norm(s.x.body.pos_ned_m - Vec3d{0, 0, -10});
    excursion = std::max(excursion, err);
    if (s.t_s >= 10.0) {
      EXPECT_LT(err, 0.02) << "t = " << s.t_s;
    }
  }
  EXPECT_LT(excursion, 0.20);
  // The wind blows toward the north, so holding station means tilting the thrust south,
  // into the wind: nose up for a north-facing vehicle, by atan(drag / weight).
  const double drag_N = p.drag_coeff_b.x * 3.0;
  const double lean_deg = std::atan(drag_N / (p.body.mass_kg * env.gravity_mps2)) / kDeg;
  EXPECT_NEAR(gnc::to_euler321(log.back().x.body.q_nb).pitch / kDeg, lean_deg, 0.1);
}

TEST(ClosedLoop, LongStepHitsSpeedAndTiltLimitsWithoutWindup) {
  // 20 m away: velocity and tilt limits engage and the velocity integrator is held,
  // so the vehicle arrives without a windup overshoot. The last centimetres come in
  // slowly: the integrator learned the drag at 5 m/s cruise and unwinds with a ~4 s time
  // constant. Guidance feed-forward (M5) takes that job off the integrator.
  const QuadParams p = default_quad_params();
  const Environment env{};
  const auto log = fly(
      p, env, hover_state(p, env, Vec3d{0, 0, -10}), [](double) { return position(20, 0, -10); },
      16.0);
  double peak = 0.0, tilt = 0.0, speed = 0.0, altitude = 0.0;
  for (const Sample& s : log) {
    peak = std::max(peak, s.x.body.pos_ned_m.x);
    altitude = std::max(altitude, std::abs(s.x.body.pos_ned_m.z + 10.0));
    tilt = std::max(tilt, tilt_deg(s.x));
    speed = std::max(speed, std::hypot(s.x.body.vel_ned_mps.x, s.x.body.vel_ned_mps.y));
  }
  EXPECT_LT(peak, 20.2);
  EXPECT_LT(tilt, 36.0);
  EXPECT_LT(speed, 5.2);
  EXPECT_LT(altitude, 0.08);  // height is held while tilted at the limit
  EXPECT_NEAR(log.back().x.body.pos_ned_m.x, 20.0, 0.015);
}

TEST(ClosedLoop, StaysStableWhenThePlantDisagreesWithTheModel) {
  const QuadParams p = mismatched_plant();
  const Environment env{};
  const auto att = fly(
      p, env, hover_state(p, env, Vec3d{0, 0, -10}), [](double t) { return roll_step(t, 10.0f); },
      2.5);
  double peak = 0.0;
  for (const Sample& s : att) peak = std::max(peak, roll_deg(s.x));
  EXPECT_LT(peak, 11.5);  // < 15% overshoot
  EXPECT_LT(
      settling_time(att, 0.5, [](const QuadState& x) { return roll_deg(x) - 10.0; }, 0.2), 1.0);

  const auto pos = fly(
      p, env, hover_state(p, env, Vec3d{0, 0, -10}),
      [](double t) { return position(t < 1.0 ? 0.0f : 1.0f, 0, -10); }, 8.0);
  EXPECT_NEAR(pos.back().x.body.pos_ned_m.x, 1.0, 0.02);
  EXPECT_NEAR(pos.back().x.body.pos_ned_m.z, -10.0, 0.02);  // integral absorbs the weak props
}

TEST(ClosedLoop, RunsAreBitIdentical) {
  const QuadParams p = default_quad_params();
  Environment env{};
  env.wind_ned_mps = {1.0, -2.0, 0.0};
  const auto target = [](double t) { return position(t < 1.0 ? 0.0f : 3.0f, 2, -12, 45); };
  const auto a = fly(p, env, hover_state(p, env, Vec3d{0, 0, -10}), target, 4.0);
  const auto b = fly(p, env, hover_state(p, env, Vec3d{0, 0, -10}), target, 4.0);
  EXPECT_EQ(a.back().x.body.pos_ned_m.x, b.back().x.body.pos_ned_m.x);
  EXPECT_EQ(a.back().x.body.pos_ned_m.y, b.back().x.body.pos_ned_m.y);
  EXPECT_EQ(a.back().x.body.q_nb.w, b.back().x.body.q_nb.w);
  EXPECT_EQ(a.back().x.body.q_nb.z, b.back().x.body.q_nb.z);
}

}  // namespace
}  // namespace sim::test
