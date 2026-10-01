// kestrel_sim: flies a scenario on the quadrotor plant and writes a CSV log. Open-loop
// scenarios drive the motors directly; closed-loop ones run the flight software's controller.
//
//   kestrel_sim <scenario> <output.csv>
//
// Physics runs at 1 kHz with integer-microsecond time; the log is written at 100 Hz.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numbers>
#include <string_view>

#include "gnc/control/quad_controller.hpp"
#include "gnc/control/quad_model.hpp"
#include "gnc/math/euler.hpp"
#include "sim/fsw_bridge.hpp"
#include "sim/quadrotor.hpp"

namespace {

using gnc::Vec3d;
using sim::Environment;
using sim::MotorCommand;
using sim::QuadParams;
using sim::QuadState;

constexpr std::uint64_t kPhysicsDtUs = 1000;     // 1 kHz
constexpr std::uint64_t kLogEveryUs = 10000;     // 100 Hz
constexpr std::uint64_t kControlEveryUs = 4000;  // 250 Hz

MotorCommand scaled_hover(const QuadParams& p, const Environment& env,
                          const std::array<double, sim::kNumRotors>& speed_scale) {
  MotorCommand cmd = sim::hover_command(p, env);
  for (std::size_t i = 0; i < sim::kNumRotors; ++i) cmd.u[i] *= speed_scale[i];
  return cmd;
}

MotorCommand hover_cmd(const QuadParams& p, const Environment& env, double /*t_s*/) {
  return sim::hover_command(p, env);
}

MotorCommand motors_off_cmd(const QuadParams& /*p*/, const Environment& /*env*/, double /*t_s*/) {
  return {};
}

// Speeds the CCW rotors (motors 1, 2) up 3% and the CW rotors (3, 4) down 3%.
MotorCommand yaw_spin_cmd(const QuadParams& p, const Environment& env, double /*t_s*/) {
  return scaled_hover(p, env, {1.03, 1.03, 0.97, 0.97});
}

// Front rotors (motors 1, 3) +2% for 0.1 s starting at t = 0.5 s: a nose-up kick. With
// no controller yet, the vehicle keeps pitching and slides backward.
MotorCommand pitch_kick_cmd(const QuadParams& p, const Environment& env, double t_s) {
  if (t_s >= 0.5 && t_s < 0.6) return scaled_hover(p, env, {1.02, 1.0, 1.02, 1.0});
  return sim::hover_command(p, env);
}

// What a closed-loop scenario asks the controller for at time t.
struct Target {
  bool attitude_mode = false;  // false: fly to pos; true: hold q_sp with thrust_N
  gnc::PositionSetpoint pos{};
  gnc::Quatf q_sp = gnc::Quatf::identity();
  float thrust_N = 0.0f;
};

constexpr float kWeightN = 1.5f * 9.80665f;  // default_quad_model() mass times g
constexpr float kDegToRadF = std::numbers::pi_v<float> / 180.0f;

gnc::PositionSetpoint at(float n, float e, float d, float yaw_deg = 0.0f) {
  gnc::PositionSetpoint sp;
  sp.pos_ned_m = {n, e, d};
  sp.yaw_rad = yaw_deg * kDegToRadF;
  return sp;
}

Target hold_target(double /*t_s*/) { return {false, at(0, 0, -10), {}, 0}; }
Target pos_step_target(double t_s) { return {false, at(t_s < 1.0 ? 0.0f : 1.0f, 0, -10), {}, 0}; }
Target yaw_step_target(double t_s) {
  return {false, at(0, 0, -10, t_s < 1.0 ? 0.0f : 90.0f), {}, 0};
}
Target square_target(double t_s) {
  if (t_s < 1.0) return {false, at(0, 0, -10), {}, 0};
  if (t_s < 5.0) return {false, at(5, 0, -10), {}, 0};
  if (t_s < 9.0) return {false, at(5, 5, -10), {}, 0};
  if (t_s < 13.0) return {false, at(0, 5, -10), {}, 0};
  return {false, at(0, 0, -10), {}, 0};
}
// Attitude mode: level, then a 10 deg right roll at t = 0.5 s with the thrust raised so
// the vertical component still carries the weight.
Target att_step_target(double t_s) {
  const float roll = t_s < 0.5 ? 0.0f : 10.0f * kDegToRadF;
  return {
      true, {}, gnc::from_euler321(gnc::Euler321f{roll, 0.0f, 0.0f}), kWeightN / std::cos(roll)};
}

struct Scenario {
  std::string_view name;
  std::string_view description;
  double duration_s;
  Vec3d start_pos_ned_m;
  gnc::Euler321d start_attitude;
  Vec3d wind_ned_mps;
  bool rotors_start_at_hover;
  MotorCommand (*open_loop)(const QuadParams&, const Environment&, double);  // or nullptr
  Target (*closed_loop)(double);                                             // or nullptr
};

constexpr gnc::Euler321d kLevel{};
constexpr Vec3d kCalm{};
constexpr double kDeg = std::numbers::pi / 180.0;

constexpr std::array<Scenario, 10> kScenarios{{
    // Open loop: motor commands only, no controller.
    {"hover",
     "open loop: hover trim held for 10 s",
     10.0,
     {0, 0, -10},
     kLevel,
     kCalm,
     true,
     hover_cmd,
     nullptr},
    {"free_fall",
     "open loop: rotors off from 100 m",
     4.0,
     {0, 0, -100},
     kLevel,
     kCalm,
     false,
     motors_off_cmd,
     nullptr},
    {"yaw_spin",
     "open loop: CCW rotors +3%, CW rotors -3%",
     5.0,
     {0, 0, -10},
     kLevel,
     kCalm,
     true,
     yaw_spin_cmd,
     nullptr},
    {"pitch_kick",
     "open loop: front rotors +2% for 0.1 s",
     3.0,
     {0, 0, -10},
     kLevel,
     kCalm,
     true,
     pitch_kick_cmd,
     nullptr},
    // Closed loop: the flight software's cascaded controller flies on truth at 250 Hz.
    {"hold",
     "recover from 3.2 m away and tilted to a hover at (0, 0, -10)",
     8.0,
     {2, -1.5, -8},
     {10 * kDeg, -5 * kDeg, 30 * kDeg},
     kCalm,
     true,
     nullptr,
     hold_target},
    {"pos_step",
     "1 m step north at t = 1 s",
     6.0,
     {0, 0, -10},
     kLevel,
     kCalm,
     true,
     nullptr,
     pos_step_target},
    {"att_step",
     "attitude mode: 10 deg roll step at t = 0.5 s",
     2.5,
     {0, 0, -10},
     kLevel,
     kCalm,
     true,
     nullptr,
     att_step_target},
    {"yaw_step",
     "90 deg yaw step at t = 1 s while holding position",
     6.0,
     {0, 0, -10},
     kLevel,
     kCalm,
     true,
     nullptr,
     yaw_step_target},
    {"square",
     "5 m square, a new corner every 4 s",
     18.0,
     {0, 0, -10},
     kLevel,
     kCalm,
     true,
     nullptr,
     square_target},
    {"wind_hold",
     "hold position in a steady 3 m/s wind blowing north",
     15.0,
     {0, 0, -10},
     kLevel,
     {3, 0, 0},
     true,
     nullptr,
     hold_target},
}};

void write_header(std::ostream& out) {
  out << "t_s,pos_n_m,pos_e_m,pos_d_m,vel_n_mps,vel_e_mps,vel_d_mps,q_w,q_x,q_y,q_z,"
         "roll_deg,pitch_deg,yaw_deg,omega_x_radps,omega_y_radps,omega_z_radps";
  for (std::size_t i = 1; i <= sim::kNumRotors; ++i) out << ",rotor" << i << "_radps";
  for (std::size_t i = 1; i <= sim::kNumRotors; ++i) out << ",rotor" << i << "_cmd_radps";
  for (std::size_t i = 1; i <= sim::kNumRotors; ++i) out << ",u" << i;
  out << ",pos_sp_n_m,pos_sp_e_m,pos_sp_d_m,vel_sp_n_mps,vel_sp_e_mps,vel_sp_d_mps,"
         "roll_sp_deg,pitch_sp_deg,yaw_sp_deg,rate_sp_x_radps,rate_sp_y_radps,rate_sp_z_radps,"
         "thrust_sp_N,saturated\n";
}

// Controller columns are "nan" for open-loop runs.
void write_row(std::ostream& out, double t_s, const QuadParams& p, const QuadState& x,
               const MotorCommand& cmd, const Target* target, const gnc::ControllerStatus* status) {
  constexpr double kRadToDeg = 180.0 / std::numbers::pi;
  const sim::RigidBodyState& b = x.body;
  const gnc::Euler321d e = gnc::to_euler321(b.q_nb);
  out << t_s << ',' << b.pos_ned_m.x << ',' << b.pos_ned_m.y << ',' << b.pos_ned_m.z << ','
      << b.vel_ned_mps.x << ',' << b.vel_ned_mps.y << ',' << b.vel_ned_mps.z << ',' << b.q_nb.w
      << ',' << b.q_nb.x << ',' << b.q_nb.y << ',' << b.q_nb.z << ',' << e.roll * kRadToDeg << ','
      << e.pitch * kRadToDeg << ',' << e.yaw * kRadToDeg << ',' << b.omega_b_radps.x << ','
      << b.omega_b_radps.y << ',' << b.omega_b_radps.z;
  for (double speed : x.rotor_speed_radps) out << ',' << speed;
  for (double u : cmd.u) out << ',' << std::clamp(u, 0.0, 1.0) * p.rotor_speed_max_radps;
  for (double u : cmd.u) out << ',' << u;
  if (target == nullptr || status == nullptr) {
    for (int i = 0; i < 13; ++i) out << ",nan";
    out << ",0\n";
    return;
  }
  // Controller values are float; widen them explicitly for the log.
  const auto d = [](float v) { return static_cast<double>(v); };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const bool pos_mode = !target->attitude_mode;
  const gnc::Euler321f es = gnc::to_euler321(status->q_sp);
  const gnc::Vec3f& pos_sp = target->pos.pos_ned_m;
  const gnc::Vec3f& vel_sp = status->vel_sp_ned_mps;
  const gnc::Vec3f& rate_sp = status->rate_sp_b_radps;
  out << ',' << (pos_mode ? d(pos_sp.x) : nan) << ',' << (pos_mode ? d(pos_sp.y) : nan) << ','
      << (pos_mode ? d(pos_sp.z) : nan) << ',' << (pos_mode ? d(vel_sp.x) : nan) << ','
      << (pos_mode ? d(vel_sp.y) : nan) << ',' << (pos_mode ? d(vel_sp.z) : nan) << ','
      << d(es.roll) * kRadToDeg << ',' << d(es.pitch) * kRadToDeg << ',' << d(es.yaw) * kRadToDeg
      << ',' << d(rate_sp.x) << ',' << d(rate_sp.y) << ',' << d(rate_sp.z) << ','
      << d(status->request.thrust_N) << ',' << (status->saturated ? 1 : 0) << '\n';
}

int usage() {
  std::cerr << "usage: kestrel_sim <scenario> <output.csv>\nscenarios:\n";
  for (const Scenario& s : kScenarios)
    std::cerr << "  " << s.name << "  (" << s.description << ")\n";
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) return usage();
  const std::string_view requested = argv[1];
  const Scenario* scenario = nullptr;
  for (const Scenario& s : kScenarios) {
    if (s.name == requested) scenario = &s;
  }
  if (scenario == nullptr) return usage();

  std::ofstream out(argv[2]);
  if (!out) {
    std::cerr << "kestrel_sim: cannot open " << argv[2] << " for writing\n";
    return 1;
  }
  out << std::setprecision(10);
  write_header(out);

  const QuadParams p = sim::default_quad_params();
  Environment env{};
  env.wind_ned_mps = scenario->wind_ned_mps;
  QuadState x;
  if (scenario->rotors_start_at_hover) {
    x = sim::hover_state(p, env, scenario->start_pos_ned_m);
  } else {
    x.body.pos_ned_m = scenario->start_pos_ned_m;
  }
  x.body.q_nb = gnc::from_euler321(scenario->start_attitude);

  gnc::QuadController controller(gnc::default_quad_model(), gnc::default_controller_gains());
  Target target{};
  MotorCommand cmd{};
  const bool closed_loop = scenario->closed_loop != nullptr;

  const auto end_us = static_cast<std::uint64_t>(scenario->duration_s * 1e6 + 0.5);
  const double dt_s = static_cast<double>(kPhysicsDtUs) * 1e-6;
  for (std::uint64_t t_us = 0;; t_us += kPhysicsDtUs) {
    const double t_s = static_cast<double>(t_us) * 1e-6;
    if (!closed_loop) {
      cmd = scenario->open_loop(p, env, t_s);
    } else if (t_us % kControlEveryUs == 0) {  // controller output held between its ticks
      target = scenario->closed_loop(t_s);
      const gnc::VehicleState fsw_state = sim::to_fsw_state(x);
      const float control_dt = static_cast<float>(kControlEveryUs) * 1e-6f;
      cmd = sim::to_motor_command(
          target.attitude_mode
              ? controller.step_attitude(fsw_state, target.q_sp, target.thrust_N, control_dt)
              : controller.step(fsw_state, target.pos, control_dt));
    }
    if (t_us % kLogEveryUs == 0) {
      write_row(out, t_s, p, x, cmd, closed_loop ? &target : nullptr,
                closed_loop ? &controller.status() : nullptr);
    }
    if (t_us >= end_us) break;
    x = sim::quad_step(p, x, cmd, env, dt_s);
  }
  out.close();
  if (!out) {
    std::cerr << "kestrel_sim: failed writing " << argv[2] << '\n';
    return 1;
  }
  std::cout << "kestrel_sim: wrote " << scenario->name << " (" << scenario->duration_s << " s) to "
            << argv[2] << '\n';
  return 0;
}
