// kestrel_sim: flies an open-loop scenario on the quadrotor plant and writes a CSV log.
//
//   kestrel_sim <scenario> <output.csv>
//
// Physics runs at 1 kHz with integer-microsecond time; the log is written at 100 Hz.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string_view>

#include "gnc/math/euler.hpp"
#include "sim/quadrotor.hpp"

namespace {

using gnc::Vec3d;
using sim::Environment;
using sim::MotorCommand;
using sim::QuadParams;
using sim::QuadState;

constexpr std::uint64_t kPhysicsDtUs = 1000;  // 1 kHz
constexpr std::uint64_t kLogEveryUs = 10000;  // 100 Hz

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

struct Scenario {
  std::string_view name;
  std::string_view description;
  double duration_s;
  Vec3d start_pos_ned_m;
  bool rotors_start_at_hover;
  MotorCommand (*command)(const QuadParams&, const Environment&, double);
};

constexpr std::array<Scenario, 4> kScenarios{{
    {"hover", "hover trim held for 10 s", 10.0, {0.0, 0.0, -10.0}, true, hover_cmd},
    {"free_fall", "rotors off from 100 m", 4.0, {0.0, 0.0, -100.0}, false, motors_off_cmd},
    {"yaw_spin", "CCW rotors +3%, CW rotors -3%", 5.0, {0.0, 0.0, -10.0}, true, yaw_spin_cmd},
    {"pitch_kick", "front rotors +2% for 0.1 s", 3.0, {0.0, 0.0, -10.0}, true, pitch_kick_cmd},
}};

void write_header(std::ostream& out) {
  out << "t_s,pos_n_m,pos_e_m,pos_d_m,vel_n_mps,vel_e_mps,vel_d_mps,q_w,q_x,q_y,q_z,"
         "roll_deg,pitch_deg,yaw_deg,omega_x_radps,omega_y_radps,omega_z_radps";
  for (std::size_t i = 1; i <= sim::kNumRotors; ++i) out << ",rotor" << i << "_radps";
  for (std::size_t i = 1; i <= sim::kNumRotors; ++i) out << ",rotor" << i << "_cmd_radps";
  for (std::size_t i = 1; i <= sim::kNumRotors; ++i) out << ",u" << i;
  out << '\n';
}

void write_row(std::ostream& out, double t_s, const QuadParams& p, const QuadState& x,
               const MotorCommand& cmd) {
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
  out << '\n';
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
  const Environment env{};
  QuadState x;
  if (scenario->rotors_start_at_hover) {
    x = sim::hover_state(p, env, scenario->start_pos_ned_m);
  } else {
    x.body.pos_ned_m = scenario->start_pos_ned_m;
  }

  const auto end_us = static_cast<std::uint64_t>(scenario->duration_s * 1e6 + 0.5);
  const double dt_s = static_cast<double>(kPhysicsDtUs) * 1e-6;
  for (std::uint64_t t_us = 0;; t_us += kPhysicsDtUs) {
    const double t_s = static_cast<double>(t_us) * 1e-6;
    const MotorCommand cmd = scenario->command(p, env, t_s);
    if (t_us % kLogEveryUs == 0) write_row(out, t_s, p, x, cmd);
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
