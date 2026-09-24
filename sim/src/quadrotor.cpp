#include "sim/quadrotor.hpp"

#include <algorithm>
#include <cmath>

#include "gnc/math/mat3.hpp"
#include "gnc/math/quat.hpp"
#include "gnc/math/rk4.hpp"

namespace sim {

using gnc::Vec3d;

std::array<Rotor, kNumRotors> quad_x_rotors(double arm_length_m) {
  const double a = arm_length_m / std::sqrt(2.0);  // arms at 45 deg to the body axes
  return {{
      {{a, a, 0.0}, +1},    // motor 1: front-right, CCW
      {{-a, -a, 0.0}, +1},  // motor 2: rear-left,   CCW
      {{a, -a, 0.0}, -1},   // motor 3: front-left,  CW
      {{-a, a, 0.0}, -1},   // motor 4: rear-right,  CW
  }};
}

QuadParams default_quad_params() {
  QuadParams p;
  p.body = make_rigid_body(1.5, gnc::diag(Vec3d{0.020, 0.020, 0.035}));
  p.rotors = quad_x_rotors(0.225);
  p.thrust_coeff = 1.0e-5;  // hover at ~606 rad/s (~5800 rpm), 55% of max speed
  p.torque_coeff = 1.6e-7;  // torque/thrust ratio 0.016 m, typical of 10 in props
  p.motor_time_constant_s = 0.03;
  p.rotor_speed_max_radps = 1100.0;
  p.drag_coeff_b = {0.25, 0.25, 0.35};
  return p;
}

QuadState operator+(const QuadState& a, const QuadState& b) {
  QuadState out{a.body + b.body, {}};
  for (std::size_t i = 0; i < kNumRotors; ++i) {
    out.rotor_speed_radps[i] = a.rotor_speed_radps[i] + b.rotor_speed_radps[i];
  }
  return out;
}

QuadState operator*(double s, const QuadState& x) {
  QuadState out{s * x.body, {}};
  for (std::size_t i = 0; i < kNumRotors; ++i) {
    out.rotor_speed_radps[i] = s * x.rotor_speed_radps[i];
  }
  return out;
}

Wrench quad_wrench(const QuadParams& p, const QuadState& x, const Environment& env) {
  Wrench w;
  for (std::size_t i = 0; i < kNumRotors; ++i) {
    const double speed_sq = x.rotor_speed_radps[i] * x.rotor_speed_radps[i];
    const Vec3d thrust_b{0.0, 0.0, -p.thrust_coeff * speed_sq};  // rotors push up = -z body
    w.force_b_N = w.force_b_N + thrust_b;
    w.torque_b_Nm = w.torque_b_Nm + gnc::cross(p.rotors[i].pos_b_m, thrust_b);
    // A CCW rotor drags the airframe clockwise seen from above: +z (nose right) in FRD.
    w.torque_b_Nm.z += static_cast<double>(p.rotors[i].spin) * p.torque_coeff * speed_sq;
  }
  const Vec3d airspeed_b =
      gnc::rotate(gnc::conjugate(x.body.q_nb), x.body.vel_ned_mps - env.wind_ned_mps);
  w.force_b_N =
      w.force_b_N - Vec3d{p.drag_coeff_b.x * airspeed_b.x, p.drag_coeff_b.y * airspeed_b.y,
                          p.drag_coeff_b.z * airspeed_b.z};
  return w;
}

QuadState quad_derivative(const QuadParams& p, const QuadState& x, const MotorCommand& cmd,
                          const Environment& env) {
  const Wrench w = quad_wrench(p, x, env);
  QuadState dx{rigid_body_derivative(p.body, x.body, w.force_b_N, w.torque_b_Nm,
                                     Vec3d{0.0, 0.0, env.gravity_mps2}),
               {}};
  for (std::size_t i = 0; i < kNumRotors; ++i) {
    const double target = std::clamp(cmd.u[i], 0.0, 1.0) * p.rotor_speed_max_radps;
    dx.rotor_speed_radps[i] = (target - x.rotor_speed_radps[i]) / p.motor_time_constant_s;
  }
  return dx;
}

QuadState quad_step(const QuadParams& p, const QuadState& x, const MotorCommand& cmd,
                    const Environment& env, double dt_s) {
  const auto f = [&](double /*t*/, const QuadState& s) { return quad_derivative(p, s, cmd, env); };
  QuadState next = gnc::rk4_step(f, 0.0, x, dt_s);
  next.body.q_nb = gnc::normalized(next.body.q_nb);
  // RK4 can overshoot below zero only when dt exceeds ~2.8 motor time constants.
  for (double& speed : next.rotor_speed_radps) speed = std::max(speed, 0.0);
  return next;
}

double hover_rotor_speed(const QuadParams& p, const Environment& env) {
  const double weight_N = p.body.mass_kg * env.gravity_mps2;
  return std::sqrt(weight_N / (static_cast<double>(kNumRotors) * p.thrust_coeff));
}

MotorCommand hover_command(const QuadParams& p, const Environment& env) {
  const double u = hover_rotor_speed(p, env) / p.rotor_speed_max_radps;
  return {{u, u, u, u}};
}

QuadState hover_state(const QuadParams& p, const Environment& env, const Vec3d& pos_ned_m) {
  QuadState x;
  x.body.pos_ned_m = pos_ned_m;
  x.rotor_speed_radps.fill(hover_rotor_speed(p, env));
  return x;
}

}  // namespace sim
