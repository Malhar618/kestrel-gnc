#pragma once

#include <array>
#include <cstddef>

#include "gnc/math/vec3.hpp"
#include "sim/environment.hpp"
#include "sim/rigid_body.hpp"

namespace sim {

inline constexpr std::size_t kNumRotors = 4;

/// One rotor: hub position in body axes (relative to the CG) and spin direction
/// seen from above: +1 counter-clockwise, -1 clockwise.
struct Rotor {
  gnc::Vec3d pos_b_m{};
  int spin = 1;
};

struct QuadParams {
  RigidBodyParams body{};
  std::array<Rotor, kNumRotors> rotors{};
  double thrust_coeff = 0.0;           // T = thrust_coeff * w^2          (N per (rad/s)^2)
  double torque_coeff = 0.0;           // Q = torque_coeff * w^2          (N m per (rad/s)^2)
  double motor_time_constant_s = 0.0;  // first-order lag from command to rotor speed
  double rotor_speed_max_radps = 0.0;  // rotor speed at a command of 1
  gnc::Vec3d drag_coeff_b{};           // linear drag along body x, y, z  (N per m/s of airspeed)
};

/// Quad X in PX4 motor order (index 0 = motor 1):
///   1 front-right CCW, 2 rear-left CCW, 3 front-left CW, 4 rear-right CW.
std::array<Rotor, kNumRotors> quad_x_rotors(double arm_length_m);

/// A representative 450 mm class quadrotor (1.5 kg, thrust-to-weight about 3.3).
/// Placeholder values until the real vehicle is identified from measurements.
QuadParams default_quad_params();

struct QuadState {
  RigidBodyState body{};
  std::array<double, kNumRotors> rotor_speed_radps{};
};

QuadState operator+(const QuadState& a, const QuadState& b);
QuadState operator*(double s, const QuadState& x);

/// Normalized motor commands in PX4 motor order. 0 = stopped, 1 = full speed;
/// values outside [0, 1] are clamped. A command sets the rotor speed target
/// u * rotor_speed_max_radps, which the rotor reaches through a first-order lag.
struct MotorCommand {
  std::array<double, kNumRotors> u{};
};

/// Force and torque on the body about the CG, in body axes.
struct Wrench {
  gnc::Vec3d force_b_N{};
  gnc::Vec3d torque_b_Nm{};
};

/// Rotor thrust (along -z body), rotor reaction torques, the moments of the thrusts
/// about the CG, and linear aerodynamic drag on the airspeed (velocity minus wind).
Wrench quad_wrench(const QuadParams& p, const QuadState& x, const Environment& env);

QuadState quad_derivative(const QuadParams& p, const QuadState& x, const MotorCommand& cmd,
                          const Environment& env);

/// Advances the state by dt_s with RK4, holding the command constant. Then
/// renormalizes q_nb and clamps rotor speeds at zero.
QuadState quad_step(const QuadParams& p, const QuadState& x, const MotorCommand& cmd,
                    const Environment& env, double dt_s);

/// Rotor speed at which the four rotors together carry the vehicle's weight.
double hover_rotor_speed(const QuadParams& p, const Environment& env);

/// The command that holds every rotor at hover speed.
MotorCommand hover_command(const QuadParams& p, const Environment& env);

/// Level, facing north, at rest at pos_ned_m, with all rotors already at hover speed.
QuadState hover_state(const QuadParams& p, const Environment& env,
                      const gnc::Vec3d& pos_ned_m = {});

}  // namespace sim
