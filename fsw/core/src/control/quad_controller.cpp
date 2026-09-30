#include "gnc/control/quad_controller.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace gnc {

QuadControllerGains default_controller_gains() {
  // Chosen from the inside out, each loop a few times slower than the one it drives, then
  // checked in closed loop against the plant, including one with +-30% inertia, a 50%
  // slower motor and +-10% thrust coefficient:
  //   motor lag        tau = 30 ms, a pole at 33 rad/s
  //   rate loop        kp 22 (crossover ~22 rad/s, below the motor pole). kd 0.2 adds the
  //                    phase lead the motor lag and 4 ms control period take away; without
  //                    it the mismatched plant overshoots a 10 deg step by 28%, with it 8%.
  //   attitude loop    kp 6 1/s, ~3.7x slower than the rate loop
  //   velocity loop    kp 4 1/s, ki 1 1/s^2 horizontal (5 and 2 vertical)
  //   position loop    kp 1.2 1/s horizontal, ~3.3x slower than velocity (2 vertical)
  QuadControllerGains g;
  g.pos_kp = {1.2f, 1.2f, 2.0f};
  g.vel_max_xy_mps = 5.0f;
  g.vel_max_z_mps = 2.0f;
  g.vel.kp = {4.0f, 4.0f, 5.0f};
  g.vel.ki = {1.0f, 1.0f, 2.0f};
  g.vel.integrator_limit = {3.0f, 3.0f, 4.0f};
  g.att.kp = {6.0f, 6.0f, 4.0f};
  g.att.rate_max_radps = {3.8f, 3.8f, 1.6f};  // about 220, 220, 90 deg/s
  g.rate.kp = {22.0f, 22.0f, 8.0f};
  g.rate.ki = {20.0f, 20.0f, 5.0f};
  g.rate.kd = {0.2f, 0.2f, 0.0f};
  g.rate.integrator_limit = {20.0f, 20.0f, 5.0f};
  g.rate.derivative_cutoff_hz = 30.0f;
  g.tilt_max_rad = 35.0f * std::numbers::pi_v<real> / 180.0f;
  return g;
}

QuadController::QuadController(const QuadModel& model, const QuadControllerGains& gains,
                               real gravity_mps2)
    : model_(model),
      gains_(gains),
      gravity_mps2_(gravity_mps2),
      mixer_(model),
      vel_pid_(gains.vel),
      rate_pid_(gains.rate) {}

void QuadController::reset() {
  vel_pid_.reset();
  rate_pid_.reset();
  status_ = {};
}

MotorOutputs QuadController::step(const VehicleState& x, const PositionSetpoint& sp, real dt_s) {
  // Position -> velocity setpoint, speed-limited horizontally and vertically.
  const Vec3f pos_error = sp.pos_ned_m - x.pos_ned_m;
  Vec3f vel_sp = Vec3f{gains_.pos_kp.x * pos_error.x, gains_.pos_kp.y * pos_error.y,
                       gains_.pos_kp.z * pos_error.z} +
                 sp.vel_ff_ned_mps;
  const real speed_xy = std::hypot(vel_sp.x, vel_sp.y);
  if (speed_xy > gains_.vel_max_xy_mps) {
    vel_sp.x *= gains_.vel_max_xy_mps / speed_xy;
    vel_sp.y *= gains_.vel_max_xy_mps / speed_xy;
  }
  vel_sp.z = std::clamp(vel_sp.z, -gains_.vel_max_z_mps, gains_.vel_max_z_mps);

  // Velocity -> acceleration. Hold the integrator while the last command was saturated
  // or tilt-limited, so it doesn't wind up asking for thrust the vehicle can't give.
  const bool freeze = status_.saturated || status_.tilt_limited;
  const Vec3f acc_sp =
      vel_pid_.update(vel_sp - x.vel_ned_mps, x.vel_ned_mps, dt_s, freeze) + sp.acc_ff_ned_mps2;

  // Acceleration -> attitude and collective thrust.
  const ThrustAttitude ta = thrust_to_attitude(acc_sp, sp.yaw_rad, x.q_nb, model_.mass_kg,
                                               gravity_mps2_, gains_.tilt_max_rad);
  const MotorOutputs out = inner_loops(x, ta.q_sp, ta.thrust_N, dt_s);
  status_.vel_sp_ned_mps = vel_sp;
  status_.acc_sp_ned_mps2 = acc_sp;
  status_.tilt_limited = ta.tilt_limited;
  status_.vel_integral = vel_pid_.integral();
  return out;
}

MotorOutputs QuadController::step_attitude(const VehicleState& x, const Quatf& q_sp, real thrust_N,
                                           real dt_s) {
  status_.tilt_limited = false;
  return inner_loops(x, q_sp, thrust_N, dt_s);
}

MotorOutputs QuadController::inner_loops(const VehicleState& x, const Quatf& q_sp, real thrust_N,
                                         real dt_s) {
  const Vec3f rate_sp = attitude_rate_setpoint(x.q_nb, q_sp, gains_.att);
  const Vec3f& w = x.omega_b_radps;
  const Vec3f alpha_sp = rate_pid_.update(rate_sp - w, w, dt_s, status_.saturated);
  // tau = J alpha + w x (J w): the second term cancels the gyroscopic coupling.
  const Mat3f& j = model_.inertia_b_kgm2;
  const ThrustTorque request{thrust_N, j * alpha_sp + cross(w, j * w)};
  const MixerResult mixed = mixer_.mix(request);

  status_.q_sp = q_sp;
  status_.rate_sp_b_radps = rate_sp;
  status_.request = request;
  status_.achieved = mixed.achieved;
  status_.saturated = mixed.saturated;
  status_.rate_integral = rate_pid_.integral();
  return mixed.outputs;
}

}  // namespace gnc
