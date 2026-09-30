#pragma once

#include "gnc/angles.hpp"
#include "gnc/control/control_laws.hpp"
#include "gnc/control/mixer.hpp"
#include "gnc/control/pid.hpp"
#include "gnc/control/quad_model.hpp"
#include "gnc/math/quat.hpp"
#include "gnc/math/vec3.hpp"
#include "gnc/types.hpp"

namespace gnc {

struct QuadControllerGains {
  Vec3f pos_kp{};           // velocity setpoint per metre of position error (1/s)
  real vel_max_xy_mps = 0;  // horizontal speed limit
  real vel_max_z_mps = 0;   // vertical speed limit
  PidGains3 vel{};          // velocity error -> acceleration (m/s^2)
  AttitudeGains att{};      // attitude error -> body rate
  PidGains3 rate{};         // body-rate error -> angular acceleration (rad/s^2)
  real tilt_max_rad = 0;
};

/// Gains for default_quad_model(), with the loop bandwidths they were chosen from.
QuadControllerGains default_controller_gains();

struct PositionSetpoint {
  Vec3f pos_ned_m{};
  real yaw_rad = 0;
  Vec3f vel_ff_ned_mps{};  // feed-forward from guidance (zero until M5)
  Vec3f acc_ff_ned_mps2{};
};

/// Every intermediate setpoint of the last step, for logging and tests.
struct ControllerStatus {
  Vec3f vel_sp_ned_mps{};
  Vec3f acc_sp_ned_mps2{};
  Quatf q_sp = Quatf::identity();
  Vec3f rate_sp_b_radps{};
  ThrustTorque request{};
  ThrustTorque achieved{};
  bool saturated = false;     // mixer reduced the request
  bool tilt_limited = false;  // horizontal acceleration was cut back
  Vec3f vel_integral{};       // integrator states, to check anti-windup
  Vec3f rate_integral{};
};

/// Cascaded quadrotor controller:
///   position -P-> velocity -PI-> acceleration -> attitude + thrust
///   attitude -P-> body rate -PID-> angular acceleration -> torque -> mixer -> motors
/// All loops run at the rate step() is called. Integrators freeze while the actuators
/// are saturated (anti-windup). No heap, no exceptions: the whole state is these members.
class QuadController {
 public:
  QuadController(const QuadModel& model, const QuadControllerGains& gains,
                 real gravity_mps2 = real(9.80665));

  /// Full cascade: fly to a position and heading.
  MotorOutputs step(const VehicleState& x, const PositionSetpoint& sp, real dt_s);

  /// Inner loops only: hold an attitude with a given collective thrust.
  MotorOutputs step_attitude(const VehicleState& x, const Quatf& q_sp, real thrust_N, real dt_s);

  void reset();
  const ControllerStatus& status() const { return status_; }

 private:
  MotorOutputs inner_loops(const VehicleState& x, const Quatf& q_sp, real thrust_N, real dt_s);

  QuadModel model_;
  QuadControllerGains gains_;
  real gravity_mps2_;
  Mixer mixer_;
  Pid3 vel_pid_;
  Pid3 rate_pid_;
  ControllerStatus status_{};
};

}  // namespace gnc
