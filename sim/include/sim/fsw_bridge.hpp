#pragma once

#include "gnc/types.hpp"
#include "sim/quadrotor.hpp"

namespace sim {

/// The simulator's truth as the flight software sees it (double -> float). Until the
/// estimator exists (M3), the controller flies on this directly.
gnc::VehicleState to_fsw_state(const QuadState& x);

/// Flight-software motor outputs as plant commands (float -> double).
MotorCommand to_motor_command(const gnc::MotorOutputs& out);

}  // namespace sim
