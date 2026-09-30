#include "sim/fsw_bridge.hpp"

namespace sim {

namespace {

gnc::Vec3f to_float(const gnc::Vec3d& v) {
  return {static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z)};
}

}  // namespace

gnc::VehicleState to_fsw_state(const QuadState& x) {
  const RigidBodyState& b = x.body;
  return {to_float(b.pos_ned_m), to_float(b.vel_ned_mps),
          gnc::normalized(gnc::Quatf{static_cast<float>(b.q_nb.w), static_cast<float>(b.q_nb.x),
                                     static_cast<float>(b.q_nb.y), static_cast<float>(b.q_nb.z)}),
          to_float(b.omega_b_radps)};
}

MotorCommand to_motor_command(const gnc::MotorOutputs& out) {
  MotorCommand cmd;
  for (std::size_t i = 0; i < kNumRotors; ++i) cmd.u[i] = static_cast<double>(out.u[i]);
  return cmd;
}

}  // namespace sim
