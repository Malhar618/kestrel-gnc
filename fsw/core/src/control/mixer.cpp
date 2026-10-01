#include "gnc/control/mixer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace gnc {

namespace {

constexpr std::size_t kN = kNumRotors;

// Inverts a row-major 4x4 matrix by Gauss-Jordan elimination with partial pivoting.
bool invert4(const std::array<real, 16>& a, std::array<real, 16>& inv) {
  std::array<real, 16> m = a;
  inv = {};
  for (std::size_t i = 0; i < 4; ++i) inv[5 * i] = 1;
  for (std::size_t col = 0; col < 4; ++col) {
    std::size_t pivot = col;
    for (std::size_t r = col + 1; r < 4; ++r) {
      if (std::abs(m[4 * r + col]) > std::abs(m[4 * pivot + col])) pivot = r;
    }
    if (std::abs(m[4 * pivot + col]) < real(1e-9)) return false;
    for (std::size_t c = 0; c < 4; ++c) {
      std::swap(m[4 * col + c], m[4 * pivot + c]);
      std::swap(inv[4 * col + c], inv[4 * pivot + c]);
    }
    const real scale = real(1) / m[4 * col + col];
    for (std::size_t c = 0; c < 4; ++c) {
      m[4 * col + c] *= scale;
      inv[4 * col + c] *= scale;
    }
    for (std::size_t r = 0; r < 4; ++r) {
      if (r == col) continue;
      const real factor = m[4 * r + col];
      for (std::size_t c = 0; c < 4; ++c) {
        m[4 * r + c] -= factor * m[4 * col + c];
        inv[4 * r + c] -= factor * inv[4 * col + c];
      }
    }
  }
  return true;
}

}  // namespace

Mixer::Mixer(const QuadModel& model) : model_(model) {
  const real yaw_per_thrust = model.torque_coeff / model.thrust_coeff;
  for (std::size_t i = 0; i < kN; ++i) {
    alloc_[0 * kN + i] = 1;                          // collective thrust
    alloc_[1 * kN + i] = -model.rotor_pos_b_m[i].y;  // roll:  tau_x = -y T
    alloc_[2 * kN + i] = model.rotor_pos_b_m[i].x;   // pitch: tau_y =  x T
    alloc_[3 * kN + i] = static_cast<real>(model.rotor_spin[i]) * yaw_per_thrust;
  }
  valid_ = invert4(alloc_, alloc_inv_);
  // Saturation handling shifts collective thrust, so every rotor must add thrust when the
  // collective request goes up.
  for (std::size_t i = 0; i < kN; ++i) valid_ = valid_ && alloc_inv_[i * 4 + 0] > 0;
}

ThrustTorque Mixer::wrench_of(const std::array<real, kNumRotors>& f) const {
  std::array<real, 4> w{};
  for (std::size_t r = 0; r < 4; ++r) {
    for (std::size_t i = 0; i < kN; ++i) w[r] += alloc_[r * kN + i] * f[i];
  }
  return {w[0], {w[1], w[2], w[3]}};
}

MixerResult Mixer::mix(const ThrustTorque& request) const {
  MixerResult result;
  const Vec3f& torque = request.torque_b_Nm;
  const bool finite = std::isfinite(request.thrust_N) && std::isfinite(torque.x) &&
                      std::isfinite(torque.y) && std::isfinite(torque.z);
  if (!valid_ || !finite) {  // nothing sensible to allocate: motors off, flagged
    result.saturated = true;
    return result;
  }
  const real f_max = model_.max_rotor_thrust_N();

  // Each rotor's share of the request, split by priority.
  std::array<real, kN> collective{}, roll_pitch{}, yaw{}, per_newton{};
  for (std::size_t i = 0; i < kN; ++i) {
    per_newton[i] = alloc_inv_[i * 4 + 0];
    collective[i] = per_newton[i] * request.thrust_N;
    roll_pitch[i] = alloc_inv_[i * 4 + 1] * request.torque_b_Nm.x +
                    alloc_inv_[i * 4 + 2] * request.torque_b_Nm.y;
    yaw[i] = alloc_inv_[i * 4 + 3] * request.torque_b_Nm.z;
  }

  // 1. Roll/pitch must fit inside the rotors' range on their own; if not, scale them down.
  const auto [rp_lo, rp_hi] = std::minmax_element(roll_pitch.begin(), roll_pitch.end());
  const real rp_range = *rp_hi - *rp_lo;
  if (rp_range > f_max) {
    const real scale = f_max / rp_range;
    for (real& f : roll_pitch) f *= scale;
    result.saturated = true;
  }

  // 2. Shift collective thrust as little as possible so every rotor lands in [0, f_max].
  real shift_lo = -std::numeric_limits<real>::infinity();
  real shift_hi = std::numeric_limits<real>::infinity();
  for (std::size_t i = 0; i < kN; ++i) {
    const real base = collective[i] + roll_pitch[i];
    shift_lo = std::max(shift_lo, -base / per_newton[i]);
    shift_hi = std::min(shift_hi, (f_max - base) / per_newton[i]);
  }
  const real shift = std::clamp(real(0), shift_lo, std::max(shift_lo, shift_hi));
  if (shift != 0) result.saturated = true;

  // 3. Add as much of the yaw request as still fits.
  std::array<real, kN> base{};
  real yaw_scale = 1;
  for (std::size_t i = 0; i < kN; ++i) {
    base[i] = collective[i] + roll_pitch[i] + per_newton[i] * shift;
    if (yaw[i] > 0) yaw_scale = std::min(yaw_scale, (f_max - base[i]) / yaw[i]);
    if (yaw[i] < 0) yaw_scale = std::min(yaw_scale, -base[i] / yaw[i]);
  }
  yaw_scale = std::max(yaw_scale, real(0));
  if (yaw_scale < 1) result.saturated = true;

  // 4. Rotor thrust -> rotor speed -> command. The clamp only absorbs rounding.
  for (std::size_t i = 0; i < kN; ++i) {
    const real f = std::clamp(base[i] + yaw_scale * yaw[i], real(0), f_max);
    result.rotor_thrust_N[i] = f;
    result.outputs.u[i] =
        std::sqrt(f / model_.thrust_coeff) / model_.rotor_speed_max_radps;  // T = k w^2
  }
  result.achieved = wrench_of(result.rotor_thrust_N);
  return result;
}

}  // namespace gnc
