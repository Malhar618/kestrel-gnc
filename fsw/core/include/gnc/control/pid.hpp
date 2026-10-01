#pragma once

#include <algorithm>
#include <numbers>

#include "gnc/angles.hpp"
#include "gnc/math/vec3.hpp"

namespace gnc {

/// Per-axis gains. The integrator stores the integral of ki * error, clamped to
/// +-integrator_limit, so changing ki never makes the output jump.
struct PidGains3 {
  Vec3f kp{};
  Vec3f ki{};
  Vec3f kd{};
  Vec3f integrator_limit{};
  real derivative_cutoff_hz = 0;  // first-order low-pass on the derivative; 0 = none
};

/// Three independent PID channels. The derivative acts on the measurement, not the
/// error, so a setpoint step doesn't kick the output.
class Pid3 {
 public:
  explicit Pid3(const PidGains3& gains) : gains_(gains) {}

  /// error = setpoint - measurement. While freeze_integrator is true (an actuator is
  /// saturated), the integrator holds its value: conditional-integration anti-windup.
  Vec3f update(const Vec3f& error, const Vec3f& measurement, real dt_s, bool freeze_integrator) {
    if (!freeze_integrator) {
      integral_ = clamp3(integral_ + dt_s * mul(gains_.ki, error), gains_.integrator_limit);
    }
    Vec3f rate{};
    if (has_previous_ && dt_s > 0) rate = (measurement - previous_) / dt_s;
    previous_ = measurement;
    if (gains_.derivative_cutoff_hz > 0 && has_previous_) {
      const real tau = real(1) / (real(2) * std::numbers::pi_v<real> * gains_.derivative_cutoff_hz);
      derivative_ = derivative_ + (dt_s / (dt_s + tau)) * (rate - derivative_);
    } else {
      derivative_ = rate;
    }
    has_previous_ = true;
    return mul(gains_.kp, error) + integral_ - mul(gains_.kd, derivative_);
  }

  void reset() {
    integral_ = {};
    derivative_ = {};
    previous_ = {};
    has_previous_ = false;
  }

  const Vec3f& integral() const { return integral_; }

 private:
  static Vec3f mul(const Vec3f& a, const Vec3f& b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
  static Vec3f clamp3(const Vec3f& v, const Vec3f& limit) {
    return {std::clamp(v.x, -limit.x, limit.x), std::clamp(v.y, -limit.y, limit.y),
            std::clamp(v.z, -limit.z, limit.z)};
  }

  PidGains3 gains_;
  Vec3f integral_{};
  Vec3f derivative_{};
  Vec3f previous_{};
  bool has_previous_ = false;
};

}  // namespace gnc
