#pragma once

namespace gnc {

using real = float;  // flight software runs in single precision (Cortex-M4F FPU)

/// Wrap an angle in radians to the interval (-pi, pi].
real wrap_pi(real angle_rad) noexcept;

}  // namespace gnc
