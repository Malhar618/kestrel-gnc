#include "gnc/angles.hpp"

#include <cmath>
#include <numbers>

namespace gnc {

real wrap_pi(real angle_rad) noexcept {
  constexpr real kPi = std::numbers::pi_v<real>;
  constexpr real kTwoPi = 2.0f * kPi;
  real a = std::fmod(angle_rad + kPi, kTwoPi);
  if (a <= 0.0f) a += kTwoPi;
  return a - kPi;
}

}  // namespace gnc
