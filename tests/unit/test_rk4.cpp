#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "gnc/math/rk4.hpp"
#include "gnc/math/vec3.hpp"
#include "test_support.hpp"

namespace gnc::test {
namespace {

double decay_error(double dt) {
  // x' = -x, x(0) = 1, exact x(1) = e^-1.
  const auto f = [](double /*t*/, double x) { return -x; };
  double x = 1.0;
  const int steps = static_cast<int>(std::lround(1.0 / dt));
  for (int i = 0; i < steps; ++i) x = rk4_step(f, i * dt, x, dt);
  return std::abs(x - std::exp(-1.0));
}

TEST(Rk4, ErrorShrinks16xWhenStepHalves) {
  // Global error ~ C dt^4, so e(dt) / e(dt/2) ~ 2^4 = 16.
  const double ratio = decay_error(0.1) / decay_error(0.05);
  EXPECT_GT(ratio, 14.0);
  EXPECT_LT(ratio, 18.0);
}

template <typename T>
class Rk4Typed : public ::testing::Test {};
TYPED_TEST_SUITE(Rk4Typed, Scalars);

TYPED_TEST(Rk4Typed, OneFullRevolutionReturnsToStart) {
  using T = TypeParam;
  // x' = omega x x spins x about z at 1 rev/s; after 1 s it must be back where it began.
  const Vec3<T> omega{T(0), T(0), T(2) * std::numbers::pi_v<T>};
  const auto f = [&](T /*t*/, const Vec3<T>& x) { return cross(omega, x); };
  const Vec3<T> x0{T(1), T(0), T(0.5)};
  Vec3<T> x = x0;
  const T dt = T(1e-3);
  for (int i = 0; i < 1000; ++i) x = rk4_step(f, T(0), x, dt);
  const T tolerance = std::is_same_v<T, float> ? T(1e-4) : T(1e-9);
  EXPECT_TRUE(vec_near(x, x0, tolerance));
}

}  // namespace
}  // namespace gnc::test
