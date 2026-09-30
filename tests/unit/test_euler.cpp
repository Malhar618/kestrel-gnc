#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>

#include "gnc/math/euler.hpp"
#include "test_support.hpp"

namespace gnc::test {
namespace {

template <typename T>
class EulerTest : public ::testing::Test {};
TYPED_TEST_SUITE(EulerTest, Scalars);

template <typename T>
constexpr T kHalfPi = std::numbers::pi_v<T> / T(2);

TYPED_TEST(EulerTest, DcmMatchesTextbook321Matrix) {
  using T = TypeParam;
  // Independent check against the closed-form C_nb = Rz(yaw) Ry(pitch) Rx(roll).
  const T r = T(0.1), p = T(0.2), y = T(0.3);
  const T cr = std::cos(r), sr = std::sin(r), cp = std::cos(p), sp = std::sin(p);
  const T cy = std::cos(y), sy = std::sin(y);
  // clang-format off
  const Mat3<T> expected{{cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy,
                          cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy,
                          -sp, sr * cp, cr * cp}};
  // clang-format on
  EXPECT_TRUE(mat_near(to_dcm(from_euler321(Euler321<T>{r, p, y})), expected, T(10) * tol<T>()));
}

TYPED_TEST(EulerTest, RoundTripAwayFromGimbalLock) {
  using T = TypeParam;
  const T rolls[] = {T(-3), T(-1), T(0), T(0.5), T(2.9)};
  const T pitches[] = {T(-1.4), T(-0.3), T(0), T(0.8), T(1.5)};
  const T yaws[] = {T(-3.1), T(-0.4), T(0), T(1), T(3)};
  // Error grows like 1/cos(pitch) near +-90 deg, hence the looser float bound.
  const T tolerance = std::is_same_v<T, float> ? T(1e-4) : T(1e-10);
  for (T r : rolls) {
    for (T p : pitches) {
      for (T y : yaws) {
        const Euler321<T> e = to_euler321(from_euler321(Euler321<T>{r, p, y}));
        KESTREL_EXPECT_NEAR(e.roll, r, tolerance);
        KESTREL_EXPECT_NEAR(e.pitch, p, tolerance);
        KESTREL_EXPECT_NEAR(e.yaw, y, tolerance);
      }
    }
  }
}

TYPED_TEST(EulerTest, GimbalLockKeepsTheRotationButNotTheAngles) {
  using T = TypeParam;
  const T tolerance = std::is_same_v<T, float> ? T(1e-3) : T(1e-7);

  // Pitch +90: only yaw - roll is observable. Expect roll = 0, yaw = 0.2 - 0.3.
  const Quat<T> up = from_euler321(Euler321<T>{T(0.3), kHalfPi<T>, T(0.2)});
  const Euler321<T> e_up = to_euler321(up);
  KESTREL_EXPECT_NEAR(e_up.pitch, kHalfPi<T>, tolerance);
  KESTREL_EXPECT_NEAR(e_up.roll, T(0), tolerance);
  KESTREL_EXPECT_NEAR(e_up.yaw, T(-0.1), tolerance);
  EXPECT_TRUE(same_rotation(from_euler321(e_up), up, tolerance));

  // Pitch -90: only yaw + roll is observable.
  const Quat<T> down = from_euler321(Euler321<T>{T(0.3), -kHalfPi<T>, T(0.2)});
  const Euler321<T> e_down = to_euler321(down);
  KESTREL_EXPECT_NEAR(e_down.pitch, -kHalfPi<T>, tolerance);
  KESTREL_EXPECT_NEAR(e_down.yaw, T(0.5), tolerance);
  EXPECT_TRUE(same_rotation(from_euler321(e_down), down, tolerance));
}

TYPED_TEST(EulerTest, RoundTripNearGimbalLockKeepsTheAttitude) {
  using T = TypeParam;
  // Fuzz pitches within 1e-2 rad of +-90 deg (down to exactly 90) with random roll and yaw.
  // Individual angles are ill-defined there, but the attitude they describe must survive.
  const T tolerance = std::is_same_v<T, float> ? T(1e-3) : T(1e-7);
  std::uint32_t state = 12345u;
  const auto uniform = [&state]() {  // deterministic LCG in [0, 1)
    state = 1664525u * state + 1013904223u;
    return static_cast<double>(state >> 8) / 16777216.0;
  };
  double worst = 0.0;
  for (int i = 0; i < 4000; ++i) {
    const double gap = uniform() < 0.25 ? 0.0 : std::pow(10.0, -2.0 - 8.0 * uniform());
    const double sign = i % 2 == 0 ? 1.0 : -1.0;
    const Euler321<T> e{static_cast<T>((2.0 * uniform() - 1.0) * 3.1),
                        static_cast<T>(sign * (std::numbers::pi / 2.0 - gap)),
                        static_cast<T>((2.0 * uniform() - 1.0) * 3.1)};
    const Quat<T> q = from_euler321(e);
    const T err = angle_between(from_euler321(to_euler321(q)), q);
    worst = std::max(worst, static_cast<double>(err));
  }
  KESTREL_EXPECT_NEAR(worst, 0.0, tolerance);
}

}  // namespace
}  // namespace gnc::test
