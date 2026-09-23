#pragma once

#include <gtest/gtest.h>

#include <type_traits>

#include "gnc/math/mat3.hpp"
#include "gnc/math/quat.hpp"
#include "gnc/math/vec3.hpp"

// EXPECT_NEAR takes doubles; the explicit casts keep -Wdouble-promotion quiet
// when the arguments are float.
#define KESTREL_EXPECT_NEAR(actual, expected, tol) \
  EXPECT_NEAR(static_cast<double>(actual), static_cast<double>(expected), static_cast<double>(tol))

namespace gnc::test {

/// Both precisions run the same tests: float is what flies, double is what the sim uses.
using Scalars = ::testing::Types<float, double>;

/// Tolerance for a handful of floating-point operations in precision T.
template <typename T>
constexpr T tol() {
  return std::is_same_v<T, float> ? T(1e-5) : T(1e-12);
}

template <typename T>
::testing::AssertionResult vec_near(const Vec3<T>& a, const Vec3<T>& b, T tolerance) {
  const T err = norm(a - b);
  if (err <= tolerance) return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure()
         << "|a - b| = " << err << " > " << tolerance << "; a = [" << a.x << ", " << a.y << ", "
         << a.z << "], b = [" << b.x << ", " << b.y << ", " << b.z << "]";
}

/// q and -q are the same rotation, so compare after fixing the sign.
template <typename T>
::testing::AssertionResult same_rotation(const Quat<T>& a, const Quat<T>& b, T tolerance) {
  const Quat<T> ca = canonical(a);
  const Quat<T> cb = canonical(b);
  const T err = norm(ca + T(-1) * cb);
  if (err <= tolerance) return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure()
         << "quaternions differ by " << err << " > " << tolerance << "; a = [" << ca.w << ", "
         << ca.x << ", " << ca.y << ", " << ca.z << "], b = [" << cb.w << ", " << cb.x << ", "
         << cb.y << ", " << cb.z << "]";
}

template <typename T>
::testing::AssertionResult mat_near(const Mat3<T>& a, const Mat3<T>& b, T tolerance) {
  for (std::size_t i = 0; i < 9; ++i) {
    const T err = std::abs(a.m[i] - b.m[i]);
    if (err > tolerance) {
      return ::testing::AssertionFailure()
             << "element (" << i / 3 << ", " << i % 3 << ") differs by " << err;
    }
  }
  return ::testing::AssertionSuccess();
}

/// A spread of test attitudes, including angles near 180 deg that exercise
/// every branch of from_dcm().
template <typename T>
std::array<Quat<T>, 7> sample_attitudes() {
  return {Quat<T>::identity(),
          from_axis_angle(normalized(Vec3<T>{T(1), T(2), T(3)}), T(0.7)),
          from_axis_angle(normalized(Vec3<T>{T(-2), T(0.5), T(1)}), T(2.5)),
          from_axis_angle(Vec3<T>{T(1), T(0), T(0)}, T(3.1)),
          from_axis_angle(Vec3<T>{T(0), T(1), T(0)}, T(3.1)),
          from_axis_angle(Vec3<T>{T(0), T(0), T(1)}, T(3.1)),
          from_axis_angle(normalized(Vec3<T>{T(0.3), T(-1), T(0.2)}), T(-1.2))};
}

}  // namespace gnc::test
