#include <gtest/gtest.h>

#include <numbers>

#include "gnc/math/quat.hpp"
#include "gnc/math/rk4.hpp"
#include "test_support.hpp"

namespace gnc::test {
namespace {

template <typename T>
class QuatTest : public ::testing::Test {};
TYPED_TEST_SUITE(QuatTest, Scalars);

template <typename T>
constexpr T kPi = std::numbers::pi_v<T>;

// --- Conventions: these pin NED/FRD + Hamilton q_nb to physical intuition ---

TYPED_TEST(QuatTest, IdentityLeavesVectorsAlone) {
  using T = TypeParam;
  const Vec3<T> v{T(0.3), T(-2), T(1.5)};
  EXPECT_TRUE(vec_near(rotate(Quat<T>::identity(), v), v, tol<T>()));
}

TYPED_TEST(QuatTest, YawRight90PointsNoseEast) {
  using T = TypeParam;
  // Positive yaw about NED down (z) turns the nose from north toward east.
  const Quat<T> q = from_axis_angle(Vec3<T>{T(0), T(0), T(1)}, kPi<T> / T(2));
  EXPECT_TRUE(vec_near(rotate(q, Vec3<T>{T(1), T(0), T(0)}), Vec3<T>{T(0), T(1), T(0)}, tol<T>()));
  // ...and the right wing now points south.
  EXPECT_TRUE(vec_near(rotate(q, Vec3<T>{T(0), T(1), T(0)}), Vec3<T>{T(-1), T(0), T(0)}, tol<T>()));
}

TYPED_TEST(QuatTest, PitchUp90PointsNoseUp) {
  using T = TypeParam;
  // Positive pitch about body y (right wing) raises the nose; "up" is -z in NED.
  const Quat<T> q = from_axis_angle(Vec3<T>{T(0), T(1), T(0)}, kPi<T> / T(2));
  EXPECT_TRUE(vec_near(rotate(q, Vec3<T>{T(1), T(0), T(0)}), Vec3<T>{T(0), T(0), T(-1)}, tol<T>()));
}

TYPED_TEST(QuatTest, RollRight90PutsRightWingDown) {
  using T = TypeParam;
  const Quat<T> q = from_axis_angle(Vec3<T>{T(1), T(0), T(0)}, kPi<T> / T(2));
  EXPECT_TRUE(vec_near(rotate(q, Vec3<T>{T(0), T(1), T(0)}), Vec3<T>{T(0), T(0), T(1)}, tol<T>()));
}

TYPED_TEST(QuatTest, HamiltonUnitsMultiplyLikeIJK) {
  using T = TypeParam;
  const Quat<T> i{T(0), T(1), T(0), T(0)}, j{T(0), T(0), T(1), T(0)}, k{T(0), T(0), T(0), T(1)};
  EXPECT_TRUE(same_rotation(i * j, k, tol<T>()));
  const Quat<T> ji = j * i;  // = -k: the product does not commute
  KESTREL_EXPECT_NEAR(ji.z, T(-1), tol<T>());
  const Quat<T> ii = i * i;  // = -1
  KESTREL_EXPECT_NEAR(ii.w, T(-1), tol<T>());
}

// --- Algebra ---

TYPED_TEST(QuatTest, CompositionAppliesRightFactorFirst) {
  using T = TypeParam;
  const auto qs = sample_attitudes<T>();
  const Vec3<T> v{T(1), T(-0.5), T(2)};
  for (const auto& a : qs) {
    for (const auto& b : qs) {
      EXPECT_TRUE(vec_near(rotate(a * b, v), rotate(a, rotate(b, v)), T(10) * tol<T>()));
    }
  }
}

TYPED_TEST(QuatTest, ConjugateIsInverse) {
  using T = TypeParam;
  for (const auto& q : sample_attitudes<T>()) {
    EXPECT_TRUE(same_rotation(q * conjugate(q), Quat<T>::identity(), tol<T>()));
  }
}

TYPED_TEST(QuatTest, QAndMinusQAreTheSameRotation) {
  using T = TypeParam;
  const Vec3<T> v{T(0.2), T(1), T(-3)};
  for (const auto& q : sample_attitudes<T>()) {
    EXPECT_TRUE(vec_near(rotate(q, v), rotate(T(-1) * q, v), T(10) * tol<T>()));
  }
}

// --- DCM ---

TYPED_TEST(QuatTest, DcmIsAProperRotation) {
  using T = TypeParam;
  for (const auto& q : sample_attitudes<T>()) {
    const Mat3<T> c = to_dcm(q);
    EXPECT_TRUE(mat_near(c * transposed(c), Mat3<T>::identity(), T(10) * tol<T>()));
    KESTREL_EXPECT_NEAR(determinant(c), T(1), T(10) * tol<T>());
  }
}

TYPED_TEST(QuatTest, DcmAgreesWithQuaternionRotation) {
  using T = TypeParam;
  const Vec3<T> v{T(-1), T(0.25), T(0.5)};
  for (const auto& q : sample_attitudes<T>()) {
    EXPECT_TRUE(vec_near(to_dcm(q) * v, rotate(q, v), T(10) * tol<T>()));
  }
}

TYPED_TEST(QuatTest, DcmOfProductIsProductOfDcms) {
  using T = TypeParam;
  const auto qs = sample_attitudes<T>();
  for (const auto& a : qs) {
    for (const auto& b : qs) {
      EXPECT_TRUE(mat_near(to_dcm(a * b), to_dcm(a) * to_dcm(b), T(10) * tol<T>()));
    }
  }
}

TYPED_TEST(QuatTest, DcmRoundTripCoversEveryShepperdBranch) {
  using T = TypeParam;
  for (const auto& q : sample_attitudes<T>()) {
    EXPECT_TRUE(same_rotation(from_dcm(to_dcm(q)), q, T(10) * tol<T>()));
  }
}

// --- Exponential / logarithm maps ---

TYPED_TEST(QuatTest, RotationVectorRoundTrip) {
  using T = TypeParam;
  const Vec3<T> rvs[] = {{T(0.1), T(-0.2), T(0.3)},
                         {T(2), T(1), T(-1)},
                         {T(0), T(0), T(3.1)},           // near pi
                         {T(1e-7), T(-2e-7), T(5e-8)}};  // tiny: Taylor branch
  for (const auto& rv : rvs) {
    EXPECT_TRUE(vec_near(to_rotation_vector(from_rotation_vector(rv)), rv, T(10) * tol<T>()));
  }
}

TYPED_TEST(QuatTest, RotationVectorMatchesAxisAngle) {
  using T = TypeParam;
  const Vec3<T> axis = normalized(Vec3<T>{T(1), T(-1), T(2)});
  EXPECT_TRUE(
      same_rotation(from_rotation_vector(T(1.3) * axis), from_axis_angle(axis, T(1.3)), tol<T>()));
}

TYPED_TEST(QuatTest, LogPicksTheShortWayAround) {
  using T = TypeParam;
  // A 270 deg rotation about z is the same attitude as -90 deg; log returns the short one.
  const Quat<T> q = from_axis_angle(Vec3<T>{T(0), T(0), T(1)}, T(1.5) * kPi<T>);
  EXPECT_TRUE(
      vec_near(to_rotation_vector(q), Vec3<T>{T(0), T(0), -kPi<T> / T(2)}, T(10) * tol<T>()));
}

TYPED_TEST(QuatTest, AngleBetweenKnownAttitudes) {
  using T = TypeParam;
  const Quat<T> a = from_axis_angle(Vec3<T>{T(0), T(0), T(1)}, T(0.2));
  const Quat<T> b = from_axis_angle(Vec3<T>{T(0), T(0), T(1)}, T(0.9));
  KESTREL_EXPECT_NEAR(angle_between(a, b), T(0.7), T(10) * tol<T>());
}

// --- Kinematics ---

TYPED_TEST(QuatTest, IntegrateConstantYawRate) {
  using T = TypeParam;
  // 1 rad/s about z for 1 s, in 1000 steps, must equal a single 1 rad yaw.
  Quat<T> q = Quat<T>::identity();
  const Vec3<T> omega{T(0), T(0), T(1)};
  for (int i = 0; i < 1000; ++i) q = integrate(q, omega, T(1e-3));
  EXPECT_TRUE(
      same_rotation(q, from_axis_angle(Vec3<T>{T(0), T(0), T(1)}, T(1)), T(100) * tol<T>()));
}

TYPED_TEST(QuatTest, BodyRatesComposeOnTheRight) {
  using T = TypeParam;
  // Yaw 90 deg first, then a body-x rate. Body x now points east, so this is a
  // roll about east, not about north. Getting this backwards is the classic bug.
  const Quat<T> yawed = from_axis_angle(Vec3<T>{T(0), T(0), T(1)}, kPi<T> / T(2));
  const Quat<T> q = integrate(yawed, Vec3<T>{T(1), T(0), T(0)}, kPi<T> / T(2));
  // After rolling right 90 deg, the right wing (body y) points down.
  EXPECT_TRUE(
      vec_near(rotate(q, Vec3<T>{T(0), T(1), T(0)}), Vec3<T>{T(0), T(0), T(1)}, T(10) * tol<T>()));
  // The nose stays east.
  EXPECT_TRUE(
      vec_near(rotate(q, Vec3<T>{T(1), T(0), T(0)}), Vec3<T>{T(0), T(1), T(0)}, T(10) * tol<T>()));
}

TYPED_TEST(QuatTest, Rk4OnDerivativeMatchesExactIntegration) {
  using T = TypeParam;
  const Vec3<T> omega{T(0.3), T(-0.2), T(0.5)};
  const auto f = [&](T /*t*/, const Quat<T>& q) { return derivative(q, omega); };
  Quat<T> q_rk4 = sample_attitudes<T>()[1];
  Quat<T> q_exact = q_rk4;
  const T dt = T(0.01);
  for (int i = 0; i < 500; ++i) {
    q_rk4 = normalized(rk4_step(f, T(0), q_rk4, dt));
    q_exact = integrate(q_exact, omega, dt);
  }
  EXPECT_TRUE(same_rotation(q_rk4, q_exact, std::is_same_v<T, float> ? T(1e-4) : T(1e-10)));
}

TYPED_TEST(QuatTest, NormStaysUnitOverLongPropagation) {
  using T = TypeParam;
  Quat<T> q = Quat<T>::identity();
  const Vec3<T> omega{T(2), T(-1), T(3)};
  for (int i = 0; i < 100000; ++i) q = integrate(q, omega, T(0.004));
  KESTREL_EXPECT_NEAR(norm(q), T(1), T(10) * tol<T>());
}

}  // namespace
}  // namespace gnc::test
