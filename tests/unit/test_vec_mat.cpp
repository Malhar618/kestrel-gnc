#include <gtest/gtest.h>

#include "gnc/math/mat3.hpp"
#include "gnc/math/vec3.hpp"
#include "test_support.hpp"

namespace gnc::test {
namespace {

template <typename T>
class VecMat : public ::testing::Test {};
TYPED_TEST_SUITE(VecMat, Scalars);

TYPED_TEST(VecMat, CrossProductIsRightHanded) {
  using T = TypeParam;
  const Vec3<T> x{T(1), T(0), T(0)}, y{T(0), T(1), T(0)}, z{T(0), T(0), T(1)};
  EXPECT_TRUE(vec_near(cross(x, y), z, tol<T>()));
  EXPECT_TRUE(vec_near(cross(y, z), x, tol<T>()));
  EXPECT_TRUE(vec_near(cross(z, x), y, tol<T>()));
  EXPECT_TRUE(vec_near(cross(y, x), -z, tol<T>()));
}

TYPED_TEST(VecMat, SkewMatrixReproducesCrossProduct) {
  using T = TypeParam;
  const Vec3<T> a{T(0.3), T(-1.2), T(2.0)}, b{T(-0.7), T(0.4), T(1.1)};
  EXPECT_TRUE(vec_near(skew(a) * b, cross(a, b), tol<T>()));
}

TYPED_TEST(VecMat, TransposeReversesProductOrder) {
  using T = TypeParam;
  const Mat3<T> a{{T(1), T(2), T(3), T(0), T(-1), T(4), T(2), T(2), T(-3)}};
  const Mat3<T> b{{T(0.5), T(0), T(1), T(-2), T(1), T(0), T(3), T(-1), T(2)}};
  EXPECT_TRUE(mat_near(transposed(a * b), transposed(b) * transposed(a), tol<T>()));
}

TYPED_TEST(VecMat, DeterminantOfKnownMatrix) {
  using T = TypeParam;
  // 2(3*2 - 2*1) - 0(1*2 - 2*1) + 1(1*1 - 3*1) = 8 - 0 - 2 = 6
  const Mat3<T> a{{T(2), T(0), T(1), T(1), T(3), T(2), T(1), T(1), T(2)}};
  KESTREL_EXPECT_NEAR(determinant(a), T(6), tol<T>());
  KESTREL_EXPECT_NEAR(determinant(Mat3<T>::identity()), T(1), tol<T>());
}

TYPED_TEST(VecMat, InverseTimesMatrixIsIdentity) {
  using T = TypeParam;
  // A full inertia-like matrix, including products of inertia.
  const Mat3<T> a{{T(0.02), T(0.001), T(0), T(0.001), T(0.03), T(0.002), T(0), T(0.002), T(0.05)}};
  // Relative tolerance: entries of inverse(a) are ~50, so scale by the entry size.
  EXPECT_TRUE(mat_near(a * inverse(a), Mat3<T>::identity(), T(100) * tol<T>()));
  EXPECT_TRUE(mat_near(inverse(a) * a, Mat3<T>::identity(), T(100) * tol<T>()));
}

TYPED_TEST(VecMat, InverseOfNonSymmetricKnownMatrix) {
  using T = TypeParam;
  // Non-symmetric, so a missing adjugate transpose would show. det = 6 and
  // adj = {4, 1, -3; 0, 3, -3; -2, -2, 6}, worked out by hand.
  const Mat3<T> a{{T(2), T(0), T(1), T(1), T(3), T(2), T(1), T(1), T(2)}};
  const Mat3<T> expected{{T(4) / T(6), T(1) / T(6), T(-3) / T(6), T(0), T(3) / T(6), T(-3) / T(6),
                          T(-2) / T(6), T(-2) / T(6), T(6) / T(6)}};
  EXPECT_TRUE(mat_near(inverse(a), expected, T(10) * tol<T>()));
  EXPECT_TRUE(mat_near(a * inverse(a), Mat3<T>::identity(), T(10) * tol<T>()));
}

TYPED_TEST(VecMat, DiagBuildsDiagonalMatrix) {
  using T = TypeParam;
  const Mat3<T> d = diag(Vec3<T>{T(1), T(2), T(3)});
  EXPECT_TRUE(vec_near(d * Vec3<T>{T(1), T(1), T(1)}, Vec3<T>{T(1), T(2), T(3)}, tol<T>()));
  KESTREL_EXPECT_NEAR(determinant(d), T(6), tol<T>());
}

}  // namespace
}  // namespace gnc::test
