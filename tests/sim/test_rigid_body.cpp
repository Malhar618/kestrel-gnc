#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <numbers>

#include "gnc/math/mat3.hpp"
#include "gnc/math/quat.hpp"
#include "sim/rigid_body.hpp"
#include "test_support.hpp"

namespace sim::test {
namespace {

using gnc::Mat3d;
using gnc::Vec3d;
using gnc::test::vec_near;

const Vec3d kZero{};
constexpr double kDt = 1e-3;

TEST(RigidBody, TorqueFreeConservesEnergyAndAngularMomentum) {
  // Full inertia tensor (with products of inertia), tumbling about all three axes.
  const RigidBodyParams p =
      make_rigid_body(1.0, Mat3d{{0.020, 0.001, 0.000, 0.001, 0.030, 0.002, 0.000, 0.002, 0.050}});
  RigidBodyState x;
  x.omega_b_radps = {0.3, 2.0, -0.4};
  const double energy0 = rotational_kinetic_energy(p, x);
  const Vec3d h0 = angular_momentum_ned(p, x);
  double max_norm_error = 0.0;
  for (int i = 0; i < 60000; ++i) {  // 60 s
    x = rigid_body_step(p, x, kZero, kZero, kZero, kDt);
    max_norm_error = std::max(max_norm_error, std::abs(gnc::norm(x.q_nb) - 1.0));
  }
  EXPECT_LT(std::abs(rotational_kinetic_energy(p, x) - energy0) / energy0, 1e-6);
  EXPECT_LT(gnc::norm(angular_momentum_ned(p, x) - h0) / gnc::norm(h0), 1e-6);
  EXPECT_LT(max_norm_error, 1e-9);
}

TEST(RigidBody, StepReturnsUnitQuaternion) {
  // One coarse RK4 step at 50 rad/s drifts ~1.7e-6 off unit norm unless q_nb is renormalized.
  const RigidBodyParams p = make_rigid_body(1.0, Mat3d::identity());
  RigidBodyState x;
  x.omega_b_radps = {0.0, 0.0, 50.0};
  x = rigid_body_step(p, x, kZero, kZero, kZero, 0.01);
  EXPECT_NEAR(gnc::norm(x.q_nb), 1.0, 1e-12);
}

TEST(RigidBody, SymmetricBodyWobblesAtTheEulerRate) {
  // Axisymmetric body (I1 = I2): the wobble (omega_x, omega_y) rotates in body axes
  // at lambda = (I3 - I1) / I1 * spin. A sign error in omega x J omega reverses it.
  const double i1 = 0.020, i3 = 0.035, spin = 5.0, wobble = 0.2, t_end = 2.0;
  const RigidBodyParams p = make_rigid_body(1.0, gnc::diag(Vec3d{i1, i1, i3}));
  RigidBodyState x;
  x.omega_b_radps = {wobble, 0.0, spin};
  for (int i = 0; i < 2000; ++i) x = rigid_body_step(p, x, kZero, kZero, kZero, kDt);
  const double lambda = (i3 - i1) / i1 * spin;
  EXPECT_NEAR(x.omega_b_radps.x, wobble * std::cos(lambda * t_end), 1e-9);
  EXPECT_NEAR(x.omega_b_radps.y, wobble * std::sin(lambda * t_end), 1e-9);
  EXPECT_NEAR(x.omega_b_radps.z, spin, 1e-12);
}

TEST(RigidBody, IntermediateAxisSpinFlipsMajorAxisSpinDoesNot) {
  // The tennis-racket (Dzhanibekov) effect: spin about the intermediate axis is
  // unstable and periodically flips; spin about the major axis is stable.
  const RigidBodyParams p = make_rigid_body(1.0, gnc::diag(Vec3d{1.0, 2.0, 3.0}));
  RigidBodyState intermediate, major;
  intermediate.omega_b_radps = {1e-3, 1.0, 1e-3};
  major.omega_b_radps = {1e-3, 1e-3, 1.0};
  double min_intermediate = 1.0, min_major = 1.0;
  for (int i = 0; i < 60000; ++i) {
    intermediate = rigid_body_step(p, intermediate, kZero, kZero, kZero, kDt);
    major = rigid_body_step(p, major, kZero, kZero, kZero, kDt);
    min_intermediate = std::min(min_intermediate, intermediate.omega_b_radps.y);
    min_major = std::min(min_major, major.omega_b_radps.z);
  }
  EXPECT_LT(min_intermediate, -0.9);  // it flipped over
  EXPECT_GT(min_major, 0.999);        // it stayed put
}

TEST(RigidBody, FreeFallMatchesClosedForm) {
  const RigidBodyParams p = make_rigid_body(2.0, Mat3d::identity());
  const Vec3d g{0.0, 0.0, 9.80665};
  RigidBodyState x;
  x.pos_ned_m = {0.0, 0.0, -100.0};
  for (int i = 0; i < 3000; ++i) x = rigid_body_step(p, x, kZero, kZero, g, kDt);
  // RK4 is exact for this quadratic; only rounding remains.
  EXPECT_NEAR(x.pos_ned_m.z, -100.0 + 0.5 * 9.80665 * 3.0 * 3.0, 1e-9);
  EXPECT_NEAR(x.vel_ned_mps.z, 9.80665 * 3.0, 1e-10);
}

TEST(RigidBody, BodyForceIsRotatedIntoNed) {
  // Yawed 90 deg right, a forward (body x) force must accelerate the vehicle east.
  const RigidBodyParams p = make_rigid_body(2.0, Mat3d::identity());
  RigidBodyState x;
  x.q_nb = gnc::from_axis_angle(Vec3d{0.0, 0.0, 1.0}, std::numbers::pi / 2.0);
  const RigidBodyState dx = rigid_body_derivative(p, x, Vec3d{4.0, 0.0, 0.0}, kZero, kZero);
  EXPECT_TRUE(vec_near(dx.vel_ned_mps, Vec3d{0.0, 2.0, 0.0}, 1e-12));
}

TEST(RigidBody, TorqueAboutPrincipalAxisGivesAlphaEqualsTauOverI) {
  const RigidBodyParams p = make_rigid_body(1.0, gnc::diag(Vec3d{0.02, 0.03, 0.05}));
  const RigidBodyState dx =
      rigid_body_derivative(p, RigidBodyState{}, kZero, Vec3d{0.1, -0.3, 0.5}, kZero);
  EXPECT_TRUE(vec_near(dx.omega_b_radps, Vec3d{5.0, -10.0, 10.0}, 1e-12));
}

}  // namespace
}  // namespace sim::test
