#include <gtest/gtest.h>

#include <numbers>

#include "gnc/angles.hpp"
#include "test_support.hpp"

namespace {
constexpr float kPi = std::numbers::pi_v<float>;
}

TEST(WrapPi, LeavesInRangeAnglesAlone) {
  KESTREL_EXPECT_NEAR(gnc::wrap_pi(0.5f), 0.5f, 1e-6f);
  KESTREL_EXPECT_NEAR(gnc::wrap_pi(-0.5f), -0.5f, 1e-6f);
}

TEST(WrapPi, WrapsPastPi) {
  KESTREL_EXPECT_NEAR(gnc::wrap_pi(1.5f * kPi), -0.5f * kPi, 1e-5f);
  KESTREL_EXPECT_NEAR(gnc::wrap_pi(-1.5f * kPi), 0.5f * kPi, 1e-5f);
}

TEST(WrapPi, MapsMinusPiToPi) { KESTREL_EXPECT_NEAR(gnc::wrap_pi(-kPi), kPi, 1e-6f); }

TEST(WrapPi, HandlesManyTurns) {
  KESTREL_EXPECT_NEAR(gnc::wrap_pi(10.0f * kPi + 0.25f), 0.25f, 1e-4f);
}
