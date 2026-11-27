/**
 * Unit tests for the speed error and heading error computations.
 */

#include <cmath>

#include <gtest/gtest.h>

#include "../control_errors.h"

namespace {

const double kTolerance = 1e-9;
const double kPi = 3.14159265358979323846;

}  // namespace

TEST(WrapAngle, LeavesAnglesInsideHalfTurnUnchanged) {
  EXPECT_NEAR(control_errors::WrapAngle(0.5), 0.5, kTolerance);
  EXPECT_NEAR(control_errors::WrapAngle(-0.5), -0.5, kTolerance);
  EXPECT_NEAR(control_errors::WrapAngle(0.0), 0.0, kTolerance);
}

TEST(WrapAngle, WrapsAnglesBeyondHalfTurn) {
  EXPECT_NEAR(control_errors::WrapAngle(1.5 * kPi), -0.5 * kPi, kTolerance);
  EXPECT_NEAR(control_errors::WrapAngle(-1.5 * kPi), 0.5 * kPi, kTolerance);
  EXPECT_NEAR(control_errors::WrapAngle(4.0 * kPi + 0.25), 0.25, kTolerance);
}

TEST(SpeedError, IsPositiveWhenCarIsTooSlow) {
  EXPECT_NEAR(control_errors::SpeedError(5.0, 2.0), 3.0, kTolerance);
}

TEST(SpeedError, IsNegativeWhenCarIsTooFast) {
  EXPECT_NEAR(control_errors::SpeedError(2.0, 5.0), -3.0, kTolerance);
}

TEST(HeadingError, IsZeroWhenCarPointsAtTarget) {
  // Car at the origin facing +x, target straight ahead.
  EXPECT_NEAR(control_errors::HeadingError(0.0, 0.0, 0.0, 10.0, 0.0), 0.0, kTolerance);
}

TEST(HeadingError, IsPositiveWhenTargetIsCounterClockwiseOfYaw) {
  // Target along +y while the car faces +x: a quarter turn.
  EXPECT_NEAR(control_errors::HeadingError(0.0, 0.0, 0.0, 0.0, 10.0), 0.5 * kPi, kTolerance);
}

TEST(HeadingError, IsNegativeWhenTargetIsClockwiseOfYaw) {
  EXPECT_NEAR(control_errors::HeadingError(0.0, 0.0, 0.0, 0.0, -10.0), -0.5 * kPi, kTolerance);
}

TEST(HeadingError, IsMeasuredFromCarPositionNotOrigin) {
  // Car at (3, 4) facing +y, target directly "above" it.
  EXPECT_NEAR(control_errors::HeadingError(3.0, 4.0, 0.5 * kPi, 3.0, 9.0), 0.0, kTolerance);
}

TEST(HeadingError, TakesShortWayRoundAcrossTheHalfTurnBoundary) {
  // Yaw just below +pi, target direction just above -pi: these are only
  // 0.2 rad apart, not almost a full turn.
  const double yaw = kPi - 0.1;
  const double target_direction = -kPi + 0.1;
  const double target_x = 10.0 * std::cos(target_direction);
  const double target_y = 10.0 * std::sin(target_direction);

  EXPECT_NEAR(control_errors::HeadingError(0.0, 0.0, yaw, target_x, target_y), 0.2, 1e-6);
}

TEST(HeadingError, IsZeroWhenCarIsAlreadyAtTarget) {
  // The direction to a point the car is standing on is undefined, so there
  // is nothing to steer towards.
  EXPECT_NEAR(control_errors::HeadingError(2.0, 2.0, 1.0, 2.0, 2.0), 0.0, kTolerance);
  EXPECT_NEAR(control_errors::HeadingError(2.0, 2.0, 1.0, 2.01, 2.0), 0.0, kTolerance);
}
