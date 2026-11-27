/**
 * Unit tests for the PID class.
 *
 * These tests only use the public methods of PID and only check the control
 * command it returns, so they need neither CARLA nor the planner. Build and
 * run them with the commands in tests/CMakeLists.txt.
 */

#include <cmath>

#include <gtest/gtest.h>

#include "../pid_controller.h"

namespace {

const double kTolerance = 1e-9;
const double kWideMax = 100.0;
const double kWideMin = -100.0;

// Builds a controller with a known time step already applied.
PID MakePid(double kp, double ki, double kd, double lim_max, double lim_min,
            double time_step) {
  PID pid;
  pid.Init(kp, ki, kd, lim_max, lim_min);
  pid.UpdateDeltaTime(time_step);
  return pid;
}

double Step(PID& pid, double error) {
  pid.UpdateError(error);
  return pid.TotalError();
}

}  // namespace

TEST(PidProportional, CommandIsGainTimesError) {
  PID pid = MakePid(2.0, 0.0, 0.0, kWideMax, kWideMin, 1.0);

  EXPECT_NEAR(Step(pid, 1.5), 3.0, kTolerance);
  EXPECT_NEAR(Step(pid, -0.5), -1.0, kTolerance);
}

TEST(PidIntegral, CommandAccumulatesErrorOverTime) {
  PID pid = MakePid(0.0, 0.5, 0.0, kWideMax, kWideMin, 2.0);

  // Each update adds error * time step = 2.0 to the integral.
  EXPECT_NEAR(Step(pid, 1.0), 1.0, kTolerance);
  EXPECT_NEAR(Step(pid, 1.0), 2.0, kTolerance);
}

TEST(PidDerivative, CommandFollowsRateOfChangeOfError) {
  PID pid = MakePid(0.0, 0.0, 3.0, kWideMax, kWideMin, 0.5);

  Step(pid, 1.0);

  // Error rose by 1.0 over 0.5 s: rate 2.0, times gain 3.0.
  EXPECT_NEAR(Step(pid, 2.0), 6.0, kTolerance);
}

TEST(PidDerivative, FirstUpdateHasNoDerivativeKick) {
  PID pid = MakePid(0.0, 0.0, 3.0, kWideMax, kWideMin, 0.5);

  // There is no earlier error to differentiate against.
  EXPECT_NEAR(Step(pid, 5.0), 0.0, kTolerance);
}

TEST(PidAllTerms, CommandIsSumOfThreeTerms) {
  PID pid = MakePid(1.0, 1.0, 1.0, kWideMax, kWideMin, 1.0);

  Step(pid, 1.0);

  // P = 2, I = 1 + 2 = 3, D = (2 - 1) / 1 = 1.
  EXPECT_NEAR(Step(pid, 2.0), 6.0, kTolerance);
}

TEST(PidSaturation, CommandIsHeldAtUpperLimit) {
  PID pid = MakePid(10.0, 0.0, 0.0, 1.2, -1.2, 1.0);

  EXPECT_NEAR(Step(pid, 5.0), 1.2, kTolerance);
}

TEST(PidSaturation, CommandIsHeldAtLowerLimit) {
  PID pid = MakePid(10.0, 0.0, 0.0, 1.2, -1.2, 1.0);

  EXPECT_NEAR(Step(pid, -5.0), -1.2, kTolerance);
}

TEST(PidWindup, IntegralStopsGrowingDuringSaturation) {
  PID pid = MakePid(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);

  // A long stretch of positive error, e.g. a car pulling away from rest.
  for (int i = 0; i < 10; ++i) {
    EXPECT_NEAR(Step(pid, 1.0), 1.0, kTolerance);
  }

  // One update of opposite error must pull the command off the limit.
  // With windup the integral would be 10 and the command would stay at 1.
  EXPECT_NEAR(Step(pid, -1.0), 0.0, kTolerance);
}

TEST(PidWindup, IntegralStopsGrowingDuringNegativeSaturation) {
  PID pid = MakePid(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);

  for (int i = 0; i < 10; ++i) {
    EXPECT_NEAR(Step(pid, -1.0), -1.0, kTolerance);
  }

  EXPECT_NEAR(Step(pid, 1.0), 0.0, kTolerance);
}

TEST(PidTimeStep, ZeroTimeStepKeepsIntegralAndReusesDerivative) {
  PID pid = MakePid(1.0, 1.0, 1.0, kWideMax, kWideMin, 1.0);
  Step(pid, 1.0);
  Step(pid, 2.0);  // P = 2, I = 3, D = 1.

  pid.UpdateDeltaTime(0.0);
  const double command = Step(pid, 3.0);

  // P = 3, I still 3, D still 1.
  EXPECT_TRUE(std::isfinite(command));
  EXPECT_NEAR(command, 7.0, kTolerance);
}

TEST(PidTimeStep, NegativeTimeStepKeepsIntegralAndReusesDerivative) {
  PID pid = MakePid(1.0, 1.0, 1.0, kWideMax, kWideMin, 1.0);
  Step(pid, 1.0);
  Step(pid, 2.0);

  pid.UpdateDeltaTime(-1.0);
  const double command = Step(pid, 3.0);

  EXPECT_TRUE(std::isfinite(command));
  EXPECT_NEAR(command, 7.0, kTolerance);
}

TEST(PidTimeStep, DerivativeResumesAfterZeroTimeStep) {
  PID pid = MakePid(0.0, 0.0, 1.0, kWideMax, kWideMin, 1.0);
  Step(pid, 1.0);
  pid.UpdateDeltaTime(0.0);
  Step(pid, 2.0);

  pid.UpdateDeltaTime(1.0);

  // The change is measured from the most recent error, 2.0.
  EXPECT_NEAR(Step(pid, 5.0), 3.0, kTolerance);
}

TEST(PidTimeStep, WithoutAnyTimeStepOnlyProportionalTermActs) {
  PID pid;
  pid.Init(2.0, 1.0, 1.0, kWideMax, kWideMin);

  const double command = Step(pid, 1.5);

  EXPECT_TRUE(std::isfinite(command));
  EXPECT_NEAR(command, 3.0, kTolerance);
}

TEST(PidTimeStep, UpdateDeltaTimeReturnsStoredTimeStep) {
  PID pid;
  pid.Init(1.0, 0.0, 0.0, kWideMax, kWideMin);

  EXPECT_NEAR(pid.UpdateDeltaTime(0.25), 0.25, kTolerance);
}

TEST(PidInit, ReinitialisingClearsAccumulatedState) {
  PID pid = MakePid(1.0, 1.0, 1.0, kWideMax, kWideMin, 1.0);
  Step(pid, 4.0);
  Step(pid, 6.0);

  pid.Init(1.0, 1.0, 1.0, kWideMax, kWideMin);
  pid.UpdateDeltaTime(1.0);

  // Behaves like a fresh controller: P = 1, I = 1, D = 0.
  EXPECT_NEAR(Step(pid, 1.0), 2.0, kTolerance);
}

TEST(PidCommand, ReadingCommandTwiceGivesSameValue) {
  PID pid = MakePid(1.0, 1.0, 1.0, kWideMax, kWideMin, 1.0);
  pid.UpdateError(2.0);

  // P = 2, I = 2, D = 0. Reading the command must not change any state.
  EXPECT_NEAR(pid.TotalError(), 4.0, kTolerance);
  EXPECT_NEAR(pid.TotalError(), 4.0, kTolerance);
}

TEST(PidWindup, IntegralStaysFrozenWhileProportionalTermSaturates) {
  PID pid = MakePid(10.0, 1.0, 0.0, 1.0, -1.0, 1.0);

  // The proportional term alone holds the command at the limit.
  for (int i = 0; i < 5; ++i) {
    EXPECT_NEAR(Step(pid, 1.0), 1.0, kTolerance);
  }

  // P = 0.5, I = 0.05. With windup the integral would be 5 and the
  // command would still be at the limit.
  EXPECT_NEAR(Step(pid, 0.05), 0.55, kTolerance);
}

TEST(PidWindup, IntegralFillsUpToTheLimitOnACoarseStep) {
  PID pid = MakePid(0.0, 1.0, 0.0, 1.0, -1.0, 1.0);

  EXPECT_NEAR(Step(pid, 0.6), 0.6, kTolerance);

  // The next full step would overshoot the limit; the integral must take
  // the part of it that reaches the limit rather than none of it.
  EXPECT_NEAR(Step(pid, 0.6), 1.0, kTolerance);
  EXPECT_NEAR(Step(pid, -0.25), 0.75, kTolerance);
}

TEST(PidWindup, ReverseActingControllerCanLeaveSaturation) {
  PID pid = MakePid(0.0, -1.0, 0.0, 1.0, -1.0, 1.0);

  // Negative gain: negative error drives the command to the upper limit.
  for (int i = 0; i < 10; ++i) {
    EXPECT_NEAR(Step(pid, -1.0), 1.0, kTolerance);
  }

  EXPECT_NEAR(Step(pid, 1.0), 0.0, kTolerance);
}

TEST(PidInvalidInput, NonFiniteErrorIsIgnored) {
  PID pid = MakePid(1.0, 1.0, 1.0, kWideMax, kWideMin, 1.0);
  Step(pid, 1.0);
  const double before = Step(pid, 2.0);

  EXPECT_NEAR(Step(pid, std::nan("")), before, kTolerance);
  EXPECT_NEAR(Step(pid, INFINITY), before, kTolerance);

  // The controller carries on from the last valid error, 2.0:
  // P = 2, I = 3 + 2 = 5, D = 0.
  EXPECT_NEAR(Step(pid, 2.0), 7.0, kTolerance);
}

TEST(PidInvalidInput, NonFiniteTimeStepIsTreatedAsUnusable) {
  PID pid = MakePid(1.0, 1.0, 1.0, kWideMax, kWideMin, 1.0);
  Step(pid, 1.0);
  Step(pid, 2.0);  // P = 2, I = 3, D = 1.

  pid.UpdateDeltaTime(std::nan(""));
  const double command = Step(pid, 3.0);

  // Same as a zero time step: P = 3, I still 3, D still 1.
  EXPECT_TRUE(std::isfinite(command));
  EXPECT_NEAR(command, 7.0, kTolerance);
}

TEST(PidInit, SwappedLimitsAreNormalised) {
  PID pid;
  pid.Init(5.0, 0.0, 0.0, -1.0, 1.0);  // Limits passed in the wrong order.
  pid.UpdateDeltaTime(1.0);

  EXPECT_NEAR(Step(pid, 0.1), 0.5, kTolerance);
  EXPECT_NEAR(Step(pid, 1.0), 1.0, kTolerance);
  EXPECT_NEAR(Step(pid, -1.0), -1.0, kTolerance);
}
