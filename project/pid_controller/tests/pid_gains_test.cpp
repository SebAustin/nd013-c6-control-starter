/**
 * Unit tests for reading the gains file used during tuning runs.
 */

#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "../pid_gains.h"

namespace {

const double kTolerance = 1e-12;

pid_gains::ControllerGains Defaults() {
  pid_gains::ControllerGains defaults;
  defaults.steer = {0.3, 0.0, 0.0};
  defaults.throttle = {0.2, 0.0, 0.0};
  return defaults;
}

pid_gains::ParseResult Parse(const std::string& text) {
  std::istringstream input(text);
  return pid_gains::ParseGains(input, Defaults());
}

}  // namespace

TEST(ParseGains, ReadsBothControllers) {
  const pid_gains::ParseResult result = Parse("steer 0.4 0.001 0.2\nthrottle 0.25 0.002 0.05\n");

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NEAR(result.gains.steer.kp, 0.4, kTolerance);
  EXPECT_NEAR(result.gains.steer.ki, 0.001, kTolerance);
  EXPECT_NEAR(result.gains.steer.kd, 0.2, kTolerance);
  EXPECT_NEAR(result.gains.throttle.kp, 0.25, kTolerance);
  EXPECT_NEAR(result.gains.throttle.ki, 0.002, kTolerance);
  EXPECT_NEAR(result.gains.throttle.kd, 0.05, kTolerance);
}

TEST(ParseGains, IgnoresCommentsAndBlankLines) {
  const pid_gains::ParseResult result =
      Parse("# tuning run 3\n\nsteer 0.4 0 0   # trailing comment\n   \nthrottle 0.25 0 0\n");

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NEAR(result.gains.steer.kp, 0.4, kTolerance);
  EXPECT_NEAR(result.gains.throttle.kp, 0.25, kTolerance);
}

TEST(ParseGains, KeepsDefaultsForControllerThatIsNotListed) {
  const pid_gains::ParseResult result = Parse("throttle 0.5 0.1 0.2\n");

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NEAR(result.gains.steer.kp, 0.3, kTolerance);
  EXPECT_NEAR(result.gains.throttle.kp, 0.5, kTolerance);
}

TEST(ParseGains, EmptyInputGivesDefaults) {
  const pid_gains::ParseResult result = Parse("");

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NEAR(result.gains.steer.kp, 0.3, kTolerance);
  EXPECT_NEAR(result.gains.throttle.kp, 0.2, kTolerance);
}

TEST(ParseGains, AcceptsZeroAndNegativeGains) {
  const pid_gains::ParseResult result = Parse("steer 0 0 0\nthrottle -0.1 0 0\n");

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NEAR(result.gains.steer.kp, 0.0, kTolerance);
  EXPECT_NEAR(result.gains.throttle.kp, -0.1, kTolerance);
}

TEST(ParseGains, RejectsUnknownControllerName) {
  const pid_gains::ParseResult result = Parse("steering 0.4 0 0\n");

  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("line 1"), std::string::npos) << result.error;
}

TEST(ParseGains, RejectsTooFewNumbers) {
  const pid_gains::ParseResult result = Parse("steer 0.4 0\n");

  EXPECT_FALSE(result.ok);
}

TEST(ParseGains, RejectsTooManyNumbers) {
  const pid_gains::ParseResult result = Parse("steer 0.4 0 0 9\n");

  EXPECT_FALSE(result.ok);
}

TEST(ParseGains, RejectsTextThatIsNotANumber) {
  const pid_gains::ParseResult result = Parse("steer 0.4 abc 0\n");

  EXPECT_FALSE(result.ok);
}

TEST(ParseGains, RejectsNonFiniteNumbers) {
  EXPECT_FALSE(Parse("steer nan 0 0\n").ok);
  EXPECT_FALSE(Parse("throttle 0.1 inf 0\n").ok);
}

TEST(ParseGains, RejectsControllerListedTwice) {
  const pid_gains::ParseResult result = Parse("steer 0.4 0 0\nsteer 0.5 0 0\n");

  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("line 2"), std::string::npos) << result.error;
}

TEST(ParseGains, ReportsTheFailingLineNumber) {
  const pid_gains::ParseResult result = Parse("# header\nsteer 0.4 0 0\nthrottle oops\n");

  EXPECT_FALSE(result.ok);
  EXPECT_NE(result.error.find("line 3"), std::string::npos) << result.error;
}

TEST(LoadGains, MissingFileGivesDefaults) {
  const pid_gains::ParseResult result =
      pid_gains::LoadGains("/nonexistent/dir/pid_gains.txt", Defaults());

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_FALSE(result.from_file);
  EXPECT_NEAR(result.gains.steer.kp, 0.3, kTolerance);
}

TEST(FormatGains, PrintsBothControllersInFileFormat) {
  pid_gains::ControllerGains gains;
  gains.steer = {0.4, 0.001, 0.2};
  gains.throttle = {0.25, 0.0, 0.05};

  // The printed form must itself be a valid gains file.
  std::istringstream round_trip(pid_gains::FormatGains(gains));
  const pid_gains::ParseResult result = pid_gains::ParseGains(round_trip, Defaults());

  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_NEAR(result.gains.steer.ki, 0.001, kTolerance);
  EXPECT_NEAR(result.gains.throttle.kd, 0.05, kTolerance);
}
