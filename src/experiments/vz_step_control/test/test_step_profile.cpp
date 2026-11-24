#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "vz_step_control/step_profile.h"

namespace {

vz_step_control::StepParams defaultParams() {
  return vz_step_control::StepParams{};
}

void expectInvalid(const vz_step_control::StepParams& params,
                   const std::string& fragment) {
  try {
    params.validate();
    FAIL() << "expected std::invalid_argument containing \"" << fragment
           << "\"";
  } catch (const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find(fragment), std::string::npos)
        << "message \"" << error.what() << "\" lacks \"" << fragment << "\"";
  }
}

TEST(StepParams, DefaultConfigurationIsValidAndMatchesSpec) {
  const vz_step_control::StepParams params = defaultParams();
  params.validate();
  EXPECT_DOUBLE_EQ(params.step_speed_mps, 0.5);
  EXPECT_DOUBLE_EQ(params.step_duration_sec, 1.0);
  EXPECT_DOUBLE_EQ(params.stabilize_duration_sec, 3.0);
  EXPECT_DOUBLE_EQ(params.hold_duration_sec, 10.0);
  EXPECT_EQ(params.cycle_count, 2);
  EXPECT_DOUBLE_EQ(params.setpoint_rate_hz, 100.0);
  EXPECT_DOUBLE_EQ(params.displacement(), 0.5);
  EXPECT_DOUBLE_EQ(params.controlDuration(), 47.0);
}

TEST(StepParams, ExampleConfiguration025x2IsValidAndMatchesSpec) {
  vz_step_control::StepParams params = defaultParams();
  params.step_speed_mps = 0.25;
  params.step_duration_sec = 2.0;
  params.validate();
  EXPECT_DOUBLE_EQ(params.displacement(), 0.5);
  EXPECT_DOUBLE_EQ(params.controlDuration(), 51.0);
}

TEST(StepParams, RejectsZeroSpeed) {
  vz_step_control::StepParams params = defaultParams();
  params.step_speed_mps = 0.0;
  expectInvalid(params, "step_speed_mps");
}

TEST(StepParams, RejectsNegativeSpeed) {
  vz_step_control::StepParams params = defaultParams();
  params.step_speed_mps = -0.5;
  expectInvalid(params, "step_speed_mps");
}

TEST(StepParams, RejectsNaNAndInfiniteSpeed) {
  vz_step_control::StepParams params = defaultParams();
  params.step_speed_mps = std::nan("");
  expectInvalid(params, "finite");
  params = defaultParams();
  params.step_speed_mps = std::numeric_limits<double>::infinity();
  expectInvalid(params, "finite");
}

TEST(StepParams, RejectsZeroNegativeAndNaNDuration) {
  vz_step_control::StepParams params = defaultParams();
  params.step_duration_sec = 0.0;
  expectInvalid(params, "step_duration_sec");
  params = defaultParams();
  params.step_duration_sec = -1.0;
  expectInvalid(params, "step_duration_sec");
  params = defaultParams();
  params.step_duration_sec = std::nan("");
  expectInvalid(params, "finite");
}

TEST(StepParams, RejectsInvalidHoldStabilizeAndCycles) {
  vz_step_control::StepParams params = defaultParams();
  params.hold_duration_sec = 0.0;
  expectInvalid(params, "hold_duration_sec");
  params = defaultParams();
  params.stabilize_duration_sec = 0.0;
  expectInvalid(params, "stabilize_duration_sec");
  params = defaultParams();
  params.cycle_count = 0;
  expectInvalid(params, "cycle_count");
  params = defaultParams();
  params.cycle_count = -3;
  expectInvalid(params, "cycle_count");
}

TEST(StepParams, RejectsSpeedAboveInheritedVzLimit) {
  vz_step_control::StepParams params = defaultParams();
  params.step_speed_mps = 0.8;  // maximum_absolute_vz_mps = 0.6
  expectInvalid(params, "maximum_absolute_vz_mps=0.6");
}

TEST(StepParams, RejectsDisplacementBeyondVerticalSpanMargin) {
  vz_step_control::StepParams params = defaultParams();
  params.step_speed_mps = 0.6;
  params.step_duration_sec = 1.4;  // 0.84 m > 0.80 m span
  expectInvalid(params, "maximum_vertical_span_m=0.8");
  params = defaultParams();
  params.step_speed_mps = 0.8;
  params.step_duration_sec = 1.0;  // exactly at the boundary is refused too
  expectInvalid(params, "maximum_vertical_span_m=0.8");
  params = defaultParams();
  params.step_speed_mps = 0.55;
  params.step_duration_sec = 1.0;  // 0.55 m stays inside the 0.8 m margin
  params.validate();
}

TEST(StepParams, RejectsStabilizeShorterThanRateWindow) {
  vz_step_control::StepParams params = defaultParams();
  params.stabilize_duration_sec = 1.0;  // rate_window_sec = 2.0
  expectInvalid(params, "rate_window_sec=2");
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
