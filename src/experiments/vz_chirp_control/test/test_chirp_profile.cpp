#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "vz_chirp_control/chirp_profile.h"

namespace {

TEST(ChirpProfile, DefaultProfileMeetsFrequencyAndDisplacementLimits) {
  vz_chirp_control::ChirpProfile profile(0.2, 0.2, 8.0, 60.0, 2.0, 100.0);
  EXPECT_NEAR(0.2, profile.instantaneousFrequency(0.0), 1e-12);
  EXPECT_NEAR(8.0, profile.instantaneousFrequency(60.0), 1e-10);
  EXPECT_GT(profile.theoreticalSpan(), 0.20);
  EXPECT_LT(profile.theoreticalSpan(), 0.30);
  EXPECT_NEAR(0.0, profile.value(0.0), 1e-12);
  EXPECT_NEAR(0.0, profile.value(60.0), 1e-12);
}

TEST(ChirpProfile, InitialPhaseProducesZeroNetCommandDisplacement) {
  vz_chirp_control::ChirpProfile profile(0.2, 0.2, 8.0, 60.0, 2.0, 100.0);
  constexpr double dt = 0.01;
  double integral = 0.0;
  double minimum = 0.0;
  double maximum = 0.0;
  for (int index = 0; index < 6000; ++index) {
    const double velocity = profile.value(index * dt);
    EXPECT_LE(std::abs(velocity), 0.2 + 1e-12);
    integral += velocity * dt;
    minimum = std::min(minimum, integral);
    maximum = std::max(maximum, integral);
  }
  EXPECT_NEAR(0.0, integral, 1e-10);
  EXPECT_NEAR(profile.theoreticalSpan(), maximum - minimum, 1e-12);
  EXPECT_LT(maximum - minimum, 0.30);
}

TEST(ChirpProfile, RejectsInvalidConfiguration) {
  EXPECT_THROW(vz_chirp_control::ChirpProfile(0.2, 1.0, 0.2, 60.0, 2.0,
                                              100.0),
               std::invalid_argument);
  EXPECT_THROW(vz_chirp_control::ChirpProfile(0.2, 0.2, 8.0, 2.0, 2.0,
                                              100.0),
               std::invalid_argument);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
