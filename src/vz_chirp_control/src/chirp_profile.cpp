#include "vz_chirp_control/chirp_profile.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace vz_chirp_control {
namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

ChirpProfile::ChirpProfile(double amplitude_mps, double start_frequency_hz,
                           double end_frequency_hz, double duration_sec,
                           double fade_duration_sec, double sample_rate_hz)
    : amplitude_mps_(amplitude_mps),
      start_frequency_hz_(start_frequency_hz),
      end_frequency_hz_(end_frequency_hz),
      duration_sec_(duration_sec),
      fade_duration_sec_(fade_duration_sec),
      sample_rate_hz_(sample_rate_hz),
      logarithmic_rate_(std::log(end_frequency_hz / start_frequency_hz) /
                        duration_sec),
      initial_phase_rad_(0.0),
      theoretical_span_m_(0.0) {
  if (!(amplitude_mps_ > 0.0) || !(start_frequency_hz_ > 0.0) ||
      !(end_frequency_hz_ > start_frequency_hz_) || !(duration_sec_ > 0.0) ||
      !(sample_rate_hz_ > 0.0) || fade_duration_sec_ < 0.0 ||
      2.0 * fade_duration_sec_ > duration_sec_) {
    throw std::invalid_argument("invalid chirp profile parameters");
  }

  const double dt = 1.0 / sample_rate_hz_;
  double sine_integral = 0.0;
  double cosine_integral = 0.0;
  const std::size_t sample_count =
      static_cast<std::size_t>(std::ceil(duration_sec_ * sample_rate_hz_));
  for (std::size_t index = 0; index < sample_count; ++index) {
    const double time = std::min(duration_sec_, index * dt);
    const double gain = amplitude_mps_ * envelope(time);
    const double phase = basePhase(time);
    sine_integral += gain * std::sin(phase) * dt;
    cosine_integral += gain * std::cos(phase) * dt;
  }

  const double phase_a = std::atan2(-sine_integral, cosine_integral);
  const double phase_b = phase_a + kPi;
  const double span_a = spanForPhase(phase_a);
  const double span_b = spanForPhase(phase_b);
  if (span_a <= span_b) {
    initial_phase_rad_ = phase_a;
    theoretical_span_m_ = span_a;
  } else {
    initial_phase_rad_ = phase_b;
    theoretical_span_m_ = span_b;
  }
}

double ChirpProfile::envelope(double elapsed_sec) const {
  if (elapsed_sec < 0.0 || elapsed_sec >= duration_sec_) {
    return 0.0;
  }
  if (fade_duration_sec_ <= 0.0) {
    return 1.0;
  }
  if (elapsed_sec < fade_duration_sec_) {
    return 0.5 *
           (1.0 - std::cos(kPi * elapsed_sec / fade_duration_sec_));
  }
  if (elapsed_sec > duration_sec_ - fade_duration_sec_) {
    return 0.5 *
           (1.0 - std::cos(kPi * (duration_sec_ - elapsed_sec) /
                           fade_duration_sec_));
  }
  return 1.0;
}

double ChirpProfile::basePhase(double elapsed_sec) const {
  const double time = std::max(0.0, std::min(duration_sec_, elapsed_sec));
  return 2.0 * kPi * start_frequency_hz_ * std::expm1(logarithmic_rate_ * time) /
         logarithmic_rate_;
}

double ChirpProfile::value(double elapsed_sec) const {
  if (elapsed_sec < 0.0 || elapsed_sec >= duration_sec_) {
    return 0.0;
  }
  return amplitude_mps_ * envelope(elapsed_sec) *
         std::sin(basePhase(elapsed_sec) + initial_phase_rad_);
}

double ChirpProfile::instantaneousFrequency(double elapsed_sec) const {
  const double time = std::max(0.0, std::min(duration_sec_, elapsed_sec));
  return start_frequency_hz_ * std::exp(logarithmic_rate_ * time);
}

double ChirpProfile::spanForPhase(double phase_rad) const {
  const double dt = 1.0 / sample_rate_hz_;
  double position = 0.0;
  double minimum = 0.0;
  double maximum = 0.0;
  const std::size_t sample_count =
      static_cast<std::size_t>(std::ceil(duration_sec_ * sample_rate_hz_));
  for (std::size_t index = 0; index < sample_count; ++index) {
    const double time = std::min(duration_sec_, index * dt);
    position += amplitude_mps_ * envelope(time) *
                std::sin(basePhase(time) + phase_rad) * dt;
    minimum = std::min(minimum, position);
    maximum = std::max(maximum, position);
  }
  return maximum - minimum;
}

}  // namespace vz_chirp_control
