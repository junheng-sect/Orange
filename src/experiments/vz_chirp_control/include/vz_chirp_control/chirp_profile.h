#pragma once

#include <cstddef>

namespace vz_chirp_control {

class ChirpProfile {
 public:
  ChirpProfile(double amplitude_mps, double start_frequency_hz,
               double end_frequency_hz, double duration_sec,
               double fade_duration_sec, double sample_rate_hz);

  double value(double elapsed_sec) const;
  double instantaneousFrequency(double elapsed_sec) const;
  double theoreticalSpan() const { return theoretical_span_m_; }
  double initialPhase() const { return initial_phase_rad_; }

 private:
  double envelope(double elapsed_sec) const;
  double basePhase(double elapsed_sec) const;
  double spanForPhase(double phase_rad) const;

  double amplitude_mps_;
  double start_frequency_hz_;
  double end_frequency_hz_;
  double duration_sec_;
  double fade_duration_sec_;
  double sample_rate_hz_;
  double logarithmic_rate_;
  double initial_phase_rad_;
  double theoretical_span_m_;
};

}  // namespace vz_chirp_control
