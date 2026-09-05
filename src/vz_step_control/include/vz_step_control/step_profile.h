#ifndef VZ_STEP_CONTROL_STEP_PROFILE_H
#define VZ_STEP_CONTROL_STEP_PROFILE_H

#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>

namespace vz_step_control {

struct StepParams {
  double step_speed_mps = 0.5;
  double step_duration_sec = 1.0;
  double stabilize_duration_sec = 3.0;
  double hold_duration_sec = 10.0;
  int cycle_count = 2;
  double setpoint_rate_hz = 100.0;

  // MAVLink telemetry and quality gate (inherited from vz_chirp.yaml).
  double requested_telemetry_rate_hz = 100.0;
  double minimum_data_rate_hz = 90.0;
  double rate_window_sec = 2.0;
  double rate_fault_hold_sec = 1.0;
  double maximum_topic_age_sec = 0.10;
  double state_timeout_sec = 2.5;

  // Indoor safety envelope (inherited from vz_chirp.yaml).
  double maximum_vertical_offset_m = 0.45;
  double maximum_vertical_span_m = 0.80;
  double maximum_absolute_vz_mps = 0.60;
  double maximum_xy_error_m = 0.30;

  double restore_local_position_rate_hz = 30.0;
  double restore_target_local_rate_hz = 10.0;

  // OFFBOARD bag-recorder coupling. When recorder_required is true the
  // controller refuses to enter STABILIZE (and refuses OFFBOARD entry) unless
  // the recorder reports healthy, and aborts mid-session if that health is
  // lost: a test must never run unrecorded. recorder_health_timeout_sec is how
  // stale a health report may be before the controller treats the recorder as
  // down.
  bool recorder_required = false;
  double recorder_health_timeout_sec = 1.0;

  double displacement() const { return step_speed_mps * step_duration_sec; }

  // Full control timeline: STABILIZE plus cycle_count rounds of
  // UP_STEP + HOLD_UP + DOWN_STEP + HOLD_DOWN.
  double controlDuration() const {
    return stabilize_duration_sec +
           static_cast<double>(cycle_count) *
               (2.0 * step_duration_sec + 2.0 * hold_duration_sec);
  }

  // Throws std::invalid_argument naming the offending parameter and the
  // inherited safety boundary when the configuration must not fly.
  void validate() const;
};

// Monotonic sliding-window rate tracker (same semantics as vz_chirp_control).
class RateTracker {
 public:
  explicit RateTracker(double window_sec = 2.0) : window_sec_(window_sec) {}

  void setWindow(double window_sec) { window_sec_ = window_sec; }

  void tick(double now_sec) {
    samples_.push_back(now_sec);
    prune(now_sec);
  }

  double rate(double now_sec) const {
    prune(now_sec);
    if (samples_.size() < 2) {
      return 0.0;
    }
    const double span = samples_.back() - samples_.front();
    return span > 0.0 ? (samples_.size() - 1) / span : 0.0;
  }

  double age(double now_sec) const {
    return samples_.empty() ? std::numeric_limits<double>::infinity()
                            : now_sec - samples_.back();
  }

 private:
  void prune(double now_sec) const {
    while (samples_.size() > 2 && now_sec - samples_.front() > window_sec_) {
      samples_.pop_front();
    }
  }

  double window_sec_;
  mutable std::deque<double> samples_;
};

}  // namespace vz_step_control

#endif  // VZ_STEP_CONTROL_STEP_PROFILE_H
