#include "vz_step_control/step_profile.h"

#include <sstream>
#include <vector>

namespace vz_step_control {
namespace {

template <typename T>
std::string valueText(const T& value) {
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

}  // namespace

void StepParams::validate() const {
  std::vector<std::string> problems;

  const bool finite =
      std::isfinite(step_speed_mps) && std::isfinite(step_duration_sec) &&
      std::isfinite(stabilize_duration_sec) &&
      std::isfinite(hold_duration_sec) && std::isfinite(setpoint_rate_hz) &&
      std::isfinite(requested_telemetry_rate_hz) &&
      std::isfinite(minimum_data_rate_hz) && std::isfinite(rate_window_sec) &&
      std::isfinite(rate_fault_hold_sec) &&
      std::isfinite(maximum_topic_age_sec) &&
      std::isfinite(state_timeout_sec) &&
      std::isfinite(maximum_vertical_offset_m) &&
      std::isfinite(maximum_vertical_span_m) &&
      std::isfinite(maximum_absolute_vz_mps) &&
      std::isfinite(maximum_xy_error_m) &&
      std::isfinite(restore_local_position_rate_hz) &&
      std::isfinite(restore_target_local_rate_hz) &&
      std::isfinite(recorder_health_timeout_sec);
  if (!finite) {
    problems.push_back("all parameters must be finite (got NaN or Inf)");
  }

  if (finite) {
    if (step_speed_mps <= 0.0) {
      problems.push_back("step_speed_mps must be > 0 (got " +
                         valueText(step_speed_mps) + ")");
    }
    if (step_duration_sec <= 0.0) {
      problems.push_back("step_duration_sec must be > 0 (got " +
                         valueText(step_duration_sec) + ")");
    }
    if (stabilize_duration_sec <= 0.0) {
      problems.push_back("stabilize_duration_sec must be > 0 (got " +
                         valueText(stabilize_duration_sec) + ")");
    }
    if (hold_duration_sec <= 0.0) {
      problems.push_back("hold_duration_sec must be > 0 (got " +
                         valueText(hold_duration_sec) + ")");
    }
    if (cycle_count < 1) {
      problems.push_back("cycle_count must be >= 1 (got " +
                         valueText(cycle_count) + ")");
    }
    if (setpoint_rate_hz <= 0.0) {
      problems.push_back("setpoint_rate_hz must be > 0 (got " +
                         valueText(setpoint_rate_hz) + ")");
    }
    if (requested_telemetry_rate_hz <= 0.0 ||
        minimum_data_rate_hz <= 0.0 ||
        requested_telemetry_rate_hz < minimum_data_rate_hz) {
      problems.push_back(
          "requested_telemetry_rate_hz and minimum_data_rate_hz must be > 0 "
          "with requested >= minimum");
    }
    if (rate_window_sec <= 0.0 || rate_fault_hold_sec < 0.0 ||
        maximum_topic_age_sec <= 0.0 || state_timeout_sec <= 0.0) {
      problems.push_back(
          "rate_window_sec/maximum_topic_age_sec/state_timeout_sec must be "
          "> 0 and rate_fault_hold_sec must be >= 0");
    }
    if (recorder_health_timeout_sec <= 0.0) {
      problems.push_back("recorder_health_timeout_sec must be > 0 (got " +
                         valueText(recorder_health_timeout_sec) + ")");
    }
    if (maximum_vertical_offset_m <= 0.0 || maximum_vertical_span_m <= 0.0 ||
        maximum_vertical_span_m >= 1.0 || maximum_xy_error_m <= 0.0) {
      problems.push_back(
          "maximum_vertical_offset_m/maximum_xy_error_m must be > 0 and "
          "maximum_vertical_span_m in (0, 1)");
    }
    if (maximum_absolute_vz_mps <= 0.0 ||
        maximum_absolute_vz_mps <= step_speed_mps) {
      problems.push_back(
          "step_speed_mps=" + valueText(step_speed_mps) +
          " exceeds maximum_absolute_vz_mps=" +
          valueText(maximum_absolute_vz_mps));
    }
    if (stabilize_duration_sec < rate_window_sec) {
      problems.push_back(
          "stabilize_duration_sec=" +
          valueText(stabilize_duration_sec) +
          " must be >= rate_window_sec=" + valueText(rate_window_sec));
    }
    if (displacement() >= maximum_vertical_span_m) {
      problems.push_back(
          "theoretical step displacement step_speed_mps(" +
          valueText(step_speed_mps) + ") * step_duration_sec(" +
          valueText(step_duration_sec) + ")=" +
          valueText(displacement()) +
          " m exceeds maximum_vertical_span_m=" +
          valueText(maximum_vertical_span_m) + " m safety margin");
    }
  }

  if (!problems.empty()) {
    std::string message = "invalid vz_step_control parameters:";
    for (const std::string& problem : problems) {
      message += " " + problem + ";";
    }
    throw std::invalid_argument(message);
  }
}

}  // namespace vz_step_control
