#include "vz_step_control/step_controller.h"

#include <cmath>

namespace vz_step_control {

namespace {

bool allFinite(double x, double y, double z, double yaw) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
         std::isfinite(yaw);
}

}  // namespace

const char* phaseText(std::uint8_t phase) {
  switch (phase) {
    case kWaitFcu:
      return "WAIT_FCU";
    case kPrestream:
      return "PRESTREAM";
    case kStabilize:
      return "STABILIZE";
    case kUpStep:
      return "UP_STEP";
    case kHoldUp:
      return "HOLD_UP";
    case kDownStep:
      return "DOWN_STEP";
    case kHoldDown:
      return "HOLD_DOWN";
    case kComplete:
      return "COMPLETE";
    case kAbort:
      return "ABORT";
    default:
      return "UNKNOWN";
  }
}

StepController::StepController(StepParams params)
    : params_(params),
      setpoint_rate_(params.rate_window_sec),
      target_rate_(params.rate_window_sec),
      velocity_rate_(params.rate_window_sec),
      pose_rate_(params.rate_window_sec) {}

void StepController::setState(bool connected, bool armed, bool offboard_mode,
                              double now) {
  have_state_ = true;
  latest_state_time_ = now;
  connected_ = connected;
  const bool active = connected && armed && offboard_mode;
  if (connected && !offboard_mode) {
    // A healthy, explicitly non-OFFBOARD report is the only thing that arms
    // the next OFFBOARD rising edge (it is also how an operator tells the
    // controller a fresh test is wanted).
    have_seen_disengaged_ = true;
    need_healthy_disengage_ = false;
  }
  if (!connected) {
    // A MAVLink drop must not count as a manual exit: it only ends the current
    // session, and a reconnect that lands straight back in OFFBOARD must hold
    // position rather than silently auto-start a fresh step sequence.
    need_healthy_disengage_ = true;
  }
  if (active && !offboard_active_) {
    offboard_active_ = true;
    if (need_healthy_disengage_) {
      abortSession("OFFBOARD resumed after a MAVLink drop without a manual "
                   "exit and re-entry; holding position and refusing a new "
                   "test",
                   now);
    } else {
      beginOffboard(now);
    }
  } else if (!active && offboard_active_) {
    offboard_active_ = false;
    updatePrestream(now);
  } else {
    offboard_active_ = active;
  }
}

void StepController::onRecorderHealth(bool healthy, bool recording,
                                     double now) {
  recorder_healthy_ = healthy;
  recorder_recording_ = recording;
  have_recorder_health_ = true;
  latest_recorder_health_time_ = now;
}

void StepController::onOdom(double x, double y, double z, double yaw,
                            double now) {
  if (!allFinite(x, y, z, yaw)) {
    return;
  }
  odom_x_m_ = x;
  odom_y_m_ = y;
  odom_z_m_ = z;
  odom_yaw_rad_ = yaw;
  have_odom_ = true;
  latest_odom_time_ = now;
  pose_rate_.tick(now);
  if (offboard_active_ && hold_locked_) {
    minimum_z_m_ = std::min(minimum_z_m_, z);
    maximum_z_m_ = std::max(maximum_z_m_, z);
  }
}

void StepController::onVelocityVz(double vz, double now) {
  if (!std::isfinite(vz)) {
    return;
  }
  velocity_vz_mps_ = vz;
  have_velocity_ = true;
  latest_velocity_time_ = now;
  velocity_rate_.tick(now);
}

bool StepController::notePublisherCallerid(const std::string& callerid,
                                           double now) {
  if (callerid.empty() || callerid == self_publisher_name_) {
    return false;
  }
  foreign_callerids_.insert(callerid);
  if (!dual_publisher_fault_) {
    dual_publisher_fault_ = true;
    first_foreign_callerid_ = callerid;
  }
  if (monitoredPhase()) {
    abortSession("duplicate setpoint publisher detected: " + callerid, now);
    return true;
  }
  return false;
}

void StepController::beginOffboard(double now) {
  lockTargetFromOdom();
  if (!have_seen_disengaged_) {
    abortSession(
        "node restarted inside OFFBOARD; exit and re-enter OFFBOARD to arm a "
        "new test",
        now);
    return;
  }
  if (!have_odom_ || now - latest_odom_time_ > params_.maximum_topic_age_sec ||
      !telemetry_requested_) {
    abortSession("entered OFFBOARD before odometry/telemetry preflight was ready",
                 now);
    return;
  }
  if (dual_publisher_fault_) {
    abortSession("duplicate setpoint publisher detected: " +
                     first_foreign_callerid_,
                 now);
    return;
  }
  if (!recorderOkay(now)) {
    abortSession("entered OFFBOARD without a healthy bag recorder; a test "
                 "must never run unrecorded",
                 now);
    return;
  }
  rate_fault_start_time_ = -1.0;
  cycle_index_ = 0;
  setPhase(kStabilize, "OFFBOARD entered; stabilizing", now);
}

void StepController::updatePrestream(double now) {
  if (have_odom_) {
    lockTargetFromOdom();
  }
  const bool base_ready = have_state_ && connected_ && have_odom_ &&
                          now - latest_state_time_ <=
                              params_.state_timeout_sec &&
                          now - latest_odom_time_ <=
                              params_.maximum_topic_age_sec;
  const bool recorder_ready = recorderOkay(now);
  const bool ready = base_ready && recorder_ready;
  std::string reason;
  if (base_ready && !recorder_ready) {
    reason = "ready; waiting for the bag recorder before OFFBOARD";
  } else {
    reason = ready
                 ? "ready; streaming position hold and waiting for OFFBOARD"
                 : "waiting for FCU, odometry and telemetry setup";
  }
  setPhase(ready ? kPrestream : kWaitFcu, reason, now);
}

StepCommand StepController::tick(double now) {
  if (offboard_active_) {
    if (phase_ == kAbort && !hold_locked_ && have_odom_) {
      // Session start failed (e.g. node restart inside OFFBOARD): capture a
      // hold target as soon as odometry exists so setpoints keep flowing.
      lockTargetFromOdom();
    }
    runSession(now);
  } else {
    updatePrestream(now);
  }
  StepCommand command = currentCommand();
  command_vz_mps_ = command.velocity_z_mode ? command.vz : 0.0;
  return command;
}

void StepController::runSession(double now) {
  if (phase_ == kAbort) {
    return;
  }
  if (monitoredPhase() && !safetyOkay(now)) {
    return;
  }
  if (phase_ == kComplete) {
    return;
  }

  if (phase_ == kStabilize) {
    if (now - phase_start_time_ >= params_.stabilize_duration_sec) {
      if (!ratesOkay(now)) {
        abortSession("one or more 100 Hz data streams are below 90 Hz", now);
        return;
      }
      if (!recorderOkay(now)) {
        abortSession("bag recorder stopped during STABILIZE; aborting so the "
                     "step never runs unrecorded",
                     now);
        return;
      }
      cycle_index_ = 1;
      setPhase(kUpStep, "up step 1/" + std::to_string(params_.cycle_count),
               now);
    }
    return;
  }

  if (phase_ == kUpStep || phase_ == kHoldUp || phase_ == kDownStep ||
      phase_ == kHoldDown) {
    if (!ratesOkay(now)) {
      if (rate_fault_start_time_ < 0.0) {
        rate_fault_start_time_ = now;
      } else if (now - rate_fault_start_time_ >=
                 params_.rate_fault_hold_sec) {
        abortSession("100 Hz data rate remained below threshold", now);
        return;
      }
    } else {
      rate_fault_start_time_ = -1.0;
    }
    if (phase_ == kAbort) {
      return;
    }
    if (!recorderOkay(now)) {
      abortSession("bag recorder stopped mid-test; aborting so the step is "
                   "not left unrecorded",
                   now);
      return;
    }
  }

  if (phase_ == kUpStep &&
      now - phase_start_time_ >= params_.step_duration_sec) {
    captureHoldTarget();
    setPhase(kHoldUp, "captured position hold after up step", now);
  } else if (phase_ == kHoldUp &&
             now - phase_start_time_ >= params_.hold_duration_sec) {
    setPhase(kDownStep, "down step " + std::to_string(cycle_index_) + "/" +
                            std::to_string(params_.cycle_count),
             now);
  } else if (phase_ == kDownStep &&
             now - phase_start_time_ >= params_.step_duration_sec) {
    captureHoldTarget();
    setPhase(kHoldDown, "captured position hold after down step", now);
  } else if (phase_ == kHoldDown &&
             now - phase_start_time_ >= params_.hold_duration_sec) {
    if (cycle_index_ < params_.cycle_count) {
      ++cycle_index_;
      setPhase(kUpStep, "up step " + std::to_string(cycle_index_) + "/" +
                            std::to_string(params_.cycle_count),
               now);
    } else {
      setPhase(kComplete,
               "all cycles complete; holding position until OFFBOARD exit",
               now);
    }
  }
}

bool StepController::ratesOkay(double now) {
  return setpoint_rate_.rate(now) >= params_.minimum_data_rate_hz &&
         target_rate_.rate(now) >= params_.minimum_data_rate_hz &&
         velocity_rate_.rate(now) >= params_.minimum_data_rate_hz &&
         setpoint_rate_.age(now) <= params_.maximum_topic_age_sec &&
         target_rate_.age(now) <= params_.maximum_topic_age_sec &&
         velocity_rate_.age(now) <= params_.maximum_topic_age_sec;
}

bool StepController::recorderOkay(double now) const {
  if (!params_.recorder_required) {
    return true;
  }
  if (!have_recorder_health_) {
    return false;
  }
  if (now - latest_recorder_health_time_ >
      params_.recorder_health_timeout_sec) {
    return false;
  }
  if (!recorder_healthy_) {
    return false;
  }
  // Healthy alone only proves the recorder node is alive; while a test is
  // running (STABILIZE through HOLD_DOWN) the bag must actually be open, so
  // "alive but not recording" is treated as not okay.
  const bool test_running = phase_ >= kStabilize && phase_ <= kHoldDown;
  return !test_running || recorder_recording_;
}

bool StepController::safetyOkay(double now) {
  if (!have_odom_ || now - latest_odom_time_ > params_.maximum_topic_age_sec) {
    abortSession("local odometry is stale", now);
    return false;
  }
  if (!have_velocity_ ||
      now - latest_velocity_time_ > params_.maximum_topic_age_sec) {
    abortSession("local velocity is stale", now);
    return false;
  }
  if (have_state_ && now - latest_state_time_ > params_.state_timeout_sec) {
    abortSession("FCU state is stale", now);
    return false;
  }
  if (maximum_z_m_ - minimum_z_m_ > params_.maximum_vertical_span_m) {
    abortSession("vertical peak-to-peak safety limit exceeded", now);
    return false;
  }
  if (start_z_m_ - minimum_z_m_ > params_.maximum_vertical_offset_m) {
    abortSession("vertical undershoot safety limit exceeded", now);
    return false;
  }
  if (std::hypot(odom_x_m_ - start_x_m_, odom_y_m_ - start_y_m_) >
      params_.maximum_xy_error_m) {
    abortSession("horizontal position safety limit exceeded", now);
    return false;
  }
  if (std::abs(velocity_vz_mps_) > params_.maximum_absolute_vz_mps) {
    abortSession("vertical velocity safety limit exceeded", now);
    return false;
  }
  return true;
}

void StepController::lockTargetFromOdom() {
  if (!have_odom_) {
    return;
  }
  hold_x_m_ = odom_x_m_;
  hold_y_m_ = odom_y_m_;
  hold_z_m_ = odom_z_m_;
  hold_yaw_rad_ = odom_yaw_rad_;
  start_x_m_ = odom_x_m_;
  start_y_m_ = odom_y_m_;
  start_z_m_ = odom_z_m_;
  minimum_z_m_ = odom_z_m_;
  maximum_z_m_ = odom_z_m_;
  hold_locked_ = true;
}

void StepController::captureHoldTarget() {
  if (!have_odom_) {
    return;
  }
  hold_x_m_ = odom_x_m_;
  hold_y_m_ = odom_y_m_;
  hold_z_m_ = odom_z_m_;
  hold_yaw_rad_ = odom_yaw_rad_;
  hold_locked_ = true;
}

void StepController::abortSession(const std::string& reason, double now) {
  if (phase_ == kAbort) {
    return;
  }
  if (have_odom_) {
    // Hold the most recent valid position; if the odom stream itself died,
    // this is the newest sample we ever trusted.
    captureHoldTarget();
  }
  setPhase(kAbort, reason, now);
  command_vz_mps_ = 0.0;
}

void StepController::setPhase(std::uint8_t phase, const std::string& reason,
                              double now) {
  if (phase_ == phase && reason_ == reason) {
    return;
  }
  const bool changed = phase_ != phase;
  phase_ = phase;
  reason_ = reason;
  if (changed) {
    phase_start_time_ = now;
    if (phase_change_cb_) {
      phase_change_cb_(phase, reason);
    }
  }
}

StepCommand StepController::currentCommand() const {
  StepCommand command;
  if (!hold_locked_) {
    return command;
  }
  command.publish = true;
  command.x = hold_x_m_;
  command.y = hold_y_m_;
  command.z = hold_z_m_;
  command.yaw = hold_yaw_rad_;
  if (phase_ == kUpStep || phase_ == kDownStep) {
    command.velocity_z_mode = true;
    command.vz = phase_ == kUpStep ? params_.step_speed_mps
                                   : -params_.step_speed_mps;
  } else {
    command.velocity_z_mode = false;
    command.vz = 0.0;
  }
  return command;
}

}  // namespace vz_step_control
