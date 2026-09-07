#ifndef VZ_STEP_CONTROL_STEP_CONTROLLER_H
#define VZ_STEP_CONTROL_STEP_CONTROLLER_H

#include <cstdint>
#include <functional>
#include <set>
#include <string>

#include "vz_step_control/step_profile.h"

namespace vz_step_control {

// Phase values mirror msg/VzStepStatus.msg; kept as plain constants so the
// controller stays usable in ROS-free unit tests.
enum StepPhase : std::uint8_t {
  kWaitFcu = 0,
  kPrestream = 1,
  kStabilize = 2,
  kUpStep = 3,
  kHoldUp = 4,
  kDownStep = 5,
  kHoldDown = 6,
  kComplete = 7,
  kAbort = 8,
};

const char* phaseText(std::uint8_t phase);

// One control-period setpoint decision. In velocity_z_mode the caller must
// send the latched XY position plus the raw Z velocity step (PX4 keeps a NaN
// Z position for that period). Otherwise the caller must send the full
// position target with a finite Z so the FCU trajectory setpoint stays
// well-defined.
struct StepCommand {
  bool publish = false;
  bool velocity_z_mode = false;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double vz = 0.0;
  double yaw = 0.0;
};

// ROS-free OFFBOARD step state machine. All time inputs use one monotonic
// seconds source supplied by the caller; state transitions therefore always
// land on control-period boundaries.
class StepController {
 public:
  explicit StepController(StepParams params);

  void setSelfPublisherName(const std::string& name) {
    self_publisher_name_ = name;
  }

  // Called on every phase change with (phase, reason) for operator logging.
  void setPhaseChangeCallback(
      std::function<void(std::uint8_t, const std::string&)> callback) {
    phase_change_cb_ = std::move(callback);
  }

  // MAVROS state proxy: connected && armed && mode == OFFBOARD defines an
  // active session; the rising edge starts one, the falling edge ends it.
  void setState(bool connected, bool armed, bool offboard_mode, double now);
  void setTelemetryRequested(bool requested) {
    telemetry_requested_ = requested;
  }
  // Feed each recorder health report (published at ~2 Hz while the recorder is
  // alive). When params_.recorder_required is true, OFFBOARD entry and the
  // STABILIZE->UP_STEP handoff are refused and a running test is aborted
  // whenever this health goes stale or false, or the recorder is healthy but
  // not actually recording (a test never runs unrecorded).
  void onRecorderHealth(bool healthy, bool recording, double now);
  void onOdom(double x, double y, double z, double yaw, double now);
  void onVelocityVz(double vz, double now);
  void noteTargetMessage(double now) { target_rate_.tick(now); }
  void noteSetpointPublished(double now) { setpoint_rate_.tick(now); }
  void notePoseMessage(double now) { pose_rate_.tick(now); }

  // Feed every callerid seen on the setpoint topic. Returns true when the
  // sample belongs to a foreign publisher; aborts during a session and
  // poisons OFFBOARD entry otherwise.
  bool notePublisherCallerid(const std::string& callerid, double now);

  // Called once per control period; returns the setpoint decision.
  StepCommand tick(double now);

  std::uint8_t phase() const { return phase_; }
  const std::string& reason() const { return reason_; }
  bool offboardActive() const { return offboard_active_; }
  int cycleIndex() const { return cycle_index_; }
  double commandVz() const { return command_vz_mps_; }
  double phaseElapsed(double now) const { return now - phase_start_time_; }

  double latchedX() const { return hold_x_m_; }
  double latchedY() const { return hold_y_m_; }
  double latchedZ() const { return hold_z_m_; }
  double latchedYaw() const { return hold_yaw_rad_; }
  double startZ() const { return start_z_m_; }
  double minZ() const { return minimum_z_m_; }
  double maxZ() const { return maximum_z_m_; }

  const RateTracker& setpointRate() const { return setpoint_rate_; }
  const RateTracker& targetRate() const { return target_rate_; }
  const RateTracker& velocityRate() const { return velocity_rate_; }
  const RateTracker& poseRate() const { return pose_rate_; }

 private:
  void beginOffboard(double now);
  void updatePrestream(double now);
  void runSession(double now);
  bool safetyOkay(double now);
  bool ratesOkay(double now);
  bool recorderOkay(double now) const;
  void lockTargetFromOdom();
  void captureHoldTarget();
  void abortSession(const std::string& reason, double now);
  void setPhase(std::uint8_t phase, const std::string& reason, double now);
  StepCommand currentCommand() const;
  bool monitoredPhase() const {
    return phase_ >= kStabilize && phase_ <= kComplete;
  }

  StepParams params_;
  std::string self_publisher_name_;

  bool have_state_{false};
  bool connected_{false};
  bool have_odom_{false};
  bool have_velocity_{false};
  bool telemetry_requested_{false};
  bool offboard_active_{false};
  bool hold_locked_{false};
  bool dual_publisher_fault_{false};
  // False until a healthy (connected) non-OFFBOARD state was observed, so a
  // node that (re)starts inside an active OFFBOARD never auto-arms a test.
  bool have_seen_disengaged_{false};
  // True from the moment the MAVLink link drops until a healthy non-OFFBOARD
  // state is observed again. A reconnect that lands straight back in OFFBOARD
  // therefore holds position instead of auto-starting a fresh step sequence.
  bool need_healthy_disengage_{false};
  bool have_recorder_health_{false};
  bool recorder_healthy_{false};
  bool recorder_recording_{false};
  double latest_recorder_health_time_{-1.0};

  double latest_state_time_{0.0};
  double latest_odom_time_{0.0};
  double latest_velocity_time_{0.0};
  double odom_x_m_{0.0};
  double odom_y_m_{0.0};
  double odom_z_m_{0.0};
  double odom_yaw_rad_{0.0};
  double velocity_vz_mps_{0.0};

  RateTracker setpoint_rate_;
  RateTracker target_rate_;
  RateTracker velocity_rate_;
  RateTracker pose_rate_;

  double hold_x_m_{0.0};
  double hold_y_m_{0.0};
  double hold_z_m_{0.0};
  double hold_yaw_rad_{0.0};
  // Session-scoped safety reference: the OFFBOARD entry point. Unlike the
  // latched hold target it never moves, so cumulative XY drift stays bounded.
  double start_x_m_{0.0};
  double start_y_m_{0.0};
  double start_z_m_{0.0};
  double minimum_z_m_{0.0};
  double maximum_z_m_{0.0};

  std::uint8_t phase_{kWaitFcu};
  std::string reason_{"waiting for FCU"};
  std::function<void(std::uint8_t, const std::string&)> phase_change_cb_;
  double phase_start_time_{0.0};
  double rate_fault_start_time_{-1.0};
  int cycle_index_{0};
  double command_vz_mps_{0.0};
  std::string first_foreign_callerid_;
  std::set<std::string> foreign_callerids_;
};

}  // namespace vz_step_control

#endif  // VZ_STEP_CONTROL_STEP_CONTROLLER_H
