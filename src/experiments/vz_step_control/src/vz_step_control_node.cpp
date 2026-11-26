#include <cmath>
#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <boost/asio/io_service.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/MessageInterval.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <vz_step_control/VzRecorderHealth.h>
#include <vz_step_control/VzStepStatus.h>
#include "vz_step_control/step_controller.h"

namespace vz_step_control {
namespace {

constexpr std::uint32_t kLocalPositionNedMessageId = 32;
constexpr std::uint32_t kPositionTargetLocalNedMessageId = 85;

double steadyNowSec() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

double quaternionYaw(const geometry_msgs::Quaternion& quaternion) {
  const double sin_yaw =
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw =
      1.0 - 2.0 * (quaternion.y * quaternion.y +
                   quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

std::string timelineText(const StepParams& params) {
  std::ostringstream stream;
  stream << "VZ step plan: STABILIZE " << params.stabilize_duration_sec << " s";
  for (int cycle = 1; cycle <= params.cycle_count; ++cycle) {
    stream << " -> UP_STEP " << cycle << "/" << params.cycle_count << " +"
           << params.step_speed_mps << " m/s for " << params.step_duration_sec
           << " s (" << params.displacement() << " m)"
           << " -> HOLD_UP " << params.hold_duration_sec
           << " s -> DOWN_STEP " << cycle << "/" << params.cycle_count << " -"
           << params.step_speed_mps << " m/s for " << params.step_duration_sec
           << " s -> HOLD_DOWN " << params.hold_duration_sec << " s";
  }
  stream << " -> COMPLETE; total " << params.controlDuration()
         << " s; theoretical displacement per step " << params.displacement()
         << " m (upward budget maximum_vertical_span_m="
         << params.maximum_vertical_span_m << " m)";
  return stream.str();
}

}  // namespace

class VzStepControlNode {
 public:
  VzStepControlNode() : private_node_("~") {
    StepParams params = readParameters();
    params.validate();  // throws with parameter names and safety boundaries
    params_ = params;

    controller_.reset(new StepController(params_));
    controller_->setSelfPublisherName(ros::this_node::getName());
    controller_->setPhaseChangeCallback([this](std::uint8_t phase,
                                               const std::string& reason) {
      if (phase == kAbort) {
        ROS_ERROR_STREAM("VZ step aborted: " << reason);
      } else {
        ROS_INFO_STREAM("VZ step state -> " << phaseText(phase) << ": "
                                            << reason);
      }
      // A completed or aborted session keeps holding inside OFFBOARD. Hand the
      // 100 Hz MAVLink streams back to their normal rates after the 1 s
      // recorder tail drains, instead of leaving them boosted for the whole
      // hold. The next OFFBOARD entry re-requests 100 Hz.
      if (phase == kComplete || phase == kAbort) {
        std::lock_guard<std::mutex> lock(streams_mutex_);
        if (local_stream_.requested || target_stream_.requested) {
          restore_after_end_at_ = steadyNowSec() + telemetry_tail_sec_;
        }
      }
    });

    ROS_WARN_STREAM(timelineText(params_));

    status_publisher_ = node_.advertise<VzStepStatus>(status_topic_, 10, true);
    if (enable_setpoint_output_) {
      setpoint_publisher_ =
          node_.advertise<mavros_msgs::PositionTarget>(setpoint_topic_, 20);
    } else {
      ROS_WARN(
          "PREVIEW ONLY: /mavros/setpoint_raw/local publisher is disabled");
    }

    state_subscriber_ = node_.subscribe(state_topic_, 20,
                                        &VzStepControlNode::stateCallback, this);
    odom_subscriber_ = node_.subscribe(odom_topic_, 100,
                                       &VzStepControlNode::odomCallback, this,
                                       ros::TransportHints().tcpNoDelay());
    velocity_subscriber_ = node_.subscribe(
        velocity_topic_, 200, &VzStepControlNode::velocityCallback, this,
        ros::TransportHints().tcpNoDelay());
    target_subscriber_ = node_.subscribe(
        target_topic_, 200, &VzStepControlNode::targetCallback, this,
        ros::TransportHints().tcpNoDelay());
    pose_subscriber_ = node_.subscribe(pose_topic_, 200,
                                       &VzStepControlNode::poseCallback, this,
                                       ros::TransportHints().tcpNoDelay());
    // Loopback monitor: any callerid other than this node on the setpoint
    // topic means a competing OFFBOARD publisher.
    setpoint_monitor_subscriber_ = node_.subscribe(
        setpoint_topic_, 10, &VzStepControlNode::setpointMonitorCallback, this);

    if (recorder_required_) {
      recorder_health_subscriber_ = node_.subscribe(
          recorder_topic_, 5, &VzStepControlNode::recorderHealthCallback,
          this);
    }

    message_interval_client_ =
        node_.serviceClient<mavros_msgs::MessageInterval>(message_interval_service_);
    service_thread_ = std::thread([this]() { service_io_.run(); });
    control_timer_ = node_.createWallTimer(
        ros::WallDuration(1.0 / params_.setpoint_rate_hz),
        &VzStepControlNode::controlTimer, this);
    status_timer_ = node_.createWallTimer(
        ros::WallDuration(0.1), &VzStepControlNode::statusTimer, this);
    telemetry_timer_ = node_.createWallTimer(
        ros::WallDuration(1.0), &VzStepControlNode::telemetryTimer, this);

    ROS_WARN("VZ step ready: speed=%.2f m/s duration=%.2f s cycles=%d "
             "publish=%.1f Hz, safety vz<=%.2f m/s span<=%.2f m "
             "undershoot<=%.2f m",
             params_.step_speed_mps, params_.step_duration_sec,
             params_.cycle_count, params_.setpoint_rate_hz,
             params_.maximum_absolute_vz_mps, params_.maximum_vertical_span_m,
             params_.maximum_vertical_offset_m);
  }

  ~VzStepControlNode() {
    restoreTelemetryRates();
    service_work_.reset();
    if (service_thread_.joinable()) {
      service_thread_.join();
    }
  }

 private:
  StepParams readParameters() {
    StepParams params;
    private_node_.param("step_speed_mps", params.step_speed_mps, 0.5);
    private_node_.param("step_duration_sec", params.step_duration_sec, 1.0);
    private_node_.param("stabilize_duration_sec",
                        params.stabilize_duration_sec, 3.0);
    private_node_.param("hold_duration_sec", params.hold_duration_sec, 10.0);
    private_node_.param("cycle_count", params.cycle_count, 2);
    private_node_.param("setpoint_rate_hz", params.setpoint_rate_hz, 100.0);
    private_node_.param("requested_telemetry_rate_hz",
                        params.requested_telemetry_rate_hz, 100.0);
    private_node_.param("minimum_data_rate_hz", params.minimum_data_rate_hz,
                        90.0);
    private_node_.param("rate_window_sec", params.rate_window_sec, 2.0);
    private_node_.param("rate_fault_hold_sec", params.rate_fault_hold_sec, 1.0);
    private_node_.param("maximum_topic_age_sec", params.maximum_topic_age_sec,
                        0.1);
    private_node_.param("state_timeout_sec", params.state_timeout_sec, 2.5);
    private_node_.param("maximum_vertical_offset_m",
                        params.maximum_vertical_offset_m, 0.45);
    private_node_.param("maximum_vertical_span_m",
                        params.maximum_vertical_span_m, 0.8);
    private_node_.param("maximum_absolute_vz_mps",
                        params.maximum_absolute_vz_mps, 0.6);
    private_node_.param("maximum_xy_error_m", params.maximum_xy_error_m, 0.3);
    private_node_.param("restore_local_position_rate_hz",
                        params.restore_local_position_rate_hz, 30.0);
    private_node_.param("restore_target_local_rate_hz",
                        params.restore_target_local_rate_hz, 10.0);
    private_node_.param("recorder_required", recorder_required_, false);
    params.recorder_required = recorder_required_;
    private_node_.param("recorder_health_timeout_sec",
                        params.recorder_health_timeout_sec, 1.0);
    private_node_.param("telemetry_tail_sec", telemetry_tail_sec_, 1.0);

    private_node_.param("enable_setpoint_output", enable_setpoint_output_, true);
    private_node_.param<std::string>("state_topic", state_topic_,
                                     "/mavros/state");
    private_node_.param<std::string>("odom_topic", odom_topic_,
                                     "/mavros/local_position/odom");
    private_node_.param<std::string>(
        "velocity_topic", velocity_topic_,
        "/mavros/local_position/velocity_local");
    private_node_.param<std::string>("pose_topic", pose_topic_,
                                     "/mavros/local_position/pose");
    private_node_.param<std::string>(
        "target_topic", target_topic_, "/mavros/setpoint_raw/target_local");
    private_node_.param<std::string>(
        "setpoint_topic", setpoint_topic_, "/mavros/setpoint_raw/local");
    private_node_.param<std::string>("status_topic", status_topic_,
                                     "/vz_step/status");
    private_node_.param<std::string>(
        "recorder_health_topic", recorder_topic_, "/vz_step/recorder_health");
    private_node_.param<std::string>(
        "message_interval_service", message_interval_service_,
        "/mavros/set_message_interval");
    return params;
  }

  void stateCallback(const mavros_msgs::State::ConstPtr& message) {
    const double now = steadyNowSec();
    const bool active = message->connected && message->armed &&
                        message->mode == "OFFBOARD";
    if (active && !controller_->offboardActive()) {
      // OFFBOARD entry: raise the MAVLink streams to 100 Hz before the
      // controller's preflight runs. Any restore scheduled by a previous
      // finished session is cancelled: this is a new session.  These calls run
      // on the state callback thread, not the 100 Hz control timer, and each
      // stream is tracked independently so a partial failure is retried.
      restore_after_end_at_ = -1.0;
      requestStream(kLocalPositionNedMessageId,
                    params_.requested_telemetry_rate_hz);
      requestStream(kPositionTargetLocalNedMessageId,
                    params_.requested_telemetry_rate_hz);
      controller_->setTelemetryRequested(streamsRequested());
    } else if (!active && controller_->offboardActive()) {
      // Session end: hand the FCU streams back to their normal rates.
      restore_after_end_at_ = -1.0;
      restoreTelemetryRates();
    }
    controller_->setState(message->connected, message->armed,
                          message->mode == "OFFBOARD", now);
    latest_state_ = *message;
    have_state_ = true;
  }

  void recorderHealthCallback(const VzRecorderHealth::ConstPtr& message) {
    controller_->onRecorderHealth(message->healthy, message->recording,
                                  steadyNowSec());
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    latest_odom_ = *message;
    have_odom_ = true;
    controller_->onOdom(message->pose.pose.position.x,
                        message->pose.pose.position.y,
                        message->pose.pose.position.z,
                        quaternionYaw(message->pose.pose.orientation),
                        steadyNowSec());
  }

  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& message) {
    latest_vz_ = message->twist.linear.z;
    have_velocity_ = true;
    controller_->onVelocityVz(message->twist.linear.z, steadyNowSec());
  }

  void targetCallback(const mavros_msgs::PositionTarget::ConstPtr&) {
    controller_->noteTargetMessage(steadyNowSec());
  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr&) {
    controller_->notePoseMessage(steadyNowSec());
  }

  void setpointMonitorCallback(
      const ros::MessageEvent<const mavros_msgs::PositionTarget>& event) {
    const std::string& publisher = event.getPublisherName();
    if (publisher.empty()) {
      return;
    }
    if (controller_->notePublisherCallerid(publisher, steadyNowSec())) {
      ROS_ERROR_STREAM("vz_step aborted: duplicate setpoint publisher "
                       << publisher);
    } else if (publisher != ros::this_node::getName()) {
      ROS_WARN_THROTTLE(
          2.0, "foreign setpoint publisher present (%s); OFFBOARD entry will "
               "be refused",
          publisher.c_str());
    }
  }

  void controlTimer(const ros::WallTimerEvent&) {
    const double now = steadyNowSec();
    const StepCommand command = controller_->tick(now);
    if (!command.publish || !have_state_ || !latest_state_.connected ||
        !have_odom_) {
      return;
    }
    // Tick the generation-rate gate even in preview mode so the controller
    // measures the command pipeline, not the physical publisher.
    controller_->noteSetpointPublished(now);

    if (!enable_setpoint_output_) {
      return;
    }
    mavros_msgs::PositionTarget message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = latest_odom_.header.frame_id;
    message.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    if (command.velocity_z_mode) {
      // Hard Z step: latched XY and yaw, velocity-only Z, no ramp/filter. All
      // three acceleration axes are ignored (incl. AFZ) so the FCU never sees
      // a finite zero Z acceleration masquerading as a valid force estimate.
      message.type_mask = mavros_msgs::PositionTarget::IGNORE_PZ |
                          mavros_msgs::PositionTarget::IGNORE_VX |
                          mavros_msgs::PositionTarget::IGNORE_VY |
                          mavros_msgs::PositionTarget::IGNORE_AFX |
                          mavros_msgs::PositionTarget::IGNORE_AFY |
                          mavros_msgs::PositionTarget::IGNORE_AFZ |
                          mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    } else {
      // Full position target so the FCU keeps a finite Z setpoint.
      message.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                          mavros_msgs::PositionTarget::IGNORE_VY |
                          mavros_msgs::PositionTarget::IGNORE_VZ |
                          mavros_msgs::PositionTarget::IGNORE_AFX |
                          mavros_msgs::PositionTarget::IGNORE_AFY |
                          mavros_msgs::PositionTarget::IGNORE_AFZ |
                          mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    }
    message.position.x = command.x;
    message.position.y = command.y;
    message.position.z = command.z;
    message.velocity.z = command.vz;
    message.yaw = static_cast<float>(command.yaw);
    setpoint_publisher_.publish(message);
  }

  // Per-stream rate state: message 32 (LOCAL_POSITION_NED) and 85
  // (POSITION_TARGET_LOCAL_NED) succeed or fail independently. Tracking them
  // separately lets a retry or restore recover only the stream(s) that are
  // still misconfigured instead of demanding both calls succeed together.
  struct StreamState {
    bool requested{false};   // currently set to the 100 Hz test rate
    bool restored{true};     // back at the normal (non-test) rate
  };

  bool streamsRequested() {
    std::lock_guard<std::mutex> lock(streams_mutex_);
    return local_stream_.requested && target_stream_.requested;
  }

  void requestStream(std::uint32_t message_id, double rate_hz) {
    if (setMessageRate(message_id, rate_hz)) {
      std::lock_guard<std::mutex> lock(streams_mutex_);
      StreamState& stream = streamFor(message_id);
      stream.requested = true;
      stream.restored = false;
      ROS_INFO("requested MAVLink message %u at %.1f Hz", message_id, rate_hz);
    }
  }

  void restoreStream(std::uint32_t message_id, double rate_hz) {
    {
      std::lock_guard<std::mutex> lock(streams_mutex_);
      const StreamState& stream = streamFor(message_id);
      if (!stream.requested || stream.restored) {
        return;
      }
    }
    if (setMessageRate(message_id, rate_hz)) {
      std::lock_guard<std::mutex> lock(streams_mutex_);
      StreamState& stream = streamFor(message_id);
      stream.restored = true;
      stream.requested = false;
      ROS_INFO("restored MAVLink message %u to %.1f Hz", message_id, rate_hz);
    }
  }

  bool requestTelemetryRates() {
    // Dispatched on the service thread pool so a slow MAVROS service reply
    // can never stall the 100 Hz control timer (the default callback queue
    // is single-threaded).
    service_io_.post([this]() {
      requestStream(kLocalPositionNedMessageId,
                    params_.requested_telemetry_rate_hz);
      requestStream(kPositionTargetLocalNedMessageId,
                    params_.requested_telemetry_rate_hz);
    });
    return true;
  }

  void telemetryTimer(const ros::WallTimerEvent&) {
    if (!have_state_ || !latest_state_.connected) {
      return;
    }
    controller_->setTelemetryRequested(streamsRequested());
    if (!controller_->offboardActive()) {
      return;  // streams are requested on the OFFBOARD edge
    }
    // Only a session that still has steps ahead may re-request the 100 Hz test
    // rate. A finished hold (COMPLETE) or a refused/aborted hold (ABORT) must
    // keep the normal rates that restoreTelemetryRates() just applied, instead
    // of being bounced straight back to 100 Hz.
    const std::uint8_t phase = controller_->phase();
    if (phase < kStabilize || phase > kHoldDown) {
      return;
    }
    // Per-stream retry: the session is running but one of the two 100 Hz
    // requests did not stick; recover only the pending stream(s).
    bool local_pending, target_pending;
    {
      std::lock_guard<std::mutex> lock(streams_mutex_);
      local_pending = !local_stream_.requested;
      target_pending = !target_stream_.requested;
    }
    if (local_pending) {
      service_io_.post([this]() {
        requestStream(kLocalPositionNedMessageId,
                      params_.requested_telemetry_rate_hz);
      });
    }
    if (target_pending) {
      service_io_.post([this]() {
        requestStream(kPositionTargetLocalNedMessageId,
                      params_.requested_telemetry_rate_hz);
      });
    }
  }

  bool setMessageRate(std::uint32_t message_id, double rate_hz) {
    mavros_msgs::MessageInterval service;
    service.request.message_id = message_id;
    service.request.message_rate = static_cast<float>(rate_hz);
    if (!message_interval_client_.call(service) || !service.response.success) {
      ROS_WARN_THROTTLE(2.0, "failed to set MAVLink message %u to %.1f Hz",
                        message_id, rate_hz);
      return false;
    }
    return true;
  }

  void restoreTelemetryRates() {
    if (!have_state_ || !latest_state_.connected) {
      return;
    }
    service_io_.post([this]() {
      restoreStream(kLocalPositionNedMessageId,
                    params_.restore_local_position_rate_hz);
      restoreStream(kPositionTargetLocalNedMessageId,
                    params_.restore_target_local_rate_hz);
    });
  }

  void statusTimer(const ros::WallTimerEvent&) {
    const double now = steadyNowSec();
    if (restore_after_end_at_ > 0.0 && now >= restore_after_end_at_) {
      // COMPLETE/ABORT tail has drained while still holding in OFFBOARD.
      restore_after_end_at_ = -1.0;
      restoreTelemetryRates();
    }
    VzStepStatus status;
    status.header.stamp = ros::Time::now();
    status.state = controller_->phase();
    status.state_text = phaseText(controller_->phase());
    status.reason = controller_->reason();
    status.phase_elapsed_sec = controller_->phaseElapsed(now);
    status.cycle_index = static_cast<std::uint32_t>(controller_->cycleIndex());
    status.cycle_count = static_cast<std::uint32_t>(params_.cycle_count);
    status.step_speed_mps = params_.step_speed_mps;
    status.step_duration_sec = params_.step_duration_sec;
    status.theoretical_displacement_m = params_.displacement();
    status.command_vz_mps = controller_->commandVz();
    status.setpoint_rate_hz = controller_->setpointRate().rate(now);
    status.target_rate_hz = controller_->targetRate().rate(now);
    status.velocity_rate_hz = controller_->velocityRate().rate(now);
    status.pose_rate_hz = controller_->poseRate().rate(now);
    status.latched_x_m = controller_->latchedX();
    status.latched_y_m = controller_->latchedY();
    status.latched_z_m = controller_->latchedZ();
    status.latched_yaw_rad = controller_->latchedYaw();
    status.current_z_m = have_odom_ ? latest_odom_.pose.pose.position.z
                                    : std::numeric_limits<double>::quiet_NaN();
    status.current_vz_mps = have_velocity_ ? latest_vz_ : 0.0;
    status.min_z_m = controller_->minZ();
    status.max_z_m = controller_->maxZ();
    status_publisher_.publish(status);
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher setpoint_publisher_;
  ros::Publisher status_publisher_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber pose_subscriber_;
  ros::Subscriber target_subscriber_;
  ros::Subscriber setpoint_monitor_subscriber_;
  ros::Subscriber recorder_health_subscriber_;
  ros::ServiceClient message_interval_client_;
  ros::WallTimer control_timer_;
  ros::WallTimer status_timer_;
  ros::WallTimer telemetry_timer_;

  StepParams params_;
  std::unique_ptr<StepController> controller_;

  mavros_msgs::State latest_state_;
  nav_msgs::Odometry latest_odom_;
  double latest_vz_{0.0};
  bool have_state_{false};
  bool have_odom_{false};
  bool have_velocity_{false};

  // Per-stream MAVLink rate state, guarded by streams_mutex_. The service
  // calls run on service_io_ so a slow reply never blocks the control timer.
  StreamState local_stream_;
  StreamState target_stream_;
  std::mutex streams_mutex_;
  boost::asio::io_service service_io_;
  std::unique_ptr<boost::asio::io_service::work> service_work_{
      new boost::asio::io_service::work(service_io_)};
  std::thread service_thread_;

  StreamState& streamFor(std::uint32_t message_id) {
    // Caller holds streams_mutex_.
    return message_id == kLocalPositionNedMessageId ? local_stream_
                                                    : target_stream_;
  }
  bool enable_setpoint_output_{true};
  bool recorder_required_{false};
  double telemetry_tail_sec_{1.0};
  // Wall-clock instant at which a finished (COMPLETE/ABORT) session's 100 Hz
  // streams should be restored; <= 0 means no restore is pending.
  double restore_after_end_at_{-1.0};

  std::string state_topic_;
  std::string odom_topic_;
  std::string velocity_topic_;
  std::string pose_topic_;
  std::string target_topic_;
  std::string setpoint_topic_;
  std::string status_topic_;
  std::string recorder_topic_;
  std::string message_interval_service_;
};

}  // namespace vz_step_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "vz_step_control_node");
  try {
    vz_step_control::VzStepControlNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL_STREAM("failed to start vz_step_control: " << exception.what());
    return 1;
  }
  return 0;
}
