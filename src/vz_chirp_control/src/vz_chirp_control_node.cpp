#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/MessageInterval.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <vz_chirp_control/VzChirpStatus.h>
#include "vz_chirp_control/chirp_profile.h"

namespace vz_chirp_control {
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

class RateTracker {
 public:
  explicit RateTracker(double window_sec = 2.0) : window_sec_(window_sec) {}

  void setWindow(double window_sec) { window_sec_ = window_sec; }

  void tick(double now_sec) {
    samples_.push_back(now_sec);
    prune(now_sec);
  }

  double rate(double now_sec) {
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
  void prune(double now_sec) {
    while (samples_.size() > 2 && now_sec - samples_.front() > window_sec_) {
      samples_.pop_front();
    }
  }

  double window_sec_;
  std::deque<double> samples_;
};

}  // namespace

class VzChirpControlNode {
 public:
  VzChirpControlNode() : private_node_("~") {
    readParameters();
    validateParameters();
    chirp_.reset(new ChirpProfile(
        amplitude_mps_, start_frequency_hz_, end_frequency_hz_,
        sweep_duration_sec_, fade_duration_sec_, publish_rate_hz_));
    if (chirp_->theoreticalSpan() >= maximum_vertical_span_m_) {
      throw std::invalid_argument(
          "theoretical chirp displacement exceeds maximum_vertical_span_m");
    }

    setpoint_rate_.setWindow(rate_window_sec_);
    target_rate_.setWindow(rate_window_sec_);
    velocity_rate_.setWindow(rate_window_sec_);

    status_publisher_ = node_.advertise<VzChirpStatus>(status_topic_, 10, true);
    if (enable_setpoint_output_) {
      setpoint_publisher_ =
          node_.advertise<mavros_msgs::PositionTarget>(setpoint_topic_, 20);
    }

    state_subscriber_ = node_.subscribe(state_topic_, 20,
                                        &VzChirpControlNode::stateCallback, this);
    odom_subscriber_ = node_.subscribe(odom_topic_, 100,
                                       &VzChirpControlNode::odomCallback, this,
                                       ros::TransportHints().tcpNoDelay());
    velocity_subscriber_ = node_.subscribe(
        velocity_topic_, 200, &VzChirpControlNode::velocityCallback, this,
        ros::TransportHints().tcpNoDelay());
    target_subscriber_ = node_.subscribe(
        target_topic_, 200, &VzChirpControlNode::targetCallback, this,
        ros::TransportHints().tcpNoDelay());

    message_interval_client_ =
        node_.serviceClient<mavros_msgs::MessageInterval>(message_interval_service_);
    control_timer_ = node_.createWallTimer(
        ros::WallDuration(1.0 / publish_rate_hz_),
        &VzChirpControlNode::controlTimer, this);
    status_timer_ = node_.createWallTimer(
        ros::WallDuration(0.1), &VzChirpControlNode::statusTimer, this);
    telemetry_timer_ = node_.createWallTimer(
        ros::WallDuration(1.0), &VzChirpControlNode::telemetryTimer, this);

    ROS_WARN_STREAM("VZ chirp ready: " << start_frequency_hz_ << " -> "
                                        << end_frequency_hz_ << " Hz, amplitude="
                                        << amplitude_mps_ << " m/s, duration="
                                        << sweep_duration_sec_ << " s, publish="
                                        << publish_rate_hz_ << " Hz, theoretical "
                                        << "displacement span="
                                        << chirp_->theoreticalSpan() << " m");
    if (!enable_setpoint_output_) {
      ROS_WARN("PREVIEW ONLY: /mavros/setpoint_raw/local publisher is disabled");
    }
  }

  ~VzChirpControlNode() { restoreTelemetryRates(); }

 private:
  void readParameters() {
    private_node_.param("enable_setpoint_output", enable_setpoint_output_, true);
    private_node_.param("publish_rate_hz", publish_rate_hz_, 100.0);
    private_node_.param("amplitude_mps", amplitude_mps_, 0.2);
    private_node_.param("start_frequency_hz", start_frequency_hz_, 0.2);
    private_node_.param("end_frequency_hz", end_frequency_hz_, 8.0);
    private_node_.param("sweep_duration_sec", sweep_duration_sec_, 60.0);
    private_node_.param("fade_duration_sec", fade_duration_sec_, 2.0);
    private_node_.param("stabilize_duration_sec", stabilize_duration_sec_, 3.0);
    private_node_.param("post_hold_duration_sec", post_hold_duration_sec_, 5.0);
    private_node_.param("requested_telemetry_rate_hz",
                        requested_telemetry_rate_hz_, 100.0);
    private_node_.param("minimum_data_rate_hz", minimum_data_rate_hz_, 90.0);
    private_node_.param("rate_window_sec", rate_window_sec_, 2.0);
    private_node_.param("rate_fault_hold_sec", rate_fault_hold_sec_, 1.0);
    private_node_.param("maximum_topic_age_sec", maximum_topic_age_sec_, 0.1);
    private_node_.param("state_timeout_sec", state_timeout_sec_, 1.0);
    private_node_.param("maximum_vertical_offset_m", maximum_vertical_offset_m_,
                        0.45);
    private_node_.param("maximum_vertical_span_m", maximum_vertical_span_m_,
                        0.8);
    private_node_.param("maximum_absolute_vz_mps", maximum_absolute_vz_mps_,
                        0.6);
    private_node_.param("maximum_xy_error_m", maximum_xy_error_m_, 0.3);
    private_node_.param("restore_local_position_rate_hz",
                        restore_local_position_rate_hz_, 30.0);
    private_node_.param("restore_target_local_rate_hz",
                        restore_target_local_rate_hz_, 10.0);

    private_node_.param<std::string>("state_topic", state_topic_,
                                     "/mavros/state");
    private_node_.param<std::string>("odom_topic", odom_topic_,
                                     "/mavros/local_position/odom");
    private_node_.param<std::string>(
        "velocity_topic", velocity_topic_,
        "/mavros/local_position/velocity_local");
    private_node_.param<std::string>(
        "target_topic", target_topic_, "/mavros/setpoint_raw/target_local");
    private_node_.param<std::string>(
        "setpoint_topic", setpoint_topic_, "/mavros/setpoint_raw/local");
    private_node_.param<std::string>("status_topic", status_topic_,
                                     "/vz_chirp/status");
    private_node_.param<std::string>(
        "message_interval_service", message_interval_service_,
        "/mavros/set_message_interval");
  }

  void validateParameters() const {
    const bool finite =
        std::isfinite(publish_rate_hz_) && std::isfinite(amplitude_mps_) &&
        std::isfinite(start_frequency_hz_) &&
        std::isfinite(end_frequency_hz_) &&
        std::isfinite(sweep_duration_sec_) &&
        std::isfinite(fade_duration_sec_) &&
        std::isfinite(stabilize_duration_sec_) &&
        std::isfinite(post_hold_duration_sec_) &&
        std::isfinite(requested_telemetry_rate_hz_) &&
        std::isfinite(minimum_data_rate_hz_) &&
        std::isfinite(rate_window_sec_) &&
        std::isfinite(rate_fault_hold_sec_) &&
        std::isfinite(maximum_topic_age_sec_) &&
        std::isfinite(state_timeout_sec_) &&
        std::isfinite(maximum_vertical_offset_m_) &&
        std::isfinite(maximum_vertical_span_m_) &&
        std::isfinite(maximum_absolute_vz_mps_) &&
        std::isfinite(maximum_xy_error_m_);
    if (!finite || publish_rate_hz_ <= 0.0 || amplitude_mps_ <= 0.0 ||
        start_frequency_hz_ <= 0.0 ||
        end_frequency_hz_ <= start_frequency_hz_ ||
        end_frequency_hz_ * 10.0 > publish_rate_hz_ + 1e-9 ||
        sweep_duration_sec_ <= 0.0 || fade_duration_sec_ < 0.0 ||
        2.0 * fade_duration_sec_ > sweep_duration_sec_ ||
        stabilize_duration_sec_ < rate_window_sec_ ||
        post_hold_duration_sec_ < 0.0 || requested_telemetry_rate_hz_ <= 0.0 ||
        minimum_data_rate_hz_ <= 0.0 ||
        requested_telemetry_rate_hz_ < minimum_data_rate_hz_ ||
        rate_window_sec_ <= 0.0 || rate_fault_hold_sec_ < 0.0 ||
        maximum_topic_age_sec_ <= 0.0 || state_timeout_sec_ <= 0.0 ||
        maximum_vertical_offset_m_ <= 0.0 ||
        maximum_vertical_span_m_ <= 0.0 || maximum_vertical_span_m_ >= 1.0 ||
        maximum_absolute_vz_mps_ <= amplitude_mps_ ||
        maximum_xy_error_m_ <= 0.0) {
      throw std::invalid_argument("invalid vz_chirp_control parameters");
    }
  }

  void stateCallback(const mavros_msgs::State::ConstPtr& message) {
    const double now = steadyNowSec();
    const bool was_offboard = offboard_active_;
    state_ = *message;
    have_state_ = true;
    latest_state_time_ = now;
    offboard_active_ =
        state_.connected && state_.armed && state_.mode == "OFFBOARD";

    if (offboard_active_ && !was_offboard) {
      offboard_session_started_ = true;
      beginOffboard(now);
    } else if (!offboard_active_ && was_offboard) {
      if (stage_ == VzChirpStatus::STABILIZE ||
          stage_ == VzChirpStatus::SWEEP ||
          stage_ == VzChirpStatus::POST_HOLD) {
        abort("left OFFBOARD or disarmed");
      }
      restoreTelemetryRates();
    }
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    latest_odom_ = *message;
    have_odom_ = true;
    latest_odom_time_ = steadyNowSec();
    if (offboard_active_ && hold_target_locked_) {
      const double z = latest_odom_.pose.pose.position.z;
      minimum_z_m_ = std::min(minimum_z_m_, z);
      maximum_z_m_ = std::max(maximum_z_m_, z);
    }
  }

  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& message) {
    latest_velocity_ = *message;
    have_velocity_ = true;
    const double now = steadyNowSec();
    latest_velocity_time_ = now;
    velocity_rate_.tick(now);
  }

  void targetCallback(const mavros_msgs::PositionTarget::ConstPtr&) {
    const double now = steadyNowSec();
    latest_target_time_ = now;
    target_rate_.tick(now);
  }

  void beginOffboard(double now) {
    lockCurrentTarget();
    if (!have_odom_ || now - latest_odom_time_ > maximum_topic_age_sec_ ||
        !telemetry_rates_requested_) {
      abort("entered OFFBOARD before odometry/telemetry preflight was ready");
      return;
    }
    offboard_entry_time_ = now;
    rate_fault_start_time_ = -1.0;
    setStage(VzChirpStatus::STABILIZE, "OFFBOARD entered; stabilizing");
  }

  void lockCurrentTarget() {
    if (!have_odom_) {
      return;
    }
    hold_x_m_ = latest_odom_.pose.pose.position.x;
    hold_y_m_ = latest_odom_.pose.pose.position.y;
    hold_yaw_rad_ = quaternionYaw(latest_odom_.pose.pose.orientation);
    start_z_m_ = latest_odom_.pose.pose.position.z;
    minimum_z_m_ = start_z_m_;
    maximum_z_m_ = start_z_m_;
    hold_target_locked_ = true;
  }

  void controlTimer(const ros::WallTimerEvent&) {
    const double now = steadyNowSec();
    updatePrestreamState(now);

    double command_vz = 0.0;
    command_frequency_hz_ = 0.0;
    if (stage_ == VzChirpStatus::WAIT_FCU ||
        stage_ == VzChirpStatus::PRESTREAM ||
        stage_ == VzChirpStatus::STABILIZE) {
      sweep_elapsed_sec_ = 0.0;
    }

    if (stage_ == VzChirpStatus::STABILIZE) {
      if (!offboard_active_) {
        abort("OFFBOARD gate lost while stabilizing");
      } else if (!safetyOkay(now)) {
        // safetyOkay sets the specific reason.
      } else if (now - offboard_entry_time_ >= stabilize_duration_sec_) {
        if (!ratesOkay(now)) {
          abort("one or more 100 Hz data streams are below 90 Hz");
        } else {
          sweep_start_time_ = now;
          setStage(VzChirpStatus::SWEEP, "chirp running");
        }
      }
    }

    if (stage_ == VzChirpStatus::SWEEP) {
      sweep_elapsed_sec_ = std::max(0.0, now - sweep_start_time_);
      if (!offboard_active_) {
        abort("OFFBOARD gate lost during sweep");
      } else if (!safetyOkay(now)) {
        // safetyOkay sets the specific reason.
      } else if (!ratesOkay(now)) {
        if (rate_fault_start_time_ < 0.0) {
          rate_fault_start_time_ = now;
        } else if (now - rate_fault_start_time_ >= rate_fault_hold_sec_) {
          abort("100 Hz data rate remained below threshold");
        }
      } else {
        rate_fault_start_time_ = -1.0;
      }

      if (stage_ == VzChirpStatus::SWEEP &&
          sweep_elapsed_sec_ >= sweep_duration_sec_) {
        post_hold_start_time_ = now;
        setStage(VzChirpStatus::POST_HOLD, "sweep complete; zero-vz hold");
      } else if (stage_ == VzChirpStatus::SWEEP) {
        command_vz = chirp_->value(sweep_elapsed_sec_);
        command_frequency_hz_ =
            chirp_->instantaneousFrequency(sweep_elapsed_sec_);
      }
    }

    if (stage_ == VzChirpStatus::POST_HOLD) {
      sweep_elapsed_sec_ = sweep_duration_sec_;
      if (!offboard_active_) {
        abort("OFFBOARD gate lost during post-hold");
      } else if (!safetyOkay(now)) {
        // safetyOkay sets the specific reason.
      } else if (now - post_hold_start_time_ >= post_hold_duration_sec_) {
        setStage(VzChirpStatus::COMPLETE,
                 "complete; holding zero vz until OFFBOARD exit");
      }
    } else if (stage_ == VzChirpStatus::COMPLETE) {
      sweep_elapsed_sec_ = sweep_duration_sec_;
    }

    command_vz_mps_ = command_vz;
    publishSetpoint(now, command_vz);
  }

  void updatePrestreamState(double now) {
    if (offboard_active_ || stage_ == VzChirpStatus::ABORT ||
        stage_ == VzChirpStatus::COMPLETE) {
      return;
    }
    if (have_odom_) {
      lockCurrentTarget();
    }
    const bool ready = have_state_ && state_.connected && have_odom_ &&
                       now - latest_state_time_ <= state_timeout_sec_ &&
                       now - latest_odom_time_ <= maximum_topic_age_sec_ &&
                       telemetry_rates_requested_;
    setStage(ready ? VzChirpStatus::PRESTREAM : VzChirpStatus::WAIT_FCU,
             ready ? "ready; streaming zero vz and waiting for OFFBOARD"
                   : "waiting for FCU, odometry and telemetry setup");
  }

  bool ratesOkay(double now) {
    return setpoint_rate_.rate(now) >= minimum_data_rate_hz_ &&
           target_rate_.rate(now) >= minimum_data_rate_hz_ &&
           velocity_rate_.rate(now) >= minimum_data_rate_hz_ &&
           setpoint_rate_.age(now) <= maximum_topic_age_sec_ &&
           target_rate_.age(now) <= maximum_topic_age_sec_ &&
           velocity_rate_.age(now) <= maximum_topic_age_sec_;
  }

  bool safetyOkay(double now) {
    if (!have_odom_ || now - latest_odom_time_ > maximum_topic_age_sec_) {
      abort("local odometry is stale");
      return false;
    }
    if (!have_velocity_ || now - latest_velocity_time_ > maximum_topic_age_sec_) {
      abort("local velocity is stale");
      return false;
    }
    if (have_state_ && now - latest_state_time_ > state_timeout_sec_) {
      abort("FCU state is stale");
      return false;
    }

    const double x = latest_odom_.pose.pose.position.x;
    const double y = latest_odom_.pose.pose.position.y;
    const double z = latest_odom_.pose.pose.position.z;
    if (std::abs(z - start_z_m_) > maximum_vertical_offset_m_) {
      abort("vertical offset safety limit exceeded");
      return false;
    }
    if (maximum_z_m_ - minimum_z_m_ > maximum_vertical_span_m_) {
      abort("vertical peak-to-peak safety limit exceeded");
      return false;
    }
    if (std::hypot(x - hold_x_m_, y - hold_y_m_) > maximum_xy_error_m_) {
      abort("horizontal position safety limit exceeded");
      return false;
    }
    if (std::abs(latest_velocity_.twist.linear.z) >
        maximum_absolute_vz_mps_) {
      abort("vertical velocity safety limit exceeded");
      return false;
    }
    return true;
  }

  void publishSetpoint(double now, double command_vz) {
    if (!enable_setpoint_output_ || !state_.connected || !have_odom_ ||
        !hold_target_locked_) {
      return;
    }
    mavros_msgs::PositionTarget message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = latest_odom_.header.frame_id;
    message.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    message.type_mask = mavros_msgs::PositionTarget::IGNORE_PZ |
                        mavros_msgs::PositionTarget::IGNORE_VX |
                        mavros_msgs::PositionTarget::IGNORE_VY |
                        mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    message.position.x = hold_x_m_;
    message.position.y = hold_y_m_;
    message.velocity.z = command_vz;
    message.yaw = hold_yaw_rad_;
    setpoint_publisher_.publish(message);
    setpoint_rate_.tick(now);
  }

  void telemetryTimer(const ros::WallTimerEvent&) {
    if (!have_state_ || !state_.connected || telemetry_rates_requested_ ||
        (offboard_session_started_ && !offboard_active_)) {
      return;
    }
    const bool local_ok = setMessageRate(kLocalPositionNedMessageId,
                                         requested_telemetry_rate_hz_);
    const bool target_ok = setMessageRate(kPositionTargetLocalNedMessageId,
                                          requested_telemetry_rate_hz_);
    telemetry_rates_requested_ = local_ok && target_ok;
    if (telemetry_rates_requested_) {
      telemetry_rates_restored_ = false;
      ROS_INFO("requested MAVLink LOCAL_POSITION_NED and POSITION_TARGET_LOCAL_NED at %.1f Hz",
               requested_telemetry_rate_hz_);
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
    if (!telemetry_rates_requested_ || telemetry_rates_restored_ ||
        !have_state_ || !state_.connected) {
      return;
    }
    const bool local_ok = setMessageRate(kLocalPositionNedMessageId,
                                         restore_local_position_rate_hz_);
    const bool target_ok = setMessageRate(kPositionTargetLocalNedMessageId,
                                          restore_target_local_rate_hz_);
    if (local_ok && target_ok) {
      telemetry_rates_restored_ = true;
      telemetry_rates_requested_ = false;
      ROS_INFO("restored MAVLink message rates to local=%.1f Hz target=%.1f Hz",
               restore_local_position_rate_hz_, restore_target_local_rate_hz_);
    }
  }

  void statusTimer(const ros::WallTimerEvent&) {
    const double now = steadyNowSec();
    VzChirpStatus status;
    status.header.stamp = ros::Time::now();
    status.state = stage_;
    status.state_text = stageText(stage_);
    status.reason = reason_;
    status.sweep_elapsed_sec = sweep_elapsed_sec_;
    status.instantaneous_frequency_hz = command_frequency_hz_;
    status.command_vz_mps = command_vz_mps_;
    status.setpoint_rate_hz = setpoint_rate_.rate(now);
    status.target_rate_hz = target_rate_.rate(now);
    status.velocity_rate_hz = velocity_rate_.rate(now);
    status.start_z_m = start_z_m_;
    status.current_z_m =
        have_odom_ ? latest_odom_.pose.pose.position.z
                   : std::numeric_limits<double>::quiet_NaN();
    status.min_z_m = minimum_z_m_;
    status.max_z_m = maximum_z_m_;
    status_publisher_.publish(status);
  }

  void abort(const std::string& reason) {
    if (stage_ == VzChirpStatus::ABORT) {
      return;
    }
    setStage(VzChirpStatus::ABORT, reason);
    command_vz_mps_ = 0.0;
    command_frequency_hz_ = 0.0;
    ROS_ERROR_STREAM("VZ chirp aborted: " << reason);
  }

  void setStage(std::uint8_t stage, const std::string& reason) {
    if (stage_ == stage && reason_ == reason) {
      return;
    }
    const bool changed = stage_ != stage;
    stage_ = stage;
    reason_ = reason;
    if (changed) {
      ROS_INFO_STREAM("VZ chirp state -> " << stageText(stage_) << ": "
                                            << reason_);
    }
  }

  static const char* stageText(std::uint8_t stage) {
    switch (stage) {
      case VzChirpStatus::WAIT_FCU:
        return "WAIT_FCU";
      case VzChirpStatus::PRESTREAM:
        return "PRESTREAM";
      case VzChirpStatus::STABILIZE:
        return "STABILIZE";
      case VzChirpStatus::SWEEP:
        return "SWEEP";
      case VzChirpStatus::POST_HOLD:
        return "POST_HOLD";
      case VzChirpStatus::COMPLETE:
        return "COMPLETE";
      case VzChirpStatus::ABORT:
        return "ABORT";
      default:
        return "UNKNOWN";
    }
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Publisher setpoint_publisher_;
  ros::Publisher status_publisher_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber target_subscriber_;
  ros::ServiceClient message_interval_client_;
  ros::WallTimer control_timer_;
  ros::WallTimer status_timer_;
  ros::WallTimer telemetry_timer_;

  std::unique_ptr<ChirpProfile> chirp_;
  RateTracker setpoint_rate_;
  RateTracker target_rate_;
  RateTracker velocity_rate_;

  mavros_msgs::State state_;
  nav_msgs::Odometry latest_odom_;
  geometry_msgs::TwistStamped latest_velocity_;
  bool have_state_{false};
  bool have_odom_{false};
  bool have_velocity_{false};
  bool offboard_active_{false};
  bool offboard_session_started_{false};
  bool hold_target_locked_{false};
  bool telemetry_rates_requested_{false};
  bool telemetry_rates_restored_{false};

  std::uint8_t stage_{VzChirpStatus::WAIT_FCU};
  std::string reason_{"waiting for FCU"};
  double latest_state_time_{0.0};
  double latest_odom_time_{0.0};
  double latest_velocity_time_{0.0};
  double latest_target_time_{0.0};
  double offboard_entry_time_{0.0};
  double sweep_start_time_{0.0};
  double post_hold_start_time_{0.0};
  double rate_fault_start_time_{-1.0};
  double hold_x_m_{0.0};
  double hold_y_m_{0.0};
  double hold_yaw_rad_{0.0};
  double start_z_m_{0.0};
  double minimum_z_m_{0.0};
  double maximum_z_m_{0.0};
  double sweep_elapsed_sec_{0.0};
  double command_frequency_hz_{0.0};
  double command_vz_mps_{0.0};

  bool enable_setpoint_output_{true};
  double publish_rate_hz_{100.0};
  double amplitude_mps_{0.2};
  double start_frequency_hz_{0.2};
  double end_frequency_hz_{8.0};
  double sweep_duration_sec_{60.0};
  double fade_duration_sec_{2.0};
  double stabilize_duration_sec_{3.0};
  double post_hold_duration_sec_{5.0};
  double requested_telemetry_rate_hz_{100.0};
  double minimum_data_rate_hz_{90.0};
  double rate_window_sec_{2.0};
  double rate_fault_hold_sec_{1.0};
  double maximum_topic_age_sec_{0.1};
  double state_timeout_sec_{1.0};
  double maximum_vertical_offset_m_{0.45};
  double maximum_vertical_span_m_{0.8};
  double maximum_absolute_vz_mps_{0.6};
  double maximum_xy_error_m_{0.3};
  double restore_local_position_rate_hz_{30.0};
  double restore_target_local_rate_hz_{10.0};

  std::string state_topic_;
  std::string odom_topic_;
  std::string velocity_topic_;
  std::string target_topic_;
  std::string setpoint_topic_;
  std::string status_topic_;
  std::string message_interval_service_;
};

}  // namespace vz_chirp_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "vz_chirp_control_node");
  try {
    vz_chirp_control::VzChirpControlNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL_STREAM("failed to start vz_chirp_control: " << exception.what());
    return 1;
  }
  return 0;
}
