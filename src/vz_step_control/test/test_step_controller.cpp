#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "vz_step_control/step_controller.h"

namespace {

using vz_step_control::kAbort;
using vz_step_control::kComplete;
using vz_step_control::kPrestream;
using vz_step_control::kStabilize;
using vz_step_control::kUpStep;
using vz_step_control::kWaitFcu;
using vz_step_control::StepCommand;
using vz_step_control::StepController;
using vz_step_control::StepParams;

struct StepRun {
  bool velocity{false};
  int sign{0};
  int count{0};
};

std::vector<StepRun> compressRuns(const std::vector<StepCommand>& commands) {
  std::vector<StepRun> runs;
  for (const StepCommand& command : commands) {
    const bool velocity = command.velocity_z_mode;
    const int sign = velocity ? (command.vz > 0.0 ? 1 : -1) : 0;
    if (!runs.empty() && runs.back().velocity == velocity &&
        runs.back().sign == sign) {
      ++runs.back().count;
    } else {
      runs.push_back(StepRun{velocity, sign, 1});
    }
  }
  return runs;
}

// Deterministic ROS-free harness: feeds the controller at 100 Hz with a
// kinematic plant (velocity segments integrate vz, position segments track
// the commanded target perfectly).
struct Harness {
  StepParams params;
  std::unique_ptr<StepController> controller;
  double now{100.0};
  double dt{0.01};
  double odom_x{1.0};
  double odom_y{2.0};
  double odom_z{3.0};
  double odom_yaw{0.7};
  double odom_vz{0.0};
  bool connected{true};
  bool armed{true};
  bool offboard{false};
  bool telemetry_requested{true};
  int telemetry_period{1};   // 1 = every tick (100 Hz); 5 = 20 Hz
  bool feed_odom{true};
  bool feed_state{true};
  bool feed_recorder{false};
  bool recorder_healthy{true};
  bool recorder_recording{true};
  double creep_z_per_tick{0.0};  // non-zero disables plant z tracking
  double drift_x_per_tick{0.0};  // applied after the plant xy snap
  std::vector<StepCommand> log;

  explicit Harness(const StepParams& p = StepParams(),
                   int telemetry_period_ticks = 1)
      : params(p), telemetry_period(telemetry_period_ticks) {
    controller.reset(new StepController(params));
    controller->setSelfPublisherName("/test_node");
    controller->setTelemetryRequested(telemetry_requested);
    feed_recorder = params.recorder_required;
    spin(2.5);  // prime PRESTREAM and the sliding rate windows
    log.clear();
  }

  void pump() {
    static int tick_index = 0;
    const bool feed_telemetry = (tick_index % telemetry_period) == 0;
    if (feed_state) {
      controller->setState(connected, armed, offboard, now);
    }
    if (feed_odom) {
      controller->onOdom(odom_x, odom_y, odom_z, odom_yaw, now);
    }
    if (feed_recorder) {
      controller->onRecorderHealth(recorder_healthy, recorder_recording, now);
    }
    if (feed_telemetry) {
      controller->onVelocityVz(odom_vz, now);
      controller->noteTargetMessage(now);
    }
    const StepCommand command = controller->tick(now);
    if (command.publish) {
      controller->noteSetpointPublished(now);
      if (command.velocity_z_mode) {
        odom_z += command.vz * dt;
      } else if (creep_z_per_tick == 0.0) {
        odom_z = command.z;
      }
      if (drift_x_per_tick == 0.0) {
        odom_x = command.x;
      }
      odom_y = command.y;
    }
    odom_z += creep_z_per_tick;
    odom_x += drift_x_per_tick;
    log.push_back(command);
    ++tick_index;
    now += dt;
  }

  void spin(double seconds) {
    const int steps = static_cast<int>(seconds / dt + 0.5);
    for (int i = 0; i < steps; ++i) {
      pump();
    }
  }

  void enterOffboard() {
    offboard = true;
    log.clear();
    pump();
  }

  void exitOffboard(bool keep_armed = true) {
    offboard = false;
    armed = keep_armed;
    log.clear();
    pump();
  }
};

TEST(StepController, WaitFcuBecomesPrestreamWhenStateAndOdomAreFresh) {
  StepParams params;
  StepController controller(params);
  controller.setSelfPublisherName("/test_node");
  double now = 0.0;
  controller.setState(true, false, false, now);
  controller.tick(now);
  EXPECT_EQ(controller.phase(), kWaitFcu);  // no odometry yet

  controller.onOdom(0.0, 0.0, 0.0, 0.0, now);
  controller.onVelocityVz(0.0, now);
  controller.noteTargetMessage(now);
  controller.tick(now + 0.01);
  EXPECT_EQ(controller.phase(), kPrestream);
  EXPECT_TRUE(controller.tick(now + 0.02).publish);
}

TEST(StepController, RestartInsideOffboardAbortsAndStillHolds) {
  StepParams params;
  StepController controller(params);
  controller.setSelfPublisherName("/test_node");
  controller.setTelemetryRequested(true);
  double now = 100.0;
  // Node (re)starts while the vehicle is already flying in OFFBOARD: the very
  // first state message is an active session, so no operator edge was seen.
  controller.setState(true, true, true, now);
  EXPECT_EQ(controller.phase(), kAbort);
  EXPECT_NE(controller.reason().find("restarted inside OFFBOARD"),
            std::string::npos);

  controller.onOdom(1.0, 2.0, 3.0, 0.7, now);
  const StepCommand command = controller.tick(now + 0.01);
  // The ABORT hold must publish as soon as odometry exists: setpoints must
  // never stay silent while the vehicle is in OFFBOARD.
  ASSERT_TRUE(command.publish);
  EXPECT_FALSE(command.velocity_z_mode);
  EXPECT_NEAR(command.z, 3.0, 1e-12);
  EXPECT_NEAR(command.x, 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(command.vz, 0.0);

  // Cycling OFFBOARD off and on arms a genuine session.
  controller.setState(true, true, false, now + 0.02);
  controller.setState(true, true, true, now + 0.03);
  EXPECT_EQ(controller.phase(), kStabilize);
}

TEST(StepController, PreflightFailureAbortsOffboardEntry) {
  Harness harness;
  harness.controller->setTelemetryRequested(false);
  harness.enterOffboard();
  EXPECT_EQ(harness.controller->phase(), kAbort);
  EXPECT_NE(harness.controller->reason().find("preflight"),
            std::string::npos);
  // ABORT holds the last locked position with a finite Z target.
  const StepCommand command = harness.log.back();
  ASSERT_TRUE(command.publish);
  EXPECT_FALSE(command.velocity_z_mode);
  EXPECT_TRUE(std::isfinite(command.z));
  EXPECT_DOUBLE_EQ(command.vz, 0.0);
}

TEST(StepController, DefaultRunMatchesSpecifiedTimeline) {
  Harness harness;  // 0.5 m/s, 1 s, 3 s stabilize, 10 s holds, 2 cycles
  harness.enterOffboard();
  EXPECT_EQ(harness.controller->phase(), kStabilize);

  harness.spin(3.5);  // inside the first UP_STEP (3.0 s .. 4.0 s)
  EXPECT_EQ(harness.controller->phase(), kUpStep);
  EXPECT_EQ(harness.controller->cycleIndex(), 1);
  EXPECT_NEAR(harness.controller->commandVz(), 0.5, 1e-12);

  harness.spin(22.0);  // inside the second UP_STEP (25.0 s .. 26.0 s)
  EXPECT_EQ(harness.controller->phase(), kUpStep);
  EXPECT_EQ(harness.controller->cycleIndex(), 2);
  EXPECT_NEAR(harness.controller->commandVz(), 0.5, 1e-12);

  harness.spin(23.0);  // 48.5 s after entry; the 47 s plan is complete
  EXPECT_EQ(harness.controller->phase(), kComplete);

  const std::vector<StepRun> runs = compressRuns(harness.log);
  ASSERT_GE(runs.size(), 9u);
  EXPECT_FALSE(runs[0].velocity);
  EXPECT_NEAR(runs[0].count, 300, 1);   // STABILIZE 3 s
  EXPECT_TRUE(runs[1].velocity);
  EXPECT_EQ(runs[1].sign, 1);
  EXPECT_NEAR(runs[1].count, 100, 1);   // +0.5 m/s for 1.0 s
  EXPECT_FALSE(runs[2].velocity);
  EXPECT_NEAR(runs[2].count, 1000, 1);  // HOLD_UP 10 s
  EXPECT_TRUE(runs[3].velocity);
  EXPECT_EQ(runs[3].sign, -1);
  EXPECT_NEAR(runs[3].count, 100, 1);   // -0.5 m/s for 1.0 s
  EXPECT_FALSE(runs[4].velocity);
  EXPECT_NEAR(runs[4].count, 1000, 1);  // HOLD_DOWN 10 s
  EXPECT_TRUE(runs[5].velocity);
  EXPECT_EQ(runs[5].sign, 1);
  EXPECT_NEAR(runs[5].count, 100, 1);   // second cycle up
  EXPECT_FALSE(runs[6].velocity);
  EXPECT_NEAR(runs[6].count, 1000, 1);
  EXPECT_TRUE(runs[7].velocity);
  EXPECT_EQ(runs[7].sign, -1);
  EXPECT_NEAR(runs[7].count, 100, 1);   // second cycle down
  EXPECT_FALSE(runs[8].velocity);
  EXPECT_GE(runs[8].count, 1000);       // final hold runs into COMPLETE
}

TEST(StepController, VelocityStepsUseLatchedXyYawAndHardVz) {
  Harness harness;
  harness.enterOffboard();
  harness.spin(3.5);        // middle of the first up step
  harness.odom_yaw = 0.9;   // must not change the latched yaw mid-step
  harness.spin(0.2);
  for (std::size_t i = harness.log.size() - 20; i < harness.log.size(); ++i) {
    const StepCommand& command = harness.log[i];
    ASSERT_TRUE(command.publish);
    EXPECT_TRUE(command.velocity_z_mode);
    EXPECT_DOUBLE_EQ(command.vz, 0.5);
    EXPECT_DOUBLE_EQ(command.x, 1.0);
    EXPECT_DOUBLE_EQ(command.y, 2.0);
    EXPECT_DOUBLE_EQ(command.yaw, 0.7);
  }

  harness.spin(0.4);  // up step ends at 4.0 s; capture uses the latest odom
  EXPECT_EQ(harness.controller->phase(), vz_step_control::kHoldUp);
  const StepCommand hold = harness.log.back();
  EXPECT_FALSE(hold.velocity_z_mode);
  EXPECT_NEAR(hold.z, 3.5, 0.01);
  EXPECT_NEAR(hold.yaw, 0.9, 1e-12);
  EXPECT_DOUBLE_EQ(hold.vz, 0.0);
}

TEST(StepController, EveryPositionSegmentCarriesFiniteZTarget) {
  Harness harness;
  harness.enterOffboard();
  harness.spin(48.0);
  for (const StepCommand& command : harness.log) {
    ASSERT_TRUE(command.publish);
    EXPECT_TRUE(std::isfinite(command.x));
    EXPECT_TRUE(std::isfinite(command.y));
    EXPECT_TRUE(std::isfinite(command.z));
    EXPECT_TRUE(std::isfinite(command.yaw));
    if (!command.velocity_z_mode) {
      EXPECT_DOUBLE_EQ(command.vz, 0.0);
    } else {
      EXPECT_NEAR(std::abs(command.vz), 0.5, 1e-12);
    }
  }
}

TEST(StepController, CapturedHoldTargetsTrackTheSteps) {
  Harness harness;
  harness.enterOffboard();
  harness.spin(4.5);   // first HOLD_UP (up step ended at 4.0 s)
  EXPECT_NEAR(harness.controller->latchedZ(), 3.5, 0.01);
  harness.spin(11.0);  // first DOWN_STEP ended at 15.0 s; now in HOLD_DOWN
  EXPECT_NEAR(harness.controller->latchedZ(), 3.0, 0.01);
  harness.spin(11.0);  // second UP_STEP ended at 26.0 s; now in HOLD_UP
  EXPECT_NEAR(harness.controller->latchedZ(), 3.5, 0.01);
  harness.spin(11.5);  // second DOWN_STEP ended at 37.0 s
  EXPECT_NEAR(harness.controller->latchedZ(), 3.0, 0.01);
  EXPECT_NEAR(harness.controller->minZ(), 3.0, 0.01);
  EXPECT_NEAR(harness.controller->maxZ(), 3.5, 0.01);
}

TEST(StepController, ExampleConfig025x2MatchesSpecifiedTimeline) {
  StepParams params;
  params.step_speed_mps = 0.25;
  params.step_duration_sec = 2.0;
  Harness harness(params);
  harness.enterOffboard();

  harness.spin(4.0);  // inside the first UP_STEP (3.0 s .. 5.0 s)
  EXPECT_EQ(harness.controller->phase(), kUpStep);
  EXPECT_NEAR(harness.controller->commandVz(), 0.25, 1e-12);

  harness.spin(24.5);  // inside the second UP_STEP (27.0 s .. 29.0 s)
  EXPECT_EQ(harness.controller->phase(), kUpStep);
  EXPECT_EQ(harness.controller->cycleIndex(), 2);

  harness.spin(25.0);  // 53.5 s after entry; the 51 s plan is complete
  EXPECT_EQ(harness.controller->phase(), kComplete);

  const std::vector<StepRun> runs = compressRuns(harness.log);
  ASSERT_GE(runs.size(), 9u);
  EXPECT_FALSE(runs[0].velocity);
  EXPECT_NEAR(runs[0].count, 300, 1);   // STABILIZE 3 s
  EXPECT_TRUE(runs[1].velocity);
  EXPECT_EQ(runs[1].sign, 1);
  EXPECT_NEAR(runs[1].count, 200, 1);   // +0.25 m/s for 2.0 s
  EXPECT_FALSE(runs[2].velocity);
  EXPECT_NEAR(runs[2].count, 1000, 1);
  EXPECT_TRUE(runs[3].velocity);
  EXPECT_EQ(runs[3].sign, -1);
  EXPECT_NEAR(runs[3].count, 200, 1);
  EXPECT_TRUE(runs[5].velocity);
  EXPECT_EQ(runs[5].sign, 1);
  EXPECT_NEAR(runs[5].count, 200, 1);
  EXPECT_TRUE(runs[7].velocity);
  EXPECT_EQ(runs[7].sign, -1);
  EXPECT_NEAR(runs[7].count, 200, 1);
}

TEST(StepController, CompleteDoesNotRerunInsideTheSameOffboard) {
  Harness harness;
  harness.enterOffboard();
  harness.spin(48.0);
  EXPECT_EQ(harness.controller->phase(), kComplete);
  harness.log.clear();
  harness.spin(3.0);
  EXPECT_EQ(harness.controller->phase(), kComplete);
  // phase_elapsed_sec must be measured from the COMPLETE transition, not
  // from the phase that preceded it. COMPLETE enters at 47.0 s while the
  // spins above ran to 48+3 s.
  EXPECT_NEAR(harness.controller->phaseElapsed(harness.now), 4.0, 0.5);
  for (const StepCommand& command : harness.log) {
    ASSERT_TRUE(command.publish);
    EXPECT_FALSE(command.velocity_z_mode);
    EXPECT_DOUBLE_EQ(command.vz, 0.0);
  }
}

TEST(StepController, LeavingOffboardReturnsToPrestreamAndReentryStartsFresh) {
  Harness harness;
  harness.enterOffboard();
  harness.spin(3.5);  // first UP_STEP
  EXPECT_EQ(harness.controller->phase(), kUpStep);

  harness.exitOffboard(true);
  EXPECT_EQ(harness.controller->phase(), kPrestream);
  const StepCommand streaming = harness.log.back();
  ASSERT_TRUE(streaming.publish);
  EXPECT_FALSE(streaming.velocity_z_mode);
  EXPECT_NEAR(streaming.z, harness.odom_z, 0.01);

  harness.enterOffboard();
  EXPECT_EQ(harness.controller->phase(), kStabilize);
  EXPECT_EQ(harness.controller->cycleIndex(), 0);
  harness.spin(47.5);
  EXPECT_EQ(harness.controller->phase(), kComplete);
}

TEST(StepController, DisarmEndsTheSessionLikeOffboardExit) {
  Harness harness;
  harness.enterOffboard();
  harness.spin(3.5);
  harness.armed = false;  // disarm while the mode switch lags behind
  harness.spin(0.1);
  EXPECT_EQ(harness.controller->phase(), kPrestream);
}

TEST(StepController, StaleOdometryAbortsAndHoldsLastValidPosition) {
  Harness harness;
  harness.enterOffboard();
  harness.spin(3.5);
  harness.feed_odom = false;
  harness.spin(0.5);
  EXPECT_EQ(harness.controller->phase(), kAbort);
  EXPECT_EQ(harness.controller->reason(), "local odometry is stale");
  const StepCommand hold = harness.log.back();
  ASSERT_TRUE(hold.publish);
  EXPECT_FALSE(hold.velocity_z_mode);
  EXPECT_NEAR(hold.z, 3.2, 0.1);  // last valid captured position
  EXPECT_DOUBLE_EQ(hold.vz, 0.0);
}

TEST(StepController, StaleStateAbortsAfterStateTimeout) {
  Harness harness;
  harness.enterOffboard();
  harness.spin(3.5);
  harness.feed_state = false;
  harness.spin(2.0);
  EXPECT_NE(harness.controller->phase(), kAbort);  // 2.5 s timeout not reached
  harness.spin(1.0);
  EXPECT_EQ(harness.controller->phase(), kAbort);
  EXPECT_EQ(harness.controller->reason(), "FCU state is stale");
}

TEST(StepController, StabilizeEndRequiresNinetyHzStreams) {
  // 20 Hz telemetry: fresh enough to avoid the staleness aborts, but below
  // the 90 Hz gate that guards the transition out of STABILIZE.
  Harness harness(StepParams{}, 5);
  harness.enterOffboard();
  harness.spin(3.5);
  EXPECT_EQ(harness.controller->phase(), kAbort);
  EXPECT_NE(harness.controller->reason().find("below 90 Hz"),
            std::string::npos);
}

TEST(StepController, RateFaultDuringStepsAbortsAfterGracePeriod) {
  Harness harness;
  harness.enterOffboard();
  harness.spin(4.5);  // first HOLD_UP
  ASSERT_EQ(harness.controller->phase(), vz_step_control::kHoldUp);
  harness.telemetry_period = 5;  // 20 Hz: fresh but below the 90 Hz gate
  harness.spin(1.5);             // grace is 1.0 s
  EXPECT_EQ(harness.controller->phase(), kAbort);
  EXPECT_EQ(harness.controller->reason(),
            "100 Hz data rate remained below threshold");
}

TEST(StepController, VerticalSpanExceedingAborts) {
  Harness harness;
  harness.creep_z_per_tick = 0.002;  // 0.2 m/s climb with zero reported vz
  harness.enterOffboard();
  harness.spin(6.0);
  EXPECT_EQ(harness.controller->phase(), kAbort);
  EXPECT_EQ(harness.controller->reason(),
            "vertical peak-to-peak safety limit exceeded");
}

TEST(StepController, VerticalUndershootExceedingAborts) {
  Harness harness;
  harness.creep_z_per_tick = -0.002;  // 0.2 m/s descent below start
  harness.enterOffboard();
  harness.spin(4.0);
  EXPECT_EQ(harness.controller->phase(), kAbort);
  EXPECT_EQ(harness.controller->reason(),
            "vertical undershoot safety limit exceeded");
}

TEST(StepController, HorizontalDriftExceedingAborts) {
  Harness harness;
  harness.drift_x_per_tick = 0.005;  // 0.5 m/s away from the entry point
  harness.enterOffboard();
  harness.spin(2.0);
  EXPECT_EQ(harness.controller->phase(), kAbort);
  EXPECT_EQ(harness.controller->reason(),
            "horizontal position safety limit exceeded");
}

TEST(StepController, XySafetyReferenceStaysAtEntryPoint) {
  // A re-captured hold target must not reset the XY safety reference: slow
  // cumulative walk-away has to abort at the 0.30 m envelope.
  Harness harness;
  harness.drift_x_per_tick = 0.0005;  // 0.05 m/s
  harness.enterOffboard();
  harness.spin(4.5);  // first capture re-anchors the hold target at ~0.2 m
  EXPECT_NEAR(harness.controller->latchedX(), 1.2, 0.02);
  EXPECT_EQ(harness.controller->phase(), vz_step_control::kHoldUp);
  harness.spin(2.5);  // cumulative drift passes 0.30 m at ~6.0 s
  EXPECT_EQ(harness.controller->phase(), kAbort);
  EXPECT_EQ(harness.controller->reason(),
            "horizontal position safety limit exceeded");
}

TEST(StepController, VerticalVelocityExceedingAborts) {
  Harness harness;
  harness.enterOffboard();
  harness.odom_vz = 0.7;  // maximum_absolute_vz_mps = 0.6
  harness.spin(0.1);
  EXPECT_EQ(harness.controller->phase(), kAbort);
  EXPECT_EQ(harness.controller->reason(),
            "vertical velocity safety limit exceeded");
}

TEST(StepController, DuplicatePublisherAbortsDuringSession) {
  Harness harness;
  harness.enterOffboard();
  harness.spin(4.0);
  EXPECT_TRUE(harness.controller->notePublisherCallerid("/other_node",
                                                        harness.now));
  EXPECT_EQ(harness.controller->phase(), kAbort);
  EXPECT_NE(harness.controller->reason().find("/other_node"),
            std::string::npos);
}

TEST(StepController, DuplicatePublisherRefusesOffboardEntry) {
  Harness harness;
  EXPECT_FALSE(harness.controller->notePublisherCallerid("/other_node",
                                                         harness.now));
  EXPECT_EQ(harness.controller->phase(), kPrestream);
  harness.enterOffboard();
  EXPECT_EQ(harness.controller->phase(), kAbort);
  EXPECT_NE(harness.controller->reason().find("duplicate setpoint publisher"),
            std::string::npos);
}

TEST(StepController, OwnCalleridIsIgnored) {
  Harness harness;
  harness.enterOffboard();
  EXPECT_FALSE(harness.controller->notePublisherCallerid("/test_node",
                                                         harness.now));
  EXPECT_EQ(harness.controller->phase(), kStabilize);
}

TEST(StepController, DisconnectReconnectInOffboardHoldsAndDoesNotRestart) {
  StepParams params;
  StepController controller(params);
  controller.setSelfPublisherName("/test_node");
  controller.setTelemetryRequested(true);
  double now = 100.0;
  // Healthy disengaged prime, exactly like a real takeoff script.
  controller.onOdom(1.0, 2.0, 3.0, 0.7, now);
  controller.onVelocityVz(0.0, now);
  controller.noteTargetMessage(now);
  controller.setState(true, true, false, now);
  controller.tick(now + 0.01);
  controller.setState(true, true, true, now + 0.02);
  EXPECT_EQ(controller.phase(), kStabilize);
  controller.tick(now + 0.03);

  // MAVLink link drops while the vehicle is flying in OFFBOARD. This must
  // only interrupt the session, never arm a fresh one.
  controller.setState(false, false, false, now + 0.10);
  EXPECT_EQ(controller.phase(), kWaitFcu);

  // The link returns and mavros reports OFFBOARD again, but no healthy
  // non-OFFBOARD state was observed in between: the controller must hold the
  // last position instead of silently re-running the step sequence.
  controller.onOdom(1.0, 2.0, 3.05, 0.7, now + 0.11);
  controller.onVelocityVz(0.0, now + 0.11);
  controller.noteTargetMessage(now + 0.11);
  controller.setState(true, true, true, now + 0.12);
  EXPECT_EQ(controller.phase(), kAbort);
  EXPECT_NE(controller.reason().find("MAVLink drop"), std::string::npos);

  const StepCommand held = controller.tick(now + 0.13);
  ASSERT_TRUE(held.publish);
  EXPECT_FALSE(held.velocity_z_mode);
  EXPECT_DOUBLE_EQ(held.vz, 0.0);

  // A genuine healthy exit + re-entry arms a fresh test.
  controller.setState(true, true, false, now + 0.20);
  EXPECT_EQ(controller.phase(), kPrestream);
  controller.setState(true, true, true, now + 0.21);
  EXPECT_EQ(controller.phase(), kStabilize);
}

TEST(StepController, DisconnectDuringPrestreamStillRequiresHealthyExit) {
  StepParams params;
  StepController controller(params);
  controller.setSelfPublisherName("/test_node");
  controller.setTelemetryRequested(true);
  double now = 0.0;
  controller.onOdom(0.0, 0.0, 1.2, 0.0, now);
  controller.onVelocityVz(0.0, now);
  controller.noteTargetMessage(now);
  controller.setState(true, true, false, now);
  controller.tick(now + 0.01);
  EXPECT_EQ(controller.phase(), kPrestream);

  // Brief link loss while idling in PRESTREAM.
  controller.setState(false, false, false, now + 0.02);
  // Reconnect lands straight in OFFBOARD: still not a manual exit, so the
  // controller must refuse to auto-start.
  controller.setState(true, true, true, now + 0.03);
  EXPECT_EQ(controller.phase(), kAbort);
  EXPECT_NE(controller.reason().find("MAVLink drop"), std::string::npos);

  // Healthy non-OFFBOARD then genuine OFFBOARD entry arms a fresh test.
  controller.setState(true, true, false, now + 0.04);
  controller.setState(true, true, true, now + 0.05);
  EXPECT_EQ(controller.phase(), kStabilize);
}

TEST(StepController, PrestreamWaitsForRequiredRecorderHealth) {
  StepParams params;
  params.recorder_required = true;
  StepController controller(params);
  controller.setSelfPublisherName("/test_node");
  controller.setTelemetryRequested(true);
  double now = 0.0;
  controller.onOdom(1.0, 2.0, 3.0, 0.7, now);
  controller.onVelocityVz(0.0, now);
  controller.noteTargetMessage(now);
  controller.setState(true, true, false, now);
  controller.tick(now + 0.01);
  // No recorder report yet: the controller must not claim it is "ready".
  EXPECT_EQ(controller.phase(), kWaitFcu);
  EXPECT_NE(controller.reason().find("bag recorder"), std::string::npos);

  controller.onRecorderHealth(true, false, now + 0.02);
  controller.tick(now + 0.03);
  EXPECT_EQ(controller.phase(), kPrestream);

  // Recorder becomes unhealthy while idling: back to not-ready.
  controller.onRecorderHealth(false, false, now + 0.04);
  controller.tick(now + 0.05);
  EXPECT_EQ(controller.phase(), kWaitFcu);
}

// A recorder that is healthy but has no bag open (e.g. respawned recorder
// waiting for a clean OFFBOARD re-entry) must not count as okay while a test
// is running: the STABILIZE->UP_STEP handoff is refused with an abort.
TEST(StepController, HealthyButNotRecordingAbortsRunningTest) {
  StepParams params;
  params.recorder_required = true;
  Harness harness(params);
  harness.feed_recorder = true;
  harness.recorder_healthy = true;
  harness.recorder_recording = false;
  harness.enterOffboard();
  EXPECT_EQ(harness.controller->phase(), kStabilize);
  harness.spin(4.0);  // past stabilize_duration_sec: the handoff must refuse
  EXPECT_EQ(harness.controller->phase(), kAbort);
  EXPECT_NE(harness.controller->reason().find("recorder"),
            std::string::npos);
  for (const StepCommand& command : harness.log) {
    ASSERT_TRUE(command.publish);
    EXPECT_FALSE(command.velocity_z_mode);  // no step ever ran unrecorded
    EXPECT_DOUBLE_EQ(command.vz, 0.0);
  }
}

TEST(StepController, RecorderRequiredMissingRefusesOffboardEntry) {
  StepParams params;
  params.recorder_required = true;  // recorder never reports health
  StepController controller(params);
  controller.setSelfPublisherName("/test_node");
  controller.setTelemetryRequested(true);
  double now = 0.0;
  controller.onOdom(1.0, 2.0, 3.0, 0.7, now);
  controller.onVelocityVz(0.0, now);
  controller.noteTargetMessage(now);
  controller.setState(true, true, false, now);
  controller.tick(now + 0.01);
  controller.setState(true, true, true, now + 0.02);
  EXPECT_EQ(controller.phase(), kAbort);
  EXPECT_NE(controller.reason().find("bag recorder"), std::string::npos);

  const StepCommand hold = controller.tick(now + 0.03);
  ASSERT_TRUE(hold.publish);
  EXPECT_FALSE(hold.velocity_z_mode);
  EXPECT_DOUBLE_EQ(hold.vz, 0.0);
}

TEST(StepController, RecorderPresentAllowsCompleteRun) {
  StepParams params;
  params.recorder_required = true;
  Harness harness(params);
  harness.enterOffboard();
  EXPECT_EQ(harness.controller->phase(), kStabilize);
  harness.spin(48.0);
  EXPECT_EQ(harness.controller->phase(), kComplete);
}

TEST(StepController, RecorderLostDuringStabilizeAbortsBeforeAnyStep) {
  StepParams params;
  params.recorder_required = true;
  Harness harness(params);
  harness.enterOffboard();
  EXPECT_EQ(harness.controller->phase(), kStabilize);
  harness.spin(0.5);
  harness.feed_recorder = false;  // recorder dies mid-STABILIZE
  harness.spin(3.5);              // the STABILIZE->UP_STEP gate must refuse
  EXPECT_EQ(harness.controller->phase(), kAbort);
  EXPECT_NE(harness.controller->reason().find("recorder"),
            std::string::npos);
  for (const StepCommand& command : harness.log) {
    ASSERT_TRUE(command.publish);
    EXPECT_FALSE(command.velocity_z_mode);  // no step ever ran unrecorded
    EXPECT_DOUBLE_EQ(command.vz, 0.0);
  }
}

TEST(StepController, RecorderLostDuringStepsAbortsAndHolds) {
  StepParams params;
  params.recorder_required = true;
  Harness harness(params);
  harness.enterOffboard();
  harness.spin(3.5);  // inside the first UP_STEP
  EXPECT_EQ(harness.controller->phase(), kUpStep);
  harness.feed_recorder = false;
  harness.spin(2.0);  // recorder_health_timeout_sec = 1.0
  EXPECT_EQ(harness.controller->phase(), kAbort);
  EXPECT_NE(harness.controller->reason().find("recorder"),
            std::string::npos);
  const StepCommand hold = harness.log.back();
  ASSERT_TRUE(hold.publish);
  EXPECT_FALSE(hold.velocity_z_mode);
  EXPECT_DOUBLE_EQ(hold.vz, 0.0);
}

TEST(StepController, RecorderUnhealthyDuringStepsAborts) {
  StepParams params;
  params.recorder_required = true;
  Harness harness(params);
  harness.enterOffboard();
  harness.spin(3.5);  // inside the first UP_STEP
  EXPECT_EQ(harness.controller->phase(), kUpStep);
  harness.recorder_healthy = false;  // recorder alive but cannot record
  harness.spin(0.1);
  EXPECT_EQ(harness.controller->phase(), kAbort);
  EXPECT_NE(harness.controller->reason().find("recorder"),
            std::string::npos);
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
