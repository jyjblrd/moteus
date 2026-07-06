// Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fw/flight_control_structs.h"

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

namespace tt = boost::test_tools;

namespace {

constexpr float NaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

using Result = FlightControlLogic::Result;

mcout::Frame MakeArmedFrame() {
  mcout::Frame frame;
  frame.flags =
      mcout::Frame::kMotorsArmed |
      mcout::Frame::kSoftArmed |
      mcout::Frame::kInterlock |
      mcout::Frame::kMotorsInitialized |
      mcout::Frame::kOutputEnabled;
  frame.spool_state = mcout::Frame::kThrottleUnlimited;
  frame.throttle = 0.5f;
  return frame;
}

FlightControlConfig MakeConfig() {
  FlightControlConfig config;
  config.enable = true;
  config.throttle_max_torque_Nm = 2.0f;
  config.idle_torque_Nm = 0.1f;
  config.cyclic_max_torque_Nm = 1.0f;
  config.max_torque_Nm = 3.0f;
  config.timeout_s = 0.1f;
  config.watchdog_timeout_s = 0.05f;
  return config;
}

// Drives Update() with a frame counter that advances on every call,
// mimicking a healthy 400Hz stream.
struct Harness {
  FlightControlLogic dut;
  FlightControlStatus status;
  uint32_t frame_count = 0;

  Result Fresh(const mcout::Frame& frame,
               const FlightControlConfig& config,
               bool servo_faulted = false) {
    return dut.Update(true, true, frame, 0.01f, ++frame_count,
                      servo_faulted, config, &status);
  }

  Result Stale(const mcout::Frame& frame,
               const FlightControlConfig& config,
               float age_s) {
    return dut.Update(true, true, frame, age_s, frame_count,
                      false, config, &status);
  }
};

}  // namespace

BOOST_AUTO_TEST_CASE(FlightMapBasic) {
  const auto config = MakeConfig();
  auto frame = MakeArmedFrame();

  {
    // Throttle only: base torque scales, no cyclic.
    const auto setpoint = FlightControlLogic::Map(frame, config);
    BOOST_TEST(setpoint.base_torque_Nm == 1.0f, tt::tolerance(1e-6f));
    BOOST_TEST(setpoint.cyclic_torque_Nm == 0.0f, tt::tolerance(1e-6f));
    BOOST_TEST(setpoint.cyclic_phase == 0.0f, tt::tolerance(1e-6f));
  }

  {
    // Zero throttle: the idle floor applies.
    frame.throttle = 0.0f;
    const auto setpoint = FlightControlLogic::Map(frame, config);
    BOOST_TEST(setpoint.base_torque_Nm == 0.1f, tt::tolerance(1e-6f));
  }

  {
    // Pure +pitch: phase 0.
    frame.roll = 0.0f;
    frame.pitch = 0.5f;
    const auto setpoint = FlightControlLogic::Map(frame, config);
    BOOST_TEST(setpoint.cyclic_torque_Nm == 0.5f, tt::tolerance(1e-6f));
    BOOST_TEST(setpoint.cyclic_phase == 0.0f, tt::tolerance(1e-4f));
  }

  {
    // Pure +roll: phase 0.25.
    frame.roll = 0.5f;
    frame.pitch = 0.0f;
    const auto setpoint = FlightControlLogic::Map(frame, config);
    BOOST_TEST(setpoint.cyclic_torque_Nm == 0.5f, tt::tolerance(1e-6f));
    BOOST_TEST(setpoint.cyclic_phase == 0.25f, tt::tolerance(1e-4f));
  }

  {
    // Pure -pitch: phase +/- 0.5.
    frame.roll = 0.0f;
    frame.pitch = -0.5f;
    const auto setpoint = FlightControlLogic::Map(frame, config);
    BOOST_TEST(std::abs(setpoint.cyclic_phase) == 0.5f, tt::tolerance(1e-4f));
  }

  {
    // Pure -roll: phase -0.25.
    frame.roll = -0.5f;
    frame.pitch = 0.0f;
    const auto setpoint = FlightControlLogic::Map(frame, config);
    BOOST_TEST(setpoint.cyclic_phase == -0.25f, tt::tolerance(1e-4f));
  }

  {
    // 3-4-5 vector: magnitude sets amplitude, direction sets phase.
    frame.roll = 0.6f;
    frame.pitch = 0.8f;
    const auto setpoint = FlightControlLogic::Map(frame, config);
    BOOST_TEST(setpoint.cyclic_torque_Nm == 1.0f, tt::tolerance(1e-6f));
    BOOST_TEST(setpoint.cyclic_phase == 0.10242f, tt::tolerance(1e-4f));
  }

  {
    // The configured phase offset shifts the result.
    auto offset_config = config;
    offset_config.cyclic_phase_offset = 0.125f;
    frame.roll = 0.5f;
    frame.pitch = 0.0f;
    const auto setpoint = FlightControlLogic::Map(frame, offset_config);
    BOOST_TEST(setpoint.cyclic_phase == 0.375f, tt::tolerance(1e-4f));
  }
}

BOOST_AUTO_TEST_CASE(FlightMapClampsAndSanitizes) {
  const auto config = MakeConfig();
  auto frame = MakeArmedFrame();

  {
    // Out of range throttle clamps to [0, 1].
    frame.throttle = 2.0f;
    auto setpoint = FlightControlLogic::Map(frame, config);
    BOOST_TEST(setpoint.base_torque_Nm == 2.0f, tt::tolerance(1e-6f));

    frame.throttle = -1.0f;
    setpoint = FlightControlLogic::Map(frame, config);
    BOOST_TEST(setpoint.base_torque_Nm == 0.1f, tt::tolerance(1e-6f));
  }

  {
    // Roll/pitch magnitude saturates at 1.
    frame.throttle = 0.5f;
    frame.roll = 3.0f;
    frame.pitch = 4.0f;
    const auto setpoint = FlightControlLogic::Map(frame, config);
    BOOST_TEST(setpoint.cyclic_torque_Nm == 1.0f, tt::tolerance(1e-6f));
  }

  {
    // Non-finite wire values degrade to zero command.
    frame.throttle = NaN;
    frame.roll = NaN;
    frame.pitch = kInf;
    const auto setpoint = FlightControlLogic::Map(frame, config);
    BOOST_TEST(setpoint.base_torque_Nm == 0.1f, tt::tolerance(1e-6f));
    BOOST_TEST(setpoint.cyclic_torque_Nm == 0.0f, tt::tolerance(1e-6f));
  }
}

BOOST_AUTO_TEST_CASE(FlightThrottleModes) {
  auto config = MakeConfig();
  auto frame = MakeArmedFrame();

  {
    // Velocity mode (the default): throttle scales the velocity
    // target.
    config.throttle_max_velocity = 40.0f;
    config.idle_velocity = 5.0f;
    auto setpoint = FlightControlLogic::Map(frame, config);
    BOOST_TEST(setpoint.velocity == 20.0f, tt::tolerance(1e-6f));

    // The idle floor applies at zero throttle.
    frame.throttle = 0.0f;
    setpoint = FlightControlLogic::Map(frame, config);
    BOOST_TEST(setpoint.velocity == 5.0f, tt::tolerance(1e-6f));

    // Out of range throttle clamps.
    frame.throttle = 2.0f;
    setpoint = FlightControlLogic::Map(frame, config);
    BOOST_TEST(setpoint.velocity == 40.0f, tt::tolerance(1e-6f));

    // A non-finite wire throttle degrades to the idle floor.
    frame.throttle = NaN;
    setpoint = FlightControlLogic::Map(frame, config);
    BOOST_TEST(setpoint.velocity == 5.0f, tt::tolerance(1e-6f));
  }

  {
    // Torque mode: the fixed velocity config applies instead, and
    // the torque mapping is unchanged.
    config.throttle_mode = kThrottleTorque;
    config.velocity = 7.0f;
    frame.throttle = 0.5f;
    const auto setpoint = FlightControlLogic::Map(frame, config);
    BOOST_TEST(setpoint.velocity == 7.0f, tt::tolerance(1e-6f));
    BOOST_TEST(setpoint.base_torque_Nm == 1.0f, tt::tolerance(1e-6f));
  }

  {
    // The mapped velocity reaches the emitted command and the
    // telemetry mirror.
    Harness harness;
    config.throttle_mode = kThrottleVelocity;
    config.throttle_max_velocity = 40.0f;
    config.idle_velocity = 5.0f;
    frame.throttle = 0.5f;
    const auto result = harness.Fresh(frame, config);
    BOOST_TEST(result.action == Result::kCommand);
    BOOST_TEST(result.command.velocity == 20.0f, tt::tolerance(1e-6f));
    BOOST_TEST(result.command.kd_scale == 20.0f);
    BOOST_TEST(harness.status.velocity_cmd == 20.0f, tt::tolerance(1e-6f));
  }
}

BOOST_AUTO_TEST_CASE(FlightGating) {
  Harness harness;
  auto config = MakeConfig();
  const auto armed = MakeArmedFrame();

  {
    // Disabled: nothing happens.
    config.enable = false;
    const auto result = harness.Fresh(armed, config);
    BOOST_TEST(result.action == Result::kNone);
    BOOST_TEST(harness.status.state == kFlightDisabled);
    BOOST_TEST(harness.status.output_active == false);
    config.enable = true;
  }

  {
    // No receiver configured: disabled.
    const auto result = harness.dut.Update(
        false, true, armed, 0.01f, ++harness.frame_count,
        false, config, &harness.status);
    BOOST_TEST(result.action == Result::kNone);
    BOOST_TEST(harness.status.state == kFlightDisabled);
  }

  {
    // No frame yet: no link.
    const auto result = harness.dut.Update(
        true, false, armed, kInf, 0,
        false, config, &harness.status);
    BOOST_TEST(result.action == Result::kNone);
    BOOST_TEST(harness.status.state == kFlightNoLink);
    BOOST_TEST(harness.status.link_up == false);
  }

  {
    // Fresh but disarmed: standby, still nothing sent.
    mcout::Frame disarmed = armed;
    disarmed.flags = 0;
    const auto result = harness.Fresh(disarmed, config);
    BOOST_TEST(result.action == Result::kNone);
    BOOST_TEST(harness.status.state == kFlightStandby);
    BOOST_TEST(harness.status.link_up == true);
  }

  {
    // Armed and fresh: a position command with cyclic terms.
    mcout::Frame commanding = armed;
    commanding.roll = 0.6f;
    commanding.pitch = 0.8f;
    const auto result = harness.Fresh(commanding, config);
    BOOST_TEST(result.action == Result::kCommand);
    BOOST_TEST(harness.status.state == kFlightActive);
    BOOST_TEST(harness.status.output_active == true);

    const auto& command = result.command;
    BOOST_TEST(command.mode == kPosition);
    BOOST_TEST(std::isnan(command.position));
    // Velocity mode is the default, but with the velocity scales
    // unconfigured the target is 0.
    BOOST_TEST(command.velocity == 0.0f);
    BOOST_TEST(command.kp_scale == 0.0f);
    BOOST_TEST(command.kd_scale == 20.0f);
    BOOST_TEST(command.ilimit_scale == 1.0f);
    BOOST_TEST(std::isnan(command.accel_limit));
    BOOST_TEST(std::isnan(command.velocity_limit));
    BOOST_TEST(command.feedforward_Nm == 1.0f, tt::tolerance(1e-6f));
    BOOST_TEST(command.cyclic_torque_Nm == 1.0f, tt::tolerance(1e-6f));
    BOOST_TEST(command.cyclic_phase == 0.10242f, tt::tolerance(1e-4f));
    // The forwarded torque limit and the servo-side watchdog come
    // from their dedicated config values.
    BOOST_TEST(command.max_torque_Nm == 3.0f);
    BOOST_TEST(command.timeout_s == 0.05f, tt::tolerance(1e-6f));

    BOOST_TEST(harness.status.base_torque_Nm == 1.0f, tt::tolerance(1e-6f));
    BOOST_TEST(harness.status.cyclic_torque_Nm == 1.0f, tt::tolerance(1e-6f));
  }

  {
    // Stale link while engaged: exactly one stop, then silence.
    const auto first = harness.Stale(armed, config, 0.5f);
    BOOST_TEST(first.action == Result::kStop);
    BOOST_TEST(first.command.mode == kStopped);
    BOOST_TEST(harness.status.state == kFlightNoLink);
    BOOST_TEST(harness.status.output_active == false);

    const auto second = harness.Stale(armed, config, 0.6f);
    BOOST_TEST(second.action == Result::kNone);
  }

  {
    // Fresh again: re-engages.
    const auto result = harness.Fresh(armed, config);
    BOOST_TEST(result.action == Result::kCommand);
  }

  {
    // Emergency stop flag: one stop, then silence.
    mcout::Frame estop = armed;
    estop.flags = armed.flags | mcout::Frame::kEmergencyStop;
    const auto first = harness.Fresh(estop, config);
    BOOST_TEST(first.action == Result::kStop);
    BOOST_TEST(harness.status.state == kFlightStandby);

    const auto second = harness.Fresh(estop, config);
    BOOST_TEST(second.action == Result::kNone);
  }

  {
    // An unconfigured torque limit (default 0) is forwarded as-is,
    // leaving the servo torqueless until deliberately configured.
    auto zero_config = config;
    zero_config.max_torque_Nm = 0.0f;
    const auto result = harness.Fresh(armed, zero_config);
    BOOST_TEST(result.action == Result::kCommand);
    BOOST_TEST(result.command.max_torque_Nm == 0.0f);
  }

  {
    // Disabling while engaged emits a final stop.
    config.enable = false;
    const auto result = harness.Fresh(armed, config);
    BOOST_TEST(result.action == Result::kStop);
    BOOST_TEST(harness.status.state == kFlightDisabled);
  }
}

BOOST_AUTO_TEST_CASE(FlightEdgeTriggeredCommands) {
  Harness harness;
  const auto config = MakeConfig();
  const auto armed = MakeArmedFrame();

  // The first fresh armed frame engages and commands.
  BOOST_TEST(harness.Fresh(armed, config).action == Result::kCommand);

  // Re-running with the same frame count sends nothing new; the
  // servo-side watchdog covers the gap.
  const auto repeat = harness.dut.Update(
      true, true, armed, 0.011f, harness.frame_count,
      false, config, &harness.status);
  BOOST_TEST(repeat.action == Result::kNone);
  BOOST_TEST(harness.status.state == kFlightActive);
  BOOST_TEST(harness.status.output_active == true);

  // A new frame produces a new command.
  BOOST_TEST(harness.Fresh(armed, config).action == Result::kCommand);

  // After a disengage/re-engage cycle, the first frame always
  // commands even if the counter did not advance in between.
  BOOST_TEST(harness.Stale(armed, config, 1.0f).action == Result::kStop);
  const auto reengage = harness.dut.Update(
      true, true, armed, 0.01f, harness.frame_count,
      false, config, &harness.status);
  BOOST_TEST(reengage.action == Result::kCommand);
}

BOOST_AUTO_TEST_CASE(FlightFlagMasks) {
  Harness harness;
  auto config = MakeConfig();
  const auto armed = MakeArmedFrame();

  {
    // Default masks: any missing required flag denies output.
    mcout::Frame no_interlock = armed;
    no_interlock.flags = armed.flags & ~mcout::Frame::kInterlock;
    BOOST_TEST(harness.Fresh(no_interlock, config).action == Result::kNone);
    BOOST_TEST(harness.status.state == kFlightStandby);
  }

  {
    // Loosened mask: an interlock-less bench setup can engage.
    auto bench_config = config;
    bench_config.required_flags =
        mcout::Frame::kMotorsArmed | mcout::Frame::kOutputEnabled;
    mcout::Frame no_interlock = armed;
    no_interlock.flags = armed.flags & ~mcout::Frame::kInterlock;
    BOOST_TEST(harness.Fresh(no_interlock, bench_config).action ==
               Result::kCommand);
    // Restore a disengaged state for the next block.
    harness.Stale(no_interlock, bench_config, 1.0f);
  }

  {
    // Tightened disallow mask: spool-up block now denies output.
    auto strict_config = config;
    strict_config.disallow_flags =
        mcout::Frame::kEmergencyStop | mcout::Frame::kSpoolupBlock;
    mcout::Frame blocked = armed;
    blocked.flags = armed.flags | mcout::Frame::kSpoolupBlock;
    BOOST_TEST(harness.Fresh(blocked, strict_config).action == Result::kNone);
    BOOST_TEST(harness.status.state == kFlightStandby);

    BOOST_TEST(harness.Fresh(armed, strict_config).action == Result::kCommand);
  }
}

BOOST_AUTO_TEST_CASE(FlightFaultClearOnEngage) {
  Harness harness;
  const auto config = MakeConfig();
  const auto armed = MakeArmedFrame();

  // ArduPilot commands output while the servo is latched in a fault:
  // the first action is a stop (which clears the fault), engagement
  // follows on the next update.
  const auto first = harness.Fresh(armed, config, true);
  BOOST_TEST(first.action == Result::kStop);
  BOOST_TEST(harness.status.state == kFlightStandby);
  BOOST_TEST(harness.status.output_active == false);

  const auto second = harness.Fresh(armed, config, false);
  BOOST_TEST(second.action == Result::kCommand);
  BOOST_TEST(harness.status.state == kFlightActive);

  // While already engaged, a servo fault does not trigger the
  // clearing behavior; commands continue and the servo ignores them
  // until the operator intervenes or ArduPilot disarms and re-arms.
  const auto third = harness.Fresh(armed, config, true);
  BOOST_TEST(third.action == Result::kCommand);
}
