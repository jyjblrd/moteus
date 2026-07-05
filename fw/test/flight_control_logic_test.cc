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
  config.timeout_s = 0.1f;
  return config;
}

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

BOOST_AUTO_TEST_CASE(FlightGating) {
  FlightControlLogic dut;
  FlightControlStatus status;
  auto config = MakeConfig();
  const auto armed = MakeArmedFrame();

  {
    // Disabled: nothing happens.
    config.enable = false;
    const auto result = dut.Update(
        true, true, armed, 0.01f, false, config, &status);
    BOOST_TEST(result.action == Result::kNone);
    BOOST_TEST(status.state == kFlightDisabled);
    BOOST_TEST(status.output_active == false);
    config.enable = true;
  }

  {
    // No receiver configured: disabled.
    const auto result = dut.Update(
        false, true, armed, 0.01f, false, config, &status);
    BOOST_TEST(result.action == Result::kNone);
    BOOST_TEST(status.state == kFlightDisabled);
  }

  {
    // No frame yet: no link.
    const auto result = dut.Update(
        true, false, armed, kInf, false, config, &status);
    BOOST_TEST(result.action == Result::kNone);
    BOOST_TEST(status.state == kFlightNoLink);
    BOOST_TEST(status.link_up == false);
  }

  {
    // Fresh but disarmed: standby, still nothing sent.
    mcout::Frame disarmed = armed;
    disarmed.flags = 0;
    const auto result = dut.Update(
        true, true, disarmed, 0.01f, false, config, &status);
    BOOST_TEST(result.action == Result::kNone);
    BOOST_TEST(status.state == kFlightStandby);
    BOOST_TEST(status.link_up == true);
  }

  {
    // Armed and fresh: a position command with cyclic terms.
    mcout::Frame commanding = armed;
    commanding.roll = 0.6f;
    commanding.pitch = 0.8f;
    const auto result = dut.Update(
        true, true, commanding, 0.01f, false, config, &status);
    BOOST_TEST(result.action == Result::kCommand);
    BOOST_TEST(status.state == kFlightActive);
    BOOST_TEST(status.output_active == true);

    const auto& command = result.command;
    BOOST_TEST(command.mode == kPosition);
    BOOST_TEST(std::isnan(command.position));
    BOOST_TEST(command.velocity == 0.0f);
    BOOST_TEST(command.kp_scale == 0.0f);
    BOOST_TEST(command.kd_scale == 0.0f);
    BOOST_TEST(command.feedforward_Nm == 1.0f, tt::tolerance(1e-6f));
    BOOST_TEST(command.cyclic_torque_Nm == 1.0f, tt::tolerance(1e-6f));
    BOOST_TEST(command.cyclic_phase == 0.10242f, tt::tolerance(1e-4f));
    BOOST_TEST(command.timeout_s == 0.1f, tt::tolerance(1e-6f));
    // Default per-command torque limit remains untouched when the
    // config value is NaN.
    BOOST_TEST(command.max_torque_Nm == 100.0f);

    BOOST_TEST(status.base_torque_Nm == 1.0f, tt::tolerance(1e-6f));
    BOOST_TEST(status.cyclic_torque_Nm == 1.0f, tt::tolerance(1e-6f));
  }

  {
    // Stale link while engaged: exactly one stop, then silence.
    const auto first = dut.Update(
        true, true, armed, 0.5f, false, config, &status);
    BOOST_TEST(first.action == Result::kStop);
    BOOST_TEST(first.command.mode == kStopped);
    BOOST_TEST(status.state == kFlightNoLink);
    BOOST_TEST(status.output_active == false);

    const auto second = dut.Update(
        true, true, armed, 0.6f, false, config, &status);
    BOOST_TEST(second.action == Result::kNone);
  }

  {
    // Fresh again: re-engages.
    const auto result = dut.Update(
        true, true, armed, 0.01f, false, config, &status);
    BOOST_TEST(result.action == Result::kCommand);
  }

  {
    // Emergency stop flag: one stop, then silence.
    mcout::Frame estop = armed;
    estop.flags = armed.flags | mcout::Frame::kEmergencyStop;
    const auto first = dut.Update(
        true, true, estop, 0.01f, false, config, &status);
    BOOST_TEST(first.action == Result::kStop);
    BOOST_TEST(status.state == kFlightStandby);

    const auto second = dut.Update(
        true, true, estop, 0.01f, false, config, &status);
    BOOST_TEST(second.action == Result::kNone);
  }

  {
    // Configured torque limit propagates.
    config.max_torque_Nm = 5.0f;
    const auto result = dut.Update(
        true, true, armed, 0.01f, false, config, &status);
    BOOST_TEST(result.action == Result::kCommand);
    BOOST_TEST(result.command.max_torque_Nm == 5.0f);
    config.max_torque_Nm = NaN;
  }

  {
    // Disabling while engaged emits a final stop.
    config.enable = false;
    const auto result = dut.Update(
        true, true, armed, 0.01f, false, config, &status);
    BOOST_TEST(result.action == Result::kStop);
    BOOST_TEST(status.state == kFlightDisabled);
  }
}

BOOST_AUTO_TEST_CASE(FlightFaultClearOnEngage) {
  FlightControlLogic dut;
  FlightControlStatus status;
  const auto config = MakeConfig();
  const auto armed = MakeArmedFrame();

  // ArduPilot commands output while the servo is latched in a fault:
  // the first action is a stop (which clears the fault), engagement
  // follows on the next update.
  const auto first = dut.Update(
      true, true, armed, 0.01f, true, config, &status);
  BOOST_TEST(first.action == Result::kStop);
  BOOST_TEST(status.state == kFlightStandby);
  BOOST_TEST(status.output_active == false);

  const auto second = dut.Update(
      true, true, armed, 0.01f, false, config, &status);
  BOOST_TEST(second.action == Result::kCommand);
  BOOST_TEST(status.state == kFlightActive);

  // While already engaged, a servo fault does not trigger the
  // clearing behavior; commands continue and the servo ignores them
  // until the operator intervenes or ArduPilot disarms and re-arms.
  const auto third = dut.Update(
      true, true, armed, 0.01f, true, config, &status);
  BOOST_TEST(third.action == Result::kCommand);
}
