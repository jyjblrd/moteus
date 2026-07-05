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

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "mjlib/base/visitor.h"

#include "fw/bldc_servo_structs.h"
#include "fw/mcout_protocol.h"

/// @file
///
/// Configuration, telemetry, and the pure decision/mapping logic for
/// driving an underactuated propeller from an ArduPilot MCOUT
/// stream.  Everything in this file is hardware independent and unit
/// tested on the host; the target-side glue lives in
/// fw/flight_control.h.

namespace moteus {

struct FlightControlConfig {
  // Master enable.  When false, the MCOUT stream is still parsed and
  // reported in telemetry, but the servo is never commanded.
  bool enable = false;

  // Base (collective) torque in Nm applied at throttle == 1.  The
  // commanded base torque is throttle * throttle_max_torque_Nm.
  float throttle_max_torque_Nm = 0.0f;

  // Minimum base torque in Nm while output is engaged, e.g. to keep
  // the propeller turning at ground idle when ArduPilot commands
  // zero throttle.
  float idle_torque_Nm = 0.0f;

  // Cyclic torque amplitude in Nm for a unit magnitude roll/pitch
  // command.  The commanded amplitude is
  // min(1, sqrt(roll^2 + pitch^2)) * cyclic_max_torque_Nm.
  float cyclic_max_torque_Nm = 0.0f;

  // Added to the commanded cyclic phase, in cycles.  Use this to
  // align the flight controller's roll/pitch frame with the
  // propeller's physical zero (servo.cyclic.zero_angle) and to
  // compensate phase lag from rotor dynamics.
  float cyclic_phase_offset = 0.0f;

  // The servo is stopped when the newest valid packet is older than
  // this many seconds.  Also used as the servo-side watchdog for
  // each forwarded command.
  float timeout_s = 0.1f;

  // Per-command torque limit forwarded to the servo.  NaN uses the
  // BldcServoCommandData default.
  float max_torque_Nm = std::numeric_limits<float>::quiet_NaN();

  // Optional closed-loop terms, mirroring the position-mode command
  // scales.  The defaults give pure feedforward torque control.  A
  // nonzero kd_scale adds velocity damping about `velocity`, which
  // matches the bench-test regime used during development.
  float kp_scale = 0.0f;
  float kd_scale = 0.0f;
  float velocity = 0.0f;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(enable));
    a->Visit(MJ_NVP(throttle_max_torque_Nm));
    a->Visit(MJ_NVP(idle_torque_Nm));
    a->Visit(MJ_NVP(cyclic_max_torque_Nm));
    a->Visit(MJ_NVP(cyclic_phase_offset));
    a->Visit(MJ_NVP(timeout_s));
    a->Visit(MJ_NVP(max_torque_Nm));
    a->Visit(MJ_NVP(kp_scale));
    a->Visit(MJ_NVP(kd_scale));
    a->Visit(MJ_NVP(velocity));
  }
};

enum FlightControlState : uint8_t {
  // mcout.enable is false, or no AUX UART is configured for mcout.
  kFlightDisabled = 0,
  // Enabled, but no fresh valid packets are arriving.
  kFlightNoLink = 1,
  // Fresh packets are arriving but ArduPilot does not allow output
  // (disarmed, interlock off, emergency stop, ...).
  kFlightStandby = 2,
  // Output is engaged and servo commands are being sent.
  kFlightActive = 3,

  kNumFlightControlStates,
};

struct FlightControlStatus {
  FlightControlState state = kFlightDisabled;

  // Fresh valid packets are arriving.
  bool link_up = false;
  // The output gate is passed and the servo is being commanded.
  bool output_active = false;

  // Link statistics, mirrored from the parser.
  uint32_t valid_frames = 0;
  uint32_t crc_errors = 0;
  uint32_t header_errors = 0;
  uint32_t missed_frames = 0;

  // Age of the newest valid frame, in seconds, saturated at 999.
  float frame_age_s = 999.0f;

  // Selected fields of the newest valid frame.
  uint32_t sequence = 0;
  uint16_t flags = 0;
  uint8_t spool_state = 0;
  uint8_t desired_spool_state = 0;
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  float throttle = 0.0f;

  // The most recently commanded mapping outputs.
  float base_torque_Nm = 0.0f;
  float cyclic_torque_Nm = 0.0f;
  float cyclic_phase = 0.0f;

  // Number of position commands and stop commands emitted.
  uint32_t commands_sent = 0;
  uint32_t stops_sent = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(state));
    a->Visit(MJ_NVP(link_up));
    a->Visit(MJ_NVP(output_active));
    a->Visit(MJ_NVP(valid_frames));
    a->Visit(MJ_NVP(crc_errors));
    a->Visit(MJ_NVP(header_errors));
    a->Visit(MJ_NVP(missed_frames));
    a->Visit(MJ_NVP(frame_age_s));
    a->Visit(MJ_NVP(sequence));
    a->Visit(MJ_NVP(flags));
    a->Visit(MJ_NVP(spool_state));
    a->Visit(MJ_NVP(desired_spool_state));
    a->Visit(MJ_NVP(roll));
    a->Visit(MJ_NVP(pitch));
    a->Visit(MJ_NVP(yaw));
    a->Visit(MJ_NVP(throttle));
    a->Visit(MJ_NVP(base_torque_Nm));
    a->Visit(MJ_NVP(cyclic_torque_Nm));
    a->Visit(MJ_NVP(cyclic_phase));
    a->Visit(MJ_NVP(commands_sent));
    a->Visit(MJ_NVP(stops_sent));
  }
};

/// The hardware independent portion of FlightControl: decides when
/// the servo may be driven and maps MCOUT commands into a moteus
/// position-mode command with cyclic torque.
///
/// Gating follows the MCOUT receiver requirements: fresh packets,
/// MOTORS_ARMED, SOFT_ARMED, INTERLOCK and OUTPUT_ENABLED present,
/// EMERGENCY_STOP clear.  When the gate closes for any reason a
/// single stop command is emitted; afterwards the servo is released
/// so that CAN or diagnostic bench control still works while
/// disarmed.
class FlightControlLogic {
 public:
  struct Result {
    enum Action : uint8_t {
      // Leave the servo alone.
      kNone = 0,
      // Send `command` (a kPosition command with cyclic torque).
      kCommand = 1,
      // Send `command` (a kStopped command).
      kStop = 2,
    };

    Action action = kNone;
    BldcServoCommandData command;
  };

  struct Setpoint {
    float base_torque_Nm = 0.0f;
    float cyclic_torque_Nm = 0.0f;
    float cyclic_phase = 0.0f;
  };

  /// Map normalized MCOUT commands to torque setpoints.
  ///
  /// The roll/pitch vector is interpreted in the moteus reference
  /// frame.  Phase convention: a pure +pitch command gives phase 0, a
  /// pure +roll command gives phase 0.25, i.e. phase =
  /// atan2(roll, pitch) in cycles, plus mcout.cyclic_phase_offset.
  /// Physical alignment is tuned with cyclic_phase_offset together
  /// with servo.cyclic.zero_angle.
  static Setpoint Map(const mcout::Frame& frame,
                      const FlightControlConfig& config) {
    Setpoint result;

    const float throttle =
        Limit(SanitizeInput(frame.throttle), 0.0f, 1.0f);
    result.base_torque_Nm = std::max(
        config.idle_torque_Nm,
        throttle * config.throttle_max_torque_Nm);

    const float roll = SanitizeInput(frame.roll);
    const float pitch = SanitizeInput(frame.pitch);
    const float magnitude = std::sqrt(roll * roll + pitch * pitch);

    result.cyclic_torque_Nm =
        std::min(magnitude, 1.0f) * config.cyclic_max_torque_Nm;
    result.cyclic_phase =
        ((magnitude > 1e-6f) ?
         (std::atan2(roll, pitch) * k1OverTwoPi) : 0.0f) +
        config.cyclic_phase_offset;

    return result;
  }

  /// Advance the gating state machine and produce at most one servo
  /// action for this update cycle.
  ///
  /// @p receiver_present  an AUX UART is configured in mcout mode
  /// @p have_frame        at least one valid frame was ever received
  /// @p frame             the newest valid frame (ignored if !have_frame)
  /// @p frame_age_s       age of that frame in seconds
  /// @p servo_faulted     the servo is currently in the fault state
  ///
  /// Only the state/gate/mapping fields of @p status are written
  /// here; link statistics are maintained by the caller.
  Result Update(bool receiver_present,
                bool have_frame,
                const mcout::Frame& frame,
                float frame_age_s,
                bool servo_faulted,
                const FlightControlConfig& config,
                FlightControlStatus* status) {
    if (!config.enable || !receiver_present) {
      status->link_up = false;
      return Disengage(kFlightDisabled, status);
    }

    const bool fresh =
        have_frame &&
        std::isfinite(frame_age_s) &&
        frame_age_s <= config.timeout_s;
    status->link_up = fresh;

    if (!fresh) {
      return Disengage(kFlightNoLink, status);
    }
    if (!frame.output_allowed()) {
      return Disengage(kFlightStandby, status);
    }

    // The gate is passed.
    Result result;

    if (!engaged_ && servo_faulted) {
      // ArduPilot has just commanded output while the servo is
      // latched in a fault.  Arming is an explicit operator action,
      // so treat it as a request to clear the fault; engagement then
      // proceeds on the next update.
      status->state = kFlightStandby;
      status->output_active = false;
      result.action = Result::kStop;
      result.command.mode = kStopped;
      return result;
    }

    engaged_ = true;
    status->state = kFlightActive;
    status->output_active = true;

    const Setpoint setpoint = Map(frame, config);
    status->base_torque_Nm = setpoint.base_torque_Nm;
    status->cyclic_torque_Nm = setpoint.cyclic_torque_Nm;
    status->cyclic_phase = setpoint.cyclic_phase;

    auto& command = result.command;
    command.mode = kPosition;
    command.position = std::numeric_limits<float>::quiet_NaN();
    command.velocity = config.velocity;
    command.kp_scale = config.kp_scale;
    command.kd_scale = config.kd_scale;
    command.feedforward_Nm = setpoint.base_torque_Nm;
    command.cyclic_torque_Nm = setpoint.cyclic_torque_Nm;
    command.cyclic_phase = setpoint.cyclic_phase;
    if (!std::isnan(config.max_torque_Nm)) {
      command.max_torque_Nm = config.max_torque_Nm;
    }
    command.timeout_s =
        (config.timeout_s > 0.0f) ? config.timeout_s : 0.0f;

    result.action = Result::kCommand;
    return result;
  }

 private:
  static constexpr float k1OverTwoPi = 0.15915494309189535f;

  static float SanitizeInput(float value) {
    return std::isfinite(value) ? value : 0.0f;
  }

  static float Limit(float value, float min, float max) {
    return std::min(max, std::max(min, value));
  }

  Result Disengage(FlightControlState state, FlightControlStatus* status) {
    status->state = state;
    status->output_active = false;

    Result result;
    if (engaged_) {
      engaged_ = false;
      result.action = Result::kStop;
      result.command.mode = kStopped;
    }
    return result;
  }

  bool engaged_ = false;
};

}  // namespace moteus

namespace mjlib {
namespace base {

template <>
struct IsEnum<moteus::FlightControlState> {
  static constexpr bool value = true;

  using S = moteus::FlightControlState;
  static std::array<std::pair<S, const char*>,
                    S::kNumFlightControlStates> map() {
    return { {
        { S::kFlightDisabled, "disabled" },
        { S::kFlightNoLink, "no_link" },
        { S::kFlightStandby, "standby" },
        { S::kFlightActive, "active" },
      }};
  }
};

}  // namespace base
}  // namespace mjlib
