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

// Selects what the normalized MCOUT throttle commands.
enum FlightThrottleMode : uint8_t {
  // Throttle scales a feedforward torque
  // (mcout.throttle_max_torque_Nm); the velocity target is the fixed
  // mcout.velocity.  Set kd_scale to 0 for pure feedforward torque.
  kThrottleTorque = 0,

  // Throttle scales the velocity target
  // (mcout.throttle_max_velocity) and kd_scale provides the
  // tracking, matching the bench-test scripts.  Any throttle-derived
  // base torque is still applied as a feedforward assist.
  kThrottleVelocity = 1,

  kNumFlightThrottleModes,
};

struct FlightControlConfig {
  // Master enable.  When false, the MCOUT stream is still parsed and
  // reported in telemetry, but the servo is never commanded.
  bool enable = false;

  FlightThrottleMode throttle_mode = kThrottleVelocity;

  // Velocity target in output rev/s at throttle == 1 (velocity
  // mode).  The commanded velocity is
  // max(idle_velocity, throttle * throttle_max_velocity).
  float throttle_max_velocity = 0.0f;

  // Minimum velocity target while output is engaged (velocity
  // mode), e.g. ground idle spin.
  float idle_velocity = 0.0f;

  // Base (collective) torque in Nm applied at throttle == 1.  The
  // commanded base torque is throttle * throttle_max_torque_Nm.  In
  // velocity mode this acts as an optional feedforward assist for
  // the velocity tracking.
  float throttle_max_torque_Nm = 0.0f;

  // Minimum base torque in Nm while output is engaged, e.g. to keep
  // the propeller turning at ground idle when ArduPilot commands
  // zero throttle.
  float idle_torque_Nm = 0.0f;

  // Cyclic torque amplitude in Nm for a unit magnitude roll/pitch
  // command.  Without the speed schedule below, the commanded
  // amplitude is min(1, sqrt(roll^2 + pitch^2)) *
  // cyclic_max_torque_Nm.  With the schedule enabled, this is the
  // high-speed endpoint.
  float cyclic_max_torque_Nm = 0.0f;

  // Optional linear speed schedule for full-stick cyclic torque.  The
  // schedule is disabled unless cyclic_schedule_high_speed is greater
  // than cyclic_schedule_low_speed.  Speeds are absolute output rev/s.
  float cyclic_schedule_low_speed = 0.0f;
  float cyclic_schedule_high_speed = 0.0f;
  float cyclic_schedule_low_torque_Nm = 0.0f;

  // Added to the commanded cyclic phase, in cycles.  Use this to
  // align the flight controller's roll/pitch frame with the
  // propeller's physical zero (servo.cyclic.zero_angle) and to
  // compensate phase lag from rotor dynamics.
  float cyclic_phase_offset = 0.0f;

  // Optional signed phase advance (cycles) added at high rotor speed.
  // Disabled unless high_speed > low_speed. A signed advance avoids
  // ambiguities at the modulo-one phase boundary (e.g. 0.49 -> 0.51).
  // Calibrate with measured response; this does not identify phase online.
  float cyclic_phase_low_speed = 0.0f;
  float cyclic_phase_high_speed = 0.0f;
  float cyclic_phase_high_advance = 0.0f;

  // Invert MCOUT command axes before cyclic mapping.
  bool invert_roll = false;
  bool invert_pitch = false;
  bool invert_yaw = false;

  // The servo is stopped when the newest valid packet is older than
  // this many seconds.
  float timeout_s = 0.1f;

  // Servo-side watchdog applied to each forwarded command; it is the
  // backstop if this subsystem itself stops running.
  float watchdog_timeout_s = 0.1f;

  // Per-command torque limit forwarded to the servo.  The default of
  // 0 keeps the servo torqueless until deliberately configured.
  float max_torque_Nm = 0.0f;

  // MCOUT flag bits which must all be present, and which must all be
  // absent, for output to engage.  The defaults implement the MCOUT
  // receiver requirements; loosen or tighten only with care.
  uint16_t required_flags =
      mcout::Frame::kMotorsArmed |
      mcout::Frame::kSoftArmed |
      mcout::Frame::kInterlock |
      mcout::Frame::kOutputEnabled;
  uint16_t disallow_flags = mcout::Frame::kEmergencyStop;

  // Closed-loop terms, mirroring the position-mode command scales
  // and limits.  kp is kept at 0 so the position loop does not fight
  // the cyclic torque; kd provides the velocity tracking.  The
  // kd_scale default matches the bench-test scripts.  For pure
  // feedforward torque control set kd_scale to 0.
  float kp_scale = 0.0f;
  float kd_scale = 20.0f;
  float ilimit_scale = 1.0f;
  // The fixed velocity target used in torque mode (typically 0);
  // ignored in velocity mode.
  float velocity = 0.0f;
  // The acceleration limit shapes velocity target changes, i.e. the
  // spool-up ramp.  NaN uses servo.default_accel_limit.
  float accel_limit = std::numeric_limits<float>::quiet_NaN();
  float velocity_limit = std::numeric_limits<float>::quiet_NaN();

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(enable));
    a->Visit(MJ_NVP(throttle_mode));
    a->Visit(MJ_NVP(throttle_max_velocity));
    a->Visit(MJ_NVP(idle_velocity));
    a->Visit(MJ_NVP(throttle_max_torque_Nm));
    a->Visit(MJ_NVP(idle_torque_Nm));
    a->Visit(MJ_NVP(cyclic_max_torque_Nm));
    a->Visit(MJ_NVP(cyclic_schedule_low_speed));
    a->Visit(MJ_NVP(cyclic_schedule_high_speed));
    a->Visit(MJ_NVP(cyclic_schedule_low_torque_Nm));
    a->Visit(MJ_NVP(cyclic_phase_offset));
    a->Visit(MJ_NVP(cyclic_phase_low_speed));
    a->Visit(MJ_NVP(cyclic_phase_high_speed));
    a->Visit(MJ_NVP(cyclic_phase_high_advance));
    a->Visit(MJ_NVP(invert_roll));
    a->Visit(MJ_NVP(invert_pitch));
    a->Visit(MJ_NVP(invert_yaw));
    a->Visit(MJ_NVP(timeout_s));
    a->Visit(MJ_NVP(watchdog_timeout_s));
    a->Visit(MJ_NVP(max_torque_Nm));
    a->Visit(MJ_NVP(required_flags));
    a->Visit(MJ_NVP(disallow_flags));
    a->Visit(MJ_NVP(kp_scale));
    a->Visit(MJ_NVP(kd_scale));
    a->Visit(MJ_NVP(ilimit_scale));
    a->Visit(MJ_NVP(velocity));
    a->Visit(MJ_NVP(accel_limit));
    a->Visit(MJ_NVP(velocity_limit));
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
  float velocity_cmd = 0.0f;
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
    a->Visit(MJ_NVP(velocity_cmd));
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
    float velocity = 0.0f;
    float base_torque_Nm = 0.0f;
    float cyclic_torque_Nm = 0.0f;
    float cyclic_phase = 0.0f;
  };

  /// Map normalized MCOUT commands to velocity/torque setpoints.
  ///
  /// The roll/pitch vector is interpreted in the moteus reference
  /// frame.  Phase convention: a pure +pitch command gives phase 0, a
  /// pure +roll command gives phase 0.25, i.e. phase =
  /// atan2(roll, pitch) in cycles, plus mcout.cyclic_phase_offset.
  /// Physical alignment is tuned with cyclic_phase_offset together
  /// with servo.cyclic.zero_angle.
  static mcout::Frame ApplyAxisInverts(const mcout::Frame& frame,
                                       const FlightControlConfig& config) {
    auto result = frame;
    result.roll = config.invert_roll ? -result.roll : result.roll;
    result.pitch = config.invert_pitch ? -result.pitch : result.pitch;
    result.yaw = config.invert_yaw ? -result.yaw : result.yaw;
    return result;
  }

  static Setpoint Map(const mcout::Frame& frame,
                      const FlightControlConfig& config,
                      float abs_output_velocity = 0.0f) {
    Setpoint result;
    const auto command_frame = ApplyAxisInverts(frame, config);

    const float throttle =
        Limit(SanitizeInput(command_frame.throttle), 0.0f, 1.0f);

    result.velocity =
        (config.throttle_mode == kThrottleVelocity) ?
        std::max(config.idle_velocity,
                 throttle * config.throttle_max_velocity) :
        config.velocity;

    result.base_torque_Nm = std::max(
        config.idle_torque_Nm,
        throttle * config.throttle_max_torque_Nm);

    const float roll = SanitizeInput(command_frame.roll);
    const float pitch = SanitizeInput(command_frame.pitch);
    const float magnitude = std::sqrt(roll * roll + pitch * pitch);
    const float cyclic_max_torque_Nm =
        CyclicMaxTorque(config, abs_output_velocity);

    result.cyclic_torque_Nm =
        std::min(magnitude, 1.0f) * cyclic_max_torque_Nm;
    result.cyclic_phase =
        ((magnitude > 1e-6f) ?
         (std::atan2(roll, pitch) * k1OverTwoPi) : 0.0f) +
        CyclicPhaseOffset(config, abs_output_velocity);

    return result;
  }

  /// Advance the gating state machine and produce at most one servo
  /// action for this update cycle.
  ///
  /// @p receiver_present  an AUX UART is configured in mcout mode
  /// @p have_frame        at least one valid frame was ever received
  /// @p frame             the newest valid frame (ignored if !have_frame)
  /// @p frame_age_s       age of that frame in seconds
  /// @p frame_count       monotonic count of valid frames received;
  ///                      while engaged, a new command is emitted
  ///                      only when this advances
  /// @p servo_faulted     the servo is currently in the fault state
  /// @p abs_output_velocity  filtered absolute output velocity, in rev/s
  ///
  /// Only the state/gate/mapping fields of @p status are written
  /// here; link statistics are maintained by the caller.
  Result Update(bool receiver_present,
                bool have_frame,
                const mcout::Frame& frame,
                float frame_age_s,
                uint32_t frame_count,
                bool servo_faulted,
                const FlightControlConfig& config,
                FlightControlStatus* status,
                float abs_output_velocity = 0.0f) {
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
    const bool flags_allow =
        ((frame.flags & config.required_flags) == config.required_flags) &&
        ((frame.flags & config.disallow_flags) == 0);
    if (!flags_allow) {
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

    if (have_commanded_ && frame_count == last_frame_count_) {
      // No new data since the last command.  The servo-side watchdog
      // covers us until the next frame or the staleness gate trips.
      return Result();
    }
    have_commanded_ = true;
    last_frame_count_ = frame_count;

    const Setpoint setpoint = Map(frame, config, abs_output_velocity);
    status->velocity_cmd = setpoint.velocity;
    status->base_torque_Nm = setpoint.base_torque_Nm;
    status->cyclic_torque_Nm = setpoint.cyclic_torque_Nm;
    status->cyclic_phase = setpoint.cyclic_phase;

    auto& command = result.command;
    command.mode = kPosition;
    command.position = std::numeric_limits<float>::quiet_NaN();
    command.velocity = setpoint.velocity;
    command.kp_scale = config.kp_scale;
    command.kd_scale = config.kd_scale;
    command.ilimit_scale = config.ilimit_scale;
    command.accel_limit = config.accel_limit;
    command.velocity_limit = config.velocity_limit;
    command.feedforward_Nm = setpoint.base_torque_Nm;
    command.cyclic_torque_Nm = setpoint.cyclic_torque_Nm;
    command.cyclic_phase = setpoint.cyclic_phase;
    command.max_torque_Nm = config.max_torque_Nm;
    command.timeout_s =
        (config.watchdog_timeout_s > 0.0f) ? config.watchdog_timeout_s : 0.0f;

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

  static float CyclicPhaseOffset(const FlightControlConfig& config,
                                 float abs_output_velocity) {
    const float low = config.cyclic_phase_low_speed;
    const float high = config.cyclic_phase_high_speed;
    const float advance = config.cyclic_phase_high_advance;

    // Preserve the original constant-offset behavior if not enabled,
    // including when an optional schedule parameter is malformed.
    if (!(std::isfinite(low) && std::isfinite(high) &&
          std::isfinite(advance) && std::isfinite(abs_output_velocity)) ||
        !(low >= 0.0f && high > low)) {
      return config.cyclic_phase_offset;
    }

    // Evaluate interpolation in double precision to avoid overflowing
    // the speed interval for finite but extreme configuration values.
    const double speed = std::abs(static_cast<double>(abs_output_velocity));
    const double fraction = std::max(
        0.0, std::min(1.0,
        (speed - static_cast<double>(low)) /
        (static_cast<double>(high) - static_cast<double>(low))));
    const double offset = static_cast<double>(config.cyclic_phase_offset) +
                          fraction * static_cast<double>(advance);
    // Invalid or unrepresentable scheduled results must not reach
    // the cyclic phase encoder. Retain the preexisting offset.
    if (!std::isfinite(offset) ||
        std::abs(offset) > static_cast<double>(
            std::numeric_limits<float>::max())) {
      return config.cyclic_phase_offset;
    }
    return static_cast<float>(offset);
  }

  static float CyclicMaxTorque(const FlightControlConfig& config,
                               float abs_output_velocity) {
    if (!(config.cyclic_schedule_high_speed >
          config.cyclic_schedule_low_speed)) {
      return config.cyclic_max_torque_Nm;
    }

    const float low_speed =
        std::max(0.0f, SanitizeInput(config.cyclic_schedule_low_speed));
    const float high_speed =
        std::max(0.0f, SanitizeInput(config.cyclic_schedule_high_speed));
    if (!(high_speed > low_speed)) { return config.cyclic_max_torque_Nm; }

    const float speed = std::abs(SanitizeInput(abs_output_velocity));
    const float t = Limit((speed - low_speed) / (high_speed - low_speed),
                          0.0f, 1.0f);
    const float low_torque =
        std::max(0.0f, SanitizeInput(config.cyclic_schedule_low_torque_Nm));
    const float high_torque =
        std::max(0.0f, SanitizeInput(config.cyclic_max_torque_Nm));
    return low_torque + t * (high_torque - low_torque);
  }

  Result Disengage(FlightControlState state, FlightControlStatus* status) {
    status->state = state;
    status->output_active = false;
    have_commanded_ = false;

    Result result;
    if (engaged_) {
      engaged_ = false;
      result.action = Result::kStop;
      result.command.mode = kStopped;
    }
    return result;
  }

  bool engaged_ = false;
  bool have_commanded_ = false;
  uint32_t last_frame_count_ = 0;
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

template <>
struct IsEnum<moteus::FlightThrottleMode> {
  static constexpr bool value = true;

  using T = moteus::FlightThrottleMode;
  static std::array<std::pair<T, const char*>,
                    T::kNumFlightThrottleModes> map() {
    return { {
        { T::kThrottleTorque, "torque" },
        { T::kThrottleVelocity, "velocity" },
      }};
  }
};

}  // namespace base
}  // namespace mjlib
