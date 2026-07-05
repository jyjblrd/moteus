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

#include <cstddef>
#include <cstdint>
#include <cstring>

/// @file
///
/// Receive side of the ArduPilot AP_MotorControlOutput ("MCOUT")
/// serial protocol.  ArduPilot publishes one fixed-size binary packet
/// per motor update (up to 400Hz, default 921600 baud) containing the
/// normalized attitude commands, throttle, arming/interlock state and
/// spool state.  moteus consumes this stream to drive an
/// underactuated propeller with angle-synchronous "cyclic" torque.
///
/// This header is dependency free and compiles on the host for unit
/// testing.  Decoding assumes a little-endian machine, like the rest
/// of this codebase.

namespace moteus {
namespace mcout {

/// XMODEM CRC-16 (polynomial 0x1021, initial value 0), bit-for-bit
/// compatible with ArduPilot's crc_xmodem.
inline constexpr uint16_t Crc16XmodemUpdate(uint16_t crc, uint8_t byte) {
  crc ^= static_cast<uint16_t>(static_cast<uint16_t>(byte) << 8);
  for (int i = 0; i < 8; i++) {
    crc = (crc & 0x8000u) ?
        static_cast<uint16_t>((crc << 1) ^ 0x1021u) :
        static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

inline constexpr uint16_t Crc16Xmodem(const uint8_t* data, size_t size) {
  uint16_t crc = 0;
  for (size_t i = 0; i < size; i++) {
    crc = Crc16XmodemUpdate(crc, data[i]);
  }
  return crc;
}

// Byte offsets of each field within the wire packet.
enum Offset : size_t {
  kOffsetMagic = 0,
  kOffsetVersion = 2,
  kOffsetLength = 3,
  kOffsetMessageId = 4,
  kOffsetSequence = 6,
  kOffsetTimestamp = 10,
  kOffsetFlags = 18,
  kOffsetSpoolState = 20,
  kOffsetDesiredSpoolState = 21,
  kOffsetRoll = 22,
  kOffsetPitch = 26,
  kOffsetYaw = 30,
  kOffsetThrottle = 34,
  kOffsetRollFeedback = 38,
  kOffsetPitchFeedback = 42,
  kOffsetYawFeedback = 46,
  kOffsetRollFeedforward = 50,
  kOffsetPitchFeedforward = 54,
  kOffsetYawFeedforward = 58,
  kOffsetThrottleFiltered = 62,
  kOffsetThrottleOut = 66,
  kOffsetForward = 70,
  kOffsetLateral = 74,
  kOffsetDt = 78,
  kOffsetCrc = 82,
};

/// A decoded MCOUT control packet.
struct Frame {
  static constexpr uint16_t kMagic = 0xC04D;
  static constexpr uint8_t kMagic0 = 0x4D;  // first byte on the wire
  static constexpr uint8_t kMagic1 = 0xC0;
  static constexpr uint8_t kVersion = 1;
  static constexpr uint8_t kLength = 84;
  static constexpr uint16_t kMessageIdControl = 1;

  // Bits of `flags`, from AP_MotorControlOutput::Flags.
  enum Flags : uint16_t {
    kMotorsArmed        = 1u << 0,
    kSoftArmed          = 1u << 1,
    kInterlock          = 1u << 2,
    kEmergencyStop      = 1u << 3,
    kSpoolupBlock       = 1u << 4,
    kMotorsInitialized  = 1u << 5,
    kOutputEnabled      = 1u << 6,
    kLimitRoll          = 1u << 7,
    kLimitPitch         = 1u << 8,
    kLimitYaw           = 1u << 9,
    kLimitThrottleLower = 1u << 10,
    kLimitThrottleUpper = 1u << 11,
    kThrustBoost        = 1u << 12,
  };

  // Values of `spool_state`.
  enum SpoolState : uint8_t {
    kShutDown = 0,
    kGroundIdle = 1,
    kSpoolingUp = 2,
    kThrottleUnlimited = 3,
    kSpoolingDown = 4,
  };

  uint32_t sequence = 0;
  uint64_t timestamp_usec = 0;
  uint16_t flags = 0;
  uint8_t spool_state = 0;
  uint8_t desired_spool_state = 0;
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  float throttle = 0.0f;
  float roll_feedback = 0.0f;
  float pitch_feedback = 0.0f;
  float yaw_feedback = 0.0f;
  float roll_feedforward = 0.0f;
  float pitch_feedforward = 0.0f;
  float yaw_feedforward = 0.0f;
  float throttle_filtered = 0.0f;
  float throttle_out = 0.0f;
  float forward = 0.0f;
  float lateral = 0.0f;
  float dt = 0.0f;

  /// True when every flag ArduPilot requires for actuator output is
  /// present and emergency stop is clear.  Receivers must combine
  /// this with their own packet freshness check.
  bool output_allowed() const {
    constexpr uint16_t kRequired =
        kMotorsArmed | kSoftArmed | kInterlock | kOutputEnabled;
    return (flags & kRequired) == kRequired &&
        (flags & kEmergencyStop) == 0;
  }
};

/// Incremental MCOUT packet parser.  Feed one byte at a time; a
/// completed, fully validated frame is returned through Feed's out
/// parameter.  Invalid data resynchronizes on the next magic byte.
class Parser {
 public:
  struct Stats {
    // Frames that passed all header checks and the CRC.
    uint32_t valid_frames = 0;
    // Completed frames rejected by CRC mismatch.
    uint32_t crc_errors = 0;
    // Bytes which broke a partially accumulated header/frame.
    uint32_t header_errors = 0;
    // Cumulative gap detected in the `sequence` field.
    uint32_t missed_frames = 0;
  };

  /// Consume one byte.  Returns true if @p frame was just filled
  /// with a newly completed valid frame.
  bool Feed(uint8_t byte, Frame* frame) {
    if (index_ == 0) {
      // Bytes outside a frame are ignored silently: idle line noise
      // or the remainder of a frame we gave up on.
      if (byte == Frame::kMagic0) { Start(byte); }
      return false;
    }

    const bool header_valid =
        (index_ != kOffsetMagic + 1 || byte == Frame::kMagic1) &&
        (index_ != kOffsetVersion || byte == Frame::kVersion) &&
        (index_ != kOffsetLength || byte == Frame::kLength) &&
        (index_ != kOffsetMessageId ||
         byte == (Frame::kMessageIdControl & 0xff)) &&
        (index_ != kOffsetMessageId + 1 ||
         byte == ((Frame::kMessageIdControl >> 8) & 0xff));
    if (!header_valid) {
      stats_.header_errors++;
      index_ = 0;
      crc_ = 0;
      // The offending byte could itself start a new frame.
      if (byte == Frame::kMagic0) { Start(byte); }
      return false;
    }

    buffer_[index_] = byte;
    if (index_ < kOffsetCrc) {
      crc_ = Crc16XmodemUpdate(crc_, byte);
    }
    index_++;
    if (index_ < Frame::kLength) { return false; }

    // A complete frame has been accumulated.
    const uint16_t computed_crc = crc_;
    index_ = 0;
    crc_ = 0;

    if (Get<uint16_t>(kOffsetCrc) != computed_crc) {
      stats_.crc_errors++;
      return false;
    }

    Decode(frame);
    TrackSequence(frame->sequence);
    stats_.valid_frames++;
    return true;
  }

  const Stats& stats() const { return stats_; }

  void Reset() {
    index_ = 0;
    crc_ = 0;
    have_sequence_ = false;
  }

  /// Construct a valid wire packet from @p frame, including the CRC.
  /// Used by unit tests and host-side tooling.
  static void Encode(const Frame& frame, uint8_t out[Frame::kLength]) {
    std::memset(out, 0, Frame::kLength);
    Put<uint16_t>(out, kOffsetMagic, Frame::kMagic);
    out[kOffsetVersion] = Frame::kVersion;
    out[kOffsetLength] = Frame::kLength;
    Put<uint16_t>(out, kOffsetMessageId, Frame::kMessageIdControl);
    Put<uint32_t>(out, kOffsetSequence, frame.sequence);
    Put<uint64_t>(out, kOffsetTimestamp, frame.timestamp_usec);
    Put<uint16_t>(out, kOffsetFlags, frame.flags);
    out[kOffsetSpoolState] = frame.spool_state;
    out[kOffsetDesiredSpoolState] = frame.desired_spool_state;
    Put<float>(out, kOffsetRoll, frame.roll);
    Put<float>(out, kOffsetPitch, frame.pitch);
    Put<float>(out, kOffsetYaw, frame.yaw);
    Put<float>(out, kOffsetThrottle, frame.throttle);
    Put<float>(out, kOffsetRollFeedback, frame.roll_feedback);
    Put<float>(out, kOffsetPitchFeedback, frame.pitch_feedback);
    Put<float>(out, kOffsetYawFeedback, frame.yaw_feedback);
    Put<float>(out, kOffsetRollFeedforward, frame.roll_feedforward);
    Put<float>(out, kOffsetPitchFeedforward, frame.pitch_feedforward);
    Put<float>(out, kOffsetYawFeedforward, frame.yaw_feedforward);
    Put<float>(out, kOffsetThrottleFiltered, frame.throttle_filtered);
    Put<float>(out, kOffsetThrottleOut, frame.throttle_out);
    Put<float>(out, kOffsetForward, frame.forward);
    Put<float>(out, kOffsetLateral, frame.lateral);
    Put<float>(out, kOffsetDt, frame.dt);
    Put<uint16_t>(out, kOffsetCrc, Crc16Xmodem(out, kOffsetCrc));
  }

 private:
  void Start(uint8_t byte) {
    buffer_[0] = byte;
    crc_ = Crc16XmodemUpdate(0, byte);
    index_ = 1;
  }

  template <typename T>
  T Get(size_t offset) const {
    T result;
    std::memcpy(&result, &buffer_[offset], sizeof(T));
    return result;
  }

  template <typename T>
  static void Put(uint8_t* buffer, size_t offset, T value) {
    std::memcpy(&buffer[offset], &value, sizeof(T));
  }

  void Decode(Frame* frame) const {
    frame->sequence = Get<uint32_t>(kOffsetSequence);
    frame->timestamp_usec = Get<uint64_t>(kOffsetTimestamp);
    frame->flags = Get<uint16_t>(kOffsetFlags);
    frame->spool_state = buffer_[kOffsetSpoolState];
    frame->desired_spool_state = buffer_[kOffsetDesiredSpoolState];
    frame->roll = Get<float>(kOffsetRoll);
    frame->pitch = Get<float>(kOffsetPitch);
    frame->yaw = Get<float>(kOffsetYaw);
    frame->throttle = Get<float>(kOffsetThrottle);
    frame->roll_feedback = Get<float>(kOffsetRollFeedback);
    frame->pitch_feedback = Get<float>(kOffsetPitchFeedback);
    frame->yaw_feedback = Get<float>(kOffsetYawFeedback);
    frame->roll_feedforward = Get<float>(kOffsetRollFeedforward);
    frame->pitch_feedforward = Get<float>(kOffsetPitchFeedforward);
    frame->yaw_feedforward = Get<float>(kOffsetYawFeedforward);
    frame->throttle_filtered = Get<float>(kOffsetThrottleFiltered);
    frame->throttle_out = Get<float>(kOffsetThrottleOut);
    frame->forward = Get<float>(kOffsetForward);
    frame->lateral = Get<float>(kOffsetLateral);
    frame->dt = Get<float>(kOffsetDt);
  }

  void TrackSequence(uint32_t sequence) {
    if (have_sequence_) {
      const uint32_t expected = last_sequence_ + 1;
      const uint32_t delta = sequence - expected;
      // Ignore implausibly large gaps, e.g. an autopilot reboot.
      if (delta != 0 && delta < 100000u) {
        stats_.missed_frames += delta;
      }
    }
    last_sequence_ = sequence;
    have_sequence_ = true;
  }

  uint8_t buffer_[Frame::kLength] = {};
  size_t index_ = 0;
  uint16_t crc_ = 0;

  uint32_t last_sequence_ = 0;
  bool have_sequence_ = false;

  Stats stats_;
};

}  // namespace mcout
}  // namespace moteus
