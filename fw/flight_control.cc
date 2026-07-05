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

#include "fw/flight_control.h"

#include <algorithm>
#include <limits>

#include "fw/mcout_receiver.h"

namespace moteus {

FlightControl::FlightControl(
    mjlib::micro::PersistentConfig* persistent_config,
    mjlib::micro::TelemetryManager* telemetry_manager,
    MillisecondTimer* timer,
    BldcServo* bldc,
    AuxPort* aux1_port,
    AuxPort* aux2_port)
    : timer_(timer),
      bldc_(bldc),
      aux1_port_(aux1_port),
      aux2_port_(aux2_port) {
  persistent_config->Register("mcout", &config_, []() {});
  status_update_ = telemetry_manager->Register("mcout", &status_);
}

void FlightControl::PollMillisecond() {
  McoutReceiver* receiver = aux1_port_->mcout_receiver();
  if (receiver == nullptr) { receiver = aux2_port_->mcout_receiver(); }

  bool have_frame = false;
  float frame_age_s = std::numeric_limits<float>::infinity();
  mcout::Frame frame;

  if (receiver != nullptr) {
    const auto& stats = receiver->stats();
    status_.valid_frames = stats.valid_frames;
    status_.crc_errors = stats.crc_errors;
    status_.header_errors = stats.header_errors;
    status_.missed_frames = stats.missed_frames;

    have_frame = receiver->have_frame();
    if (have_frame) {
      frame = receiver->frame();

      // Unsigned arithmetic keeps this correct across read_ms()
      // rollover.
      const uint32_t age_ms = timer_->read_ms() - receiver->last_frame_ms();
      frame_age_s = static_cast<float>(age_ms) * 0.001f;

      status_.sequence = frame.sequence;
      status_.flags = frame.flags;
      status_.spool_state = frame.spool_state;
      status_.desired_spool_state = frame.desired_spool_state;
      status_.roll = frame.roll;
      status_.pitch = frame.pitch;
      status_.yaw = frame.yaw;
      status_.throttle = frame.throttle;
    }
  }

  status_.frame_age_s = std::min(frame_age_s, 999.0f);

  const bool servo_faulted = bldc_->status().mode == kFault;

  const auto result = logic_.Update(
      receiver != nullptr, have_frame, frame, frame_age_s,
      status_.valid_frames, servo_faulted, config_, &status_);

  switch (result.action) {
    case FlightControlLogic::Result::kNone: {
      break;
    }
    case FlightControlLogic::Result::kCommand: {
      status_.commands_sent++;
      bldc_->Command(result.command);
      break;
    }
    case FlightControlLogic::Result::kStop: {
      status_.stops_sent++;
      bldc_->Command(result.command);
      break;
    }
  }

  status_update_();
}

}  // namespace moteus
