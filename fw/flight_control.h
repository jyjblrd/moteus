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

#include "mjlib/base/inplace_function.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/telemetry_manager.h"

#include "fw/aux_port.h"
#include "fw/bldc_servo.h"
#include "fw/flight_control_structs.h"
#include "fw/millisecond_timer.h"

namespace moteus {

/// Drives the servo from an ArduPilot AP_MotorControlOutput
/// ("MCOUT") stream received on an AUX UART.
///
/// The AUX UART is selected by configuring `auxN.uart.mode` to
/// `mcout` (aux1 is preferred if both are configured).  When
/// `mcout.enable` is set and ArduPilot commands output (armed,
/// interlock on, no emergency stop, fresh packets), this subsystem
/// sends a position-mode command each millisecond consisting of a
/// throttle-derived base torque plus an angle-synchronous cyclic
/// torque whose amplitude and phase follow the roll/pitch command
/// vector.  See fw/flight_control_structs.h for the mapping and
/// docs/guides/ardupilot-mcout.md for the integration guide.
class FlightControl {
 public:
  FlightControl(mjlib::micro::PersistentConfig* persistent_config,
                mjlib::micro::TelemetryManager* telemetry_manager,
                MillisecondTimer* timer,
                BldcServo* bldc,
                AuxPort* aux1_port,
                AuxPort* aux2_port);

  void PollMillisecond();

  const FlightControlStatus& status() const { return status_; }

 private:
  MillisecondTimer* const timer_;
  BldcServo* const bldc_;
  AuxPort* const aux1_port_;
  AuxPort* const aux2_port_;

  FlightControlConfig config_;
  FlightControlStatus status_;
  FlightControlLogic logic_;

  mjlib::base::inplace_function<void()> status_update_;
};

}  // namespace moteus
