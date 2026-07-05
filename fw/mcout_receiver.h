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

#include <array>

#include "mjlib/base/string_span.h"

#include "fw/mcout_protocol.h"
#include "fw/millisecond_timer.h"
#include "fw/stm32g4_dma_uart.h"

namespace moteus {

/// Owns the receive side of an AUX UART configured in `mcout` mode:
/// continuously drains the DMA stream through the MCOUT parser and
/// retains the most recent valid frame.
///
/// Poll() must be called from the main loop, like the aux tunnel
/// path.  At the default 921600 baud the receive buffer holds ~2ms
/// of line data, far longer than a main loop iteration.
class McoutReceiver {
 public:
  McoutReceiver(Stm32G4DmaUart* uart, MillisecondTimer* timer)
      : uart_(uart), timer_(timer) {
    uart_->start_dma_read(rx_span());
  }

  ~McoutReceiver() {
    uart_->finish_dma_read();
  }

  void Poll() {
    if (uart_->read_bytes_remaining() ==
        static_cast<int>(buffer_.size())) {
      return;
    }
    uart_->finish_dma_read();
    const int size =
        static_cast<int>(buffer_.size()) - uart_->read_bytes_remaining();
    for (int i = 0; i < size; i++) {
      if (parser_.Feed(static_cast<uint8_t>(buffer_[i]), &frame_)) {
        last_frame_ms_ = timer_->read_ms();
        have_frame_ = true;
      }
    }
    uart_->start_dma_read(rx_span());
  }

  bool have_frame() const { return have_frame_; }
  const mcout::Frame& frame() const { return frame_; }
  const mcout::Parser::Stats& stats() const { return parser_.stats(); }
  uint32_t last_frame_ms() const { return last_frame_ms_; }

 private:
  mjlib::base::string_span rx_span() {
    return mjlib::base::string_span(
        buffer_.data(), buffer_.data() + buffer_.size());
  }

  Stm32G4DmaUart* const uart_;
  MillisecondTimer* const timer_;

  mcout::Parser parser_;
  mcout::Frame frame_;
  bool have_frame_ = false;
  uint32_t last_frame_ms_ = 0;

  std::array<char, 192> buffer_ = {};
};

}  // namespace moteus
