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
#include <cstdint>

#include "mjlib/base/string_span.h"

#include "fw/mcout_protocol.h"
#include "fw/millisecond_timer.h"
#include "fw/stm32g4_dma_uart.h"

namespace moteus {

/// Owns the receive side of an AUX UART configured in `mcout` mode:
/// drains the DMA stream through the MCOUT parser and retains the
/// most recent valid frame.  Poll() must be called from the main
/// loop, like the aux tunnel path.
///
/// DMA runs continuously into the buffer while bytes are consumed
/// behind it, so there is no stop/restart gap in normal operation.
/// When the buffer fills, the re-arm path first consumes the
/// unprocessed tail and then recovers any bytes that arrive while
/// DMA is disarmed directly from the UART receive register.
class McoutReceiver {
 public:
  McoutReceiver(Stm32G4DmaUart* uart, MillisecondTimer* timer)
      : uart_(uart), timer_(timer) {
    Rearm();
  }

  ~McoutReceiver() {
    uart_->finish_dma_read();
  }

  void Poll() {
    // Consume whatever DMA has delivered so far, without stopping it.
    ProcessTo(buffer_.size() -
              static_cast<size_t>(uart_->read_bytes_remaining()));

    if (uart_->is_dma_read_finished()) {
      // The buffer filled.  Stop DMA, consume any tail bytes that
      // landed after the snapshot above, then re-arm.
      uart_->finish_dma_read();
      ProcessTo(buffer_.size());
      Rearm();
    }
  }

  bool have_frame() const { return have_frame_; }
  const mcout::Frame& frame() const { return frame_; }
  const mcout::Parser::Stats& stats() const { return parser_.stats(); }
  uint32_t last_frame_ms() const { return last_frame_ms_; }

 private:
  void ProcessTo(size_t end) {
    while (processed_ < end) {
      Feed(buffer_[processed_++]);
    }
  }

  void Feed(uint8_t byte) {
    if (parser_.Feed(byte, &frame_)) {
      last_frame_ms_ = timer_->read_ms();
      have_frame_ = true;
    }
  }

  void Rearm() {
    auto* const uart = uart_->uart();

    // Clear any stale receive errors and consume bytes that arrived
    // while DMA was not armed.
    uart->ICR =
        USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    while ((uart->ISR & USART_ISR_RXNE) != 0) {
      Feed(static_cast<uint8_t>(uart->RDR));
    }

    processed_ = 0;
    uart_->start_dma_read(
        mjlib::base::string_span(
            reinterpret_cast<char*>(buffer_.data()),
            reinterpret_cast<char*>(buffer_.data()) + buffer_.size()));
  }

  Stm32G4DmaUart* const uart_;
  MillisecondTimer* const timer_;

  mcout::Parser parser_;
  mcout::Frame frame_;
  bool have_frame_ = false;
  uint32_t last_frame_ms_ = 0;

  // At 921600 baud this holds ~2.8ms of line data; the main loop
  // polls far more often than that, so in practice the buffer-full
  // re-arm path only runs at startup or after a stall.
  std::array<uint8_t, 256> buffer_ = {};
  size_t processed_ = 0;
};

}  // namespace moteus
