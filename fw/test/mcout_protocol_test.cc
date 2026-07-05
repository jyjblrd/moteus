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

#include "fw/mcout_protocol.h"

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

namespace {

mcout::Frame MakeTestFrame() {
  mcout::Frame frame;
  frame.sequence = 100;
  frame.timestamp_usec = 123456789ull;
  frame.flags =
      mcout::Frame::kMotorsArmed |
      mcout::Frame::kSoftArmed |
      mcout::Frame::kInterlock |
      mcout::Frame::kMotorsInitialized |
      mcout::Frame::kOutputEnabled;
  frame.spool_state = mcout::Frame::kThrottleUnlimited;
  frame.desired_spool_state = 2;
  frame.roll = 0.125f;
  frame.pitch = -0.25f;
  frame.yaw = 0.0625f;
  frame.throttle = 0.5f;
  frame.roll_feedback = 0.1f;
  frame.pitch_feedback = -0.2f;
  frame.yaw_feedback = 0.05f;
  frame.roll_feedforward = 0.025f;
  frame.pitch_feedforward = -0.05f;
  frame.yaw_feedforward = 0.0125f;
  frame.throttle_filtered = 0.5f;
  frame.throttle_out = 0.55f;
  frame.forward = 0.0f;
  frame.lateral = 0.0f;
  frame.dt = 0.0025f;
  return frame;
}

int FeedAll(mcout::Parser* parser,
            const uint8_t* data, size_t size,
            mcout::Frame* frame) {
  int complete = 0;
  for (size_t i = 0; i < size; i++) {
    if (parser->Feed(data[i], frame)) { complete++; }
  }
  return complete;
}

}  // namespace

BOOST_AUTO_TEST_CASE(McoutCrc16Xmodem) {
  // The standard CRC-16/XMODEM check value.
  const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  BOOST_TEST(mcout::Crc16Xmodem(data, sizeof(data)) == 0x31c3);
}

BOOST_AUTO_TEST_CASE(McoutParserRoundTrip) {
  uint8_t buffer[mcout::Frame::kLength] = {};
  const auto sent = MakeTestFrame();
  mcout::Parser::Encode(sent, buffer);

  mcout::Parser dut;
  mcout::Frame received;

  BOOST_TEST(FeedAll(&dut, buffer, sizeof(buffer), &received) == 1);
  BOOST_TEST(dut.stats().valid_frames == 1);
  BOOST_TEST(dut.stats().crc_errors == 0);
  BOOST_TEST(dut.stats().header_errors == 0);
  BOOST_TEST(dut.stats().missed_frames == 0);

  BOOST_TEST(received.sequence == 100);
  BOOST_TEST(received.timestamp_usec == 123456789ull);
  BOOST_TEST(received.flags == sent.flags);
  BOOST_TEST(static_cast<int>(received.spool_state) ==
             static_cast<int>(mcout::Frame::kThrottleUnlimited));
  BOOST_TEST(static_cast<int>(received.desired_spool_state) == 2);
  BOOST_TEST(received.roll == 0.125f);
  BOOST_TEST(received.pitch == -0.25f);
  BOOST_TEST(received.yaw == 0.0625f);
  BOOST_TEST(received.throttle == 0.5f);
  BOOST_TEST(received.roll_feedback == 0.1f);
  BOOST_TEST(received.pitch_feedback == -0.2f);
  BOOST_TEST(received.yaw_feedback == 0.05f);
  BOOST_TEST(received.roll_feedforward == 0.025f);
  BOOST_TEST(received.pitch_feedforward == -0.05f);
  BOOST_TEST(received.yaw_feedforward == 0.0125f);
  BOOST_TEST(received.throttle_filtered == 0.5f);
  BOOST_TEST(received.throttle_out == 0.55f);
  BOOST_TEST(received.dt == 0.0025f);
  BOOST_TEST(received.output_allowed() == true);
}

BOOST_AUTO_TEST_CASE(McoutParserBackToBackAndGarbage) {
  uint8_t buffer[mcout::Frame::kLength] = {};
  auto sent = MakeTestFrame();

  mcout::Parser dut;
  mcout::Frame received;

  // Idle garbage that does not look like a frame start is ignored
  // silently.
  const uint8_t garbage[] = {0xff, 0x00, 0x12, 0xc0, 0xff};
  BOOST_TEST(FeedAll(&dut, garbage, sizeof(garbage), &received) == 0);
  BOOST_TEST(dut.stats().header_errors == 0);

  sent.sequence = 1;
  mcout::Parser::Encode(sent, buffer);
  BOOST_TEST(FeedAll(&dut, buffer, sizeof(buffer), &received) == 1);

  sent.sequence = 2;
  mcout::Parser::Encode(sent, buffer);
  BOOST_TEST(FeedAll(&dut, buffer, sizeof(buffer), &received) == 1);

  BOOST_TEST(dut.stats().valid_frames == 2);
  BOOST_TEST(dut.stats().missed_frames == 0);
  BOOST_TEST(received.sequence == 2);
}

BOOST_AUTO_TEST_CASE(McoutParserResync) {
  uint8_t buffer[mcout::Frame::kLength] = {};
  const auto sent = MakeTestFrame();
  mcout::Parser::Encode(sent, buffer);

  mcout::Parser dut;
  mcout::Frame received;

  // A magic byte followed by a non-magic byte breaks the partial
  // frame, but the stream recovers on the next real frame.
  const uint8_t false_start[] = {0x4d, 0x22};
  BOOST_TEST(FeedAll(&dut, false_start, sizeof(false_start), &received) == 0);
  BOOST_TEST(dut.stats().header_errors == 1);

  BOOST_TEST(FeedAll(&dut, buffer, sizeof(buffer), &received) == 1);
  BOOST_TEST(dut.stats().valid_frames == 1);

  // A false start whose offending byte is itself a magic byte still
  // captures an immediately following frame.
  const uint8_t overlapping_start[] = {0x4d};
  BOOST_TEST(FeedAll(&dut, overlapping_start,
                     sizeof(overlapping_start), &received) == 0);
  BOOST_TEST(FeedAll(&dut, buffer, sizeof(buffer), &received) == 1);
  BOOST_TEST(dut.stats().valid_frames == 2);
}

BOOST_AUTO_TEST_CASE(McoutParserHeaderRejects) {
  uint8_t buffer[mcout::Frame::kLength] = {};
  const auto sent = MakeTestFrame();

  mcout::Parser dut;
  mcout::Frame received;

  // Wrong version.
  mcout::Parser::Encode(sent, buffer);
  buffer[mcout::kOffsetVersion] = 9;
  BOOST_TEST(FeedAll(&dut, buffer, sizeof(buffer), &received) == 0);
  BOOST_TEST(dut.stats().header_errors >= 1);

  // Wrong length.
  mcout::Parser::Encode(sent, buffer);
  buffer[mcout::kOffsetLength] = 80;
  BOOST_TEST(FeedAll(&dut, buffer, sizeof(buffer), &received) == 0);

  // Wrong message id.
  mcout::Parser::Encode(sent, buffer);
  buffer[mcout::kOffsetMessageId] = 2;
  BOOST_TEST(FeedAll(&dut, buffer, sizeof(buffer), &received) == 0);

  BOOST_TEST(dut.stats().valid_frames == 0);

  // A clean frame afterwards parses.
  mcout::Parser::Encode(sent, buffer);
  BOOST_TEST(FeedAll(&dut, buffer, sizeof(buffer), &received) == 1);
  BOOST_TEST(dut.stats().valid_frames == 1);
}

BOOST_AUTO_TEST_CASE(McoutParserCrcError) {
  uint8_t buffer[mcout::Frame::kLength] = {};
  const auto sent = MakeTestFrame();
  mcout::Parser::Encode(sent, buffer);

  mcout::Parser dut;
  mcout::Frame received;

  // Corrupt a payload byte.
  buffer[mcout::kOffsetThrottle] ^= 0x01;
  BOOST_TEST(FeedAll(&dut, buffer, sizeof(buffer), &received) == 0);
  BOOST_TEST(dut.stats().crc_errors == 1);
  BOOST_TEST(dut.stats().valid_frames == 0);

  // A clean frame afterwards parses.
  mcout::Parser::Encode(sent, buffer);
  BOOST_TEST(FeedAll(&dut, buffer, sizeof(buffer), &received) == 1);
  BOOST_TEST(dut.stats().valid_frames == 1);
}

BOOST_AUTO_TEST_CASE(McoutParserSequenceTracking) {
  uint8_t buffer[mcout::Frame::kLength] = {};
  auto sent = MakeTestFrame();

  mcout::Parser dut;
  mcout::Frame received;

  sent.sequence = 5;
  mcout::Parser::Encode(sent, buffer);
  BOOST_TEST(FeedAll(&dut, buffer, sizeof(buffer), &received) == 1);
  BOOST_TEST(dut.stats().missed_frames == 0);

  // A gap of two dropped frames.
  sent.sequence = 8;
  mcout::Parser::Encode(sent, buffer);
  BOOST_TEST(FeedAll(&dut, buffer, sizeof(buffer), &received) == 1);
  BOOST_TEST(dut.stats().missed_frames == 2);

  // An implausibly large gap (e.g. an autopilot reboot) is not
  // counted.
  sent.sequence = 2;
  mcout::Parser::Encode(sent, buffer);
  BOOST_TEST(FeedAll(&dut, buffer, sizeof(buffer), &received) == 1);
  BOOST_TEST(dut.stats().missed_frames == 2);
  BOOST_TEST(dut.stats().valid_frames == 3);
}

BOOST_AUTO_TEST_CASE(McoutOutputAllowed) {
  mcout::Frame frame;

  constexpr uint16_t kAllRequired =
      mcout::Frame::kMotorsArmed |
      mcout::Frame::kSoftArmed |
      mcout::Frame::kInterlock |
      mcout::Frame::kOutputEnabled;

  frame.flags = 0;
  BOOST_TEST(frame.output_allowed() == false);

  frame.flags = kAllRequired;
  BOOST_TEST(frame.output_allowed() == true);

  // Any missing required flag denies output.
  frame.flags = kAllRequired & ~mcout::Frame::kInterlock;
  BOOST_TEST(frame.output_allowed() == false);
  frame.flags = kAllRequired & ~mcout::Frame::kOutputEnabled;
  BOOST_TEST(frame.output_allowed() == false);

  // Emergency stop denies output regardless.
  frame.flags = kAllRequired | mcout::Frame::kEmergencyStop;
  BOOST_TEST(frame.output_allowed() == false);

  // Extra informational flags do not affect the result.
  frame.flags = kAllRequired |
      mcout::Frame::kMotorsInitialized |
      mcout::Frame::kThrustBoost;
  BOOST_TEST(frame.output_allowed() == true);
}
