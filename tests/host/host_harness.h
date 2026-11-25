/*
 * host_harness.h
 *
 * Test-only glue for running secplus_receiver on ESPHome's native `host`
 * platform (see secplus_host.yaml). It is pulled in through
 * `esphome: includes:` and is never part of a real firmware build.
 *
 * It provides two things:
 *   1. No-op bodies for RemoteReceiverComponent, which has no implementation
 *      on the host platform (there is no radio to read).
 *   2. secplus_test_press(), which builds a real Security+ 2.0 transmission
 *      with the upstream encoder and feeds its pulse timings into the
 *      component exactly as remote_receiver would.
 */

#pragma once

#ifdef USE_HOST

#include <cstdint>
#include <string>
#include <vector>

#include "esphome/components/remote_base/remote_base.h"
#include "esphome/components/remote_receiver/remote_receiver.h"
#include "esphome/components/secplus_receiver/secplus_receiver.h"
#include "esphome/core/log.h"

extern "C" {
#include "esphome/components/secplus_receiver/secplus.h"
}

namespace esphome {
namespace remote_receiver {

void RemoteReceiverComponent::setup() {}
void RemoteReceiverComponent::dump_config() {}
void RemoteReceiverComponent::loop() {}

}  // namespace remote_receiver
}  // namespace esphome

namespace secplus_test {

static const int HALF_BIT_US = 250;   // 4000 baud
static const int GAP_US = 60000;      // silence between packets

// Append one data bit, Manchester encoded: 0 -> high,low  1 -> low,high.
inline void push_bit(std::vector<uint8_t> &levels, int bit) {
  levels.push_back(bit ? 0 : 1);
  levels.push_back(bit ? 1 : 0);
}

// One over-the-air packet: 20-bit preamble, 2-bit frame ID, then a 40-bit
// payload (remote) or a 64-bit payload (keypad).
inline void push_packet(std::vector<uint8_t> &levels, int frame_id, const uint8_t *payload, int payload_bits) {
  for (int i = 0; i < 16; i++) {
    push_bit(levels, 0);
  }
  for (int i = 0; i < 4; i++) {
    push_bit(levels, 1);
  }
  push_bit(levels, 0);
  push_bit(levels, frame_id);
  for (int i = 0; i < payload_bits; i++) {
    push_bit(levels, (payload[i >> 3] >> (7 - (i & 7))) & 1);
  }
}

// Convert half-bit levels into signed pulse durations (positive = high,
// negative = low) followed by a long low gap. With gap = false the pulses
// simply stop, as they do when a receive burst ends.
inline void push_timings(esphome::remote_base::RawTimings &raw, const std::vector<uint8_t> &levels,
                         bool gap = true) {
  size_t i = 0;
  while (i < levels.size()) {
    size_t run = 1;
    while (i + run < levels.size() && levels[i + run] == levels[i]) {
      run++;
    }
    int32_t dur = (int32_t) run * HALF_BIT_US;
    raw.push_back(levels[i] ? dur : -dur);
    i += run;
  }
  if (!gap) {
    // The receiver never reports the trailing low: it is part of the silence.
    if (!raw.empty() && raw.back() < 0) {
      raw.pop_back();
    }
    return;
  }
  if (!raw.empty() && raw.back() < 0) {
    raw.back() -= GAP_US;
  } else {
    raw.push_back(-GAP_US);
  }
}

}  // namespace secplus_test

// Simulate one button press. Like a real remote, the packet pair is sent
// twice, which also exercises the component's duplicate filter.
// With pin >= 0 the press is a keypad entry instead: a 64-bit frame carrying
// that PIN.
inline void secplus_test_press(esphome::secplus_receiver::SecplusReceiverComponent *rx, uint64_t fixed,
                               uint32_t rolling, int pin = -1) {
  uint8_t packet1[8] = {0};
  uint8_t packet2[8] = {0};
  const uint8_t frame_type = pin >= 0 ? 1 : 0;
  const int payload_bits = frame_type ? 64 : 40;
  // Keypad data: PIN low byte in bits 31..24, high byte in bits 23..16.
  const uint32_t data = pin >= 0 ? (((uint32_t) pin & 0xff) << 24) | ((((uint32_t) pin >> 8) & 0xff) << 16) : 0;
  if (encode_v2(rolling, fixed, data, frame_type, packet1, packet2) < 0) {
    ESP_LOGE("secplus_test", "encode_v2 failed for fixed=%llu rolling=%u", (unsigned long long) fixed,
             (unsigned) rolling);
    return;
  }

  esphome::remote_base::RawTimings raw;
  for (int repeat = 0; repeat < 2; repeat++) {
    std::vector<uint8_t> levels;
    secplus_test::push_packet(levels, 0, packet1, payload_bits);
    secplus_test::push_timings(raw, levels);
    levels.clear();
    secplus_test::push_packet(levels, 1, packet2, payload_bits);
    secplus_test::push_timings(raw, levels);
  }

  rx->on_receive(esphome::remote_base::RemoteReceiveData(raw, 25, esphome::remote_base::TOLERANCE_MODE_PERCENTAGE));
}

// Inject hand-picked frames of one transmission as a single receive burst,
// to exercise the decoder's frame pairing. `frames` is a sequence of:
//   '1'  frame 1            '2'  frame 2
//   't'  frame 2 cut short (20 of its 40 payload bits)
//   'x'  (last character only) end the burst with no trailing gap
inline void secplus_test_inject(esphome::secplus_receiver::SecplusReceiverComponent *rx, uint64_t fixed,
                                uint32_t rolling, const std::string &frames) {
  uint8_t packet1[8] = {0};
  uint8_t packet2[8] = {0};
  if (encode_v2(rolling, fixed, 0, 0, packet1, packet2) < 0) {
    ESP_LOGE("secplus_test", "encode_v2 failed for fixed=%llu rolling=%u", (unsigned long long) fixed,
             (unsigned) rolling);
    return;
  }
  const bool no_gap = !frames.empty() && frames.back() == 'x';
  const size_t count = no_gap ? frames.size() - 1 : frames.size();

  esphome::remote_base::RawTimings raw;
  for (size_t i = 0; i < count; i++) {
    std::vector<uint8_t> levels;
    switch (frames[i]) {
      case '1':
        secplus_test::push_packet(levels, 0, packet1, 40);
        break;
      case '2':
        secplus_test::push_packet(levels, 1, packet2, 40);
        break;
      case 't':
        secplus_test::push_packet(levels, 1, packet2, 20);
        break;
      default:
        ESP_LOGE("secplus_test", "unknown frame token '%c'", frames[i]);
        return;
    }
    secplus_test::push_timings(raw, levels, !(no_gap && i + 1 == count));
  }
  rx->on_receive(esphome::remote_base::RemoteReceiveData(raw, 25, esphome::remote_base::TOLERANCE_MODE_PERCENTAGE));
}

#endif  // USE_HOST
