/*
 * secplus_receiver.cpp
 *
 * Decodes Manchester-encoded (G.E. Thomas convention) Security+ 2.0 pulse data
 * at 4000 baud and feeds the result to the upstream argilo/secplus decoder.
 *
 * Input format (from remote_receiver via on_receive):
 *   int32_t durations in microseconds.
 *   Negative = signal LOW (space), positive = signal HIGH (mark).
 *
 * Encoding:
 *   Clock period = 250 us (4000 baud). Every bit has a mid-cycle transition.
 *   G.E. Thomas convention:
 *     Bit 0  ->  HIGH first half, LOW second half  ->  "10"
 *     Bit 1  ->  LOW first half, HIGH second half  ->  "01"
 *
 * Preamble:
 *   Data 0x0F (00001111) encodes to "1010101001010101" = 0xAA55, located via a
 *   sliding 16-bit window over the packed bit buffer.
 */

#include "secplus_receiver.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cstring>

extern "C" {
#include "secplus.h"
}

namespace esphome {
namespace secplus_receiver {

static const char *const TAG = "secplus_receiver";

// ── Tuneable constants ───────────────────────────────────────────────────────
static const int CLOCK_US = 250;       // microseconds per half-bit clock period
static const int LONG_THRESH = 625;    // |dur| >= this -> packet gap / flush

static const int MAX_MANCHESTER = 512;   // max Manchester bits in flight
static const int MAX_DECODED_BITS = 256;  // max decoded payload bits per packet
static const int MAX_DECODED_BYTES = MAX_DECODED_BITS / 8;

// The two frames of a packet are sent back to back. A stored frame 1 older
// than this belongs to an earlier transmission and must not be paired.
static const uint32_t PAIR_TIMEOUT_MS = 250;

// With require_repeat, the same code must be decoded twice within this time.
// A remote repeats its packet several times per press, well inside it.
static const uint32_t REPEAT_WINDOW_MS = 1500;

// Preamble: 0x0F Manchester-encoded = "1010101001010101" = 0xAA55
static const int PREAMBLE_BITS = 16;
static const uint16_t PREAMBLE_VAL = 0xAA55u;

// ── Packed bit-array helpers ─────────────────────────────────────────────────
// Bits stored MSB-first within each byte: index 0 -> byte 0 bit 7 (0x80).
static inline void bitarr_set(uint8_t *arr, int idx, int val) {
  if (val) {
    arr[idx >> 3] |= (0x80u >> (idx & 7));
  } else {
    arr[idx >> 3] &= ~(0x80u >> (idx & 7));
  }
}

static inline int bitarr_get(const uint8_t *arr, int idx) { return (arr[idx >> 3] >> (7 - (idx & 7))) & 1; }

// Sliding 16-bit window search for PREAMBLE_VAL (0xAA55).
// Returns bit index immediately after the preamble, or -1 if not found.
static int find_preamble(const uint8_t *bits, int nbits) {
  if (nbits < PREAMBLE_BITS) {
    return -1;
  }
  uint16_t window = 0;
  for (int i = 0; i < PREAMBLE_BITS - 1; i++) {
    window = (uint16_t) ((window << 1) | bitarr_get(bits, i));
  }
  for (int i = PREAMBLE_BITS - 1; i < nbits; i++) {
    window = (uint16_t) ((window << 1) | bitarr_get(bits, i));
    if (window == PREAMBLE_VAL) {
      return i + 1;
    }
  }
  return -1;
}

// Manchester decode (G.E. Thomas): "10" -> 0, "01" -> 1, anything else -> sync
// error (skip one bit and retry). Returns number of bits written to out, or -1
// on overflow.
static int manchester_decode(const uint8_t *bits, int start, int nbits, uint8_t *out, int out_size_bits) {
  int pos = 0;
  int i = start;
  while (i + 1 < nbits) {
    int a = bitarr_get(bits, i);
    int b = bitarr_get(bits, i + 1);
    if (a == 1 && b == 0) {
      if (pos >= out_size_bits) {
        return -1;
      }
      bitarr_set(out, pos++, 0);
      i += 2;
    } else if (a == 0 && b == 1) {
      if (pos >= out_size_bits) {
        return -1;
      }
      bitarr_set(out, pos++, 1);
      i += 2;
    } else {
      i++;
    }
  }
  return pos;
}

// Finds the preamble, reads the 2-field header (frame ID + packet length), then
// Manchester-decodes the payload into the packed packet buffer.
// Returns frame_id (1 or 2), -1 if no preamble / insufficient data, or -2 on
// unknown header values / decode overflow. *expected_length is the payload
// size the header announces; a frame is complete only if *packet_length
// equals it.
static int process_packet(const uint8_t *bits, int nbits, uint8_t *packet, int *packet_length,
                          int *expected_length) {
  int start = find_preamble(bits, nbits);
  if (start < 0) {
    return -1;
  }
  if (start + 8 > nbits) {
    return -1;
  }

  // Frame ID: 4 Manchester bits after preamble.
  //   "1010" (data 00) -> frame 1, "1001" (data 01) -> frame 2
  int a0 = bitarr_get(bits, start + 0), a1 = bitarr_get(bits, start + 1);
  int a2 = bitarr_get(bits, start + 2), a3 = bitarr_get(bits, start + 3);
  int frame_id;
  if (a0 == 1 && a1 == 0 && a2 == 1 && a3 == 0) {
    frame_id = 1;
  } else if (a0 == 1 && a1 == 0 && a2 == 0 && a3 == 1) {
    frame_id = 2;
  } else {
    return -2;
  }
  start += 4;

  // Packet length: next 4 Manchester bits.
  //   "1010" -> 40-bit payload, "1001" -> 64-bit payload
  int b0 = bitarr_get(bits, start + 0), b1 = bitarr_get(bits, start + 1);
  int b2 = bitarr_get(bits, start + 2), b3 = bitarr_get(bits, start + 3);
  if (b0 == 1 && b1 == 0 && b2 == 1 && b3 == 0) {
    *expected_length = 40;
  } else if (b0 == 1 && b1 == 0 && b2 == 0 && b3 == 1) {
    *expected_length = 64;
  } else {
    return -2;
  }

  int nbits_decoded = manchester_decode(bits, start, nbits, packet, MAX_DECODED_BITS);
  if (nbits_decoded < 0) {
    return -2;
  }
  *packet_length = nbits_decoded;
  return frame_id;
}

// ── Component ────────────────────────────────────────────────────────────────

// Preference keys. Learn slots are keyed by index so that changing
// max_remotes keeps the slots that still exist. Static remotes are keyed by
// their fixed code so reordering the YAML list keeps their counters.
// This holds on platforms that look preferences up by key (ESP32, LibreTiny,
// host). ESP8266 and RP2040 allocate preferences in creation order, so there
// a change to max_remotes, to the static list, or to other components that
// save state can discard the saved slots.
static const uint32_t SLOT_PREF_BASE = fnv1_hash("secplus_receiver_slot");
static const uint32_t STATIC_PREF_BASE = fnv1_hash("secplus_receiver_static");

void SecplusReceiverComponent::setup() {
  if (!this->auth_enabled_) {
    return;
  }
  const size_t n = this->store_.size();
  this->slot_prefs_.reserve(n);
  for (size_t i = 0; i < n; i++) {
    uint32_t key;
    if (this->store_.is_static(i)) {
      const uint64_t fixed = this->store_.slot(i).fixed;
      key = fnv1_hash_extend(STATIC_PREF_BASE, fixed);
    } else {
      key = SLOT_PREF_BASE + (uint32_t) i;
    }
    // in_flash=true: on ESP8266 the default storage is RTC memory, which a
    // power cut erases.
    this->slot_prefs_.push_back(global_preferences->make_preference<RemoteSlot>(key, true));
  }
  // Slots that changed while loading and must be written back.
  std::vector<size_t> dirty;
  for (size_t i = 0; i < n; i++) {
    RemoteSlot saved;
    if (this->slot_prefs_[i].load(&saved)) {
      int absorbed_by = this->store_.restore(i, saved);
      if (absorbed_by >= 0) {
        ESP_LOGI(TAG, "Learned copy of fixed=%" PRIu64 " in slot %u dropped: already in slot %d", saved.fixed,
                 (unsigned) (i + 1), absorbed_by + 1);
        dirty.push_back(i);
        dirty.push_back((size_t) absorbed_by);
      }
    }
  }
  for (size_t i : dirty) {
    this->save_slot_(i);
  }
  for (size_t i = 0; i < n; i++) {
    this->publish_slot_(i);
  }
}

void SecplusReceiverComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Security+ 2.0 Receiver:");
  LOG_TEXT_SENSOR("  ", "Remote ID", this->remote_id_sensor_);
  LOG_TEXT_SENSOR("  ", "Rolling Code", this->rolling_code_sensor_);
  if (!this->auth_enabled_) {
    ESP_LOGCONFIG(TAG, "  Authentication: disabled");
    return;
  }
  ESP_LOGCONFIG(TAG,
                "  Authentication: enabled\n"
                "    Learn slots: %u\n"
                "    Static remotes: %u\n"
                "    Counter window: %u\n"
                "    Learn timeout: %u ms",
                (unsigned) this->store_.learn_slots(), (unsigned) (this->store_.size() - this->store_.learn_slots()),
                (unsigned) this->window_, (unsigned) this->learn_timeout_ms_);
  for (size_t i = 0; i < this->store_.size(); i++) {
    if (this->store_.occupied(i)) {
      ESP_LOGCONFIG(TAG, "    Slot %u%s: fixed=%" PRIu64, (unsigned) (i + 1), this->store_.is_static(i) ? " (static)" : "",
                    this->store_.slot(i).fixed);
    }
  }
}

void SecplusReceiverComponent::publish_slot_(size_t index) {
  if (index >= this->slot_sensors_.size() || this->slot_sensors_[index] == nullptr) {
    return;
  }
  if (!this->store_.occupied(index)) {
    this->slot_sensors_[index]->publish_state("empty");
    return;
  }
  char buf[24];
  snprintf(buf, sizeof(buf), "%" PRIu64, this->store_.slot(index).fixed);
  this->slot_sensors_[index]->publish_state(buf);
}

void SecplusReceiverComponent::save_slot_(size_t index) {
  if (index >= this->slot_prefs_.size()) {
    return;
  }
  RemoteSlot slot = this->store_.slot(index);
  // Write straight away instead of waiting for flash_write_interval: a counter
  // lost to a power cut would make recently used codes replayable. Presses
  // are rare, so the extra flash wear is negligible.
  if (!this->slot_prefs_[index].save(&slot) || !global_preferences->sync()) {
    ESP_LOGE(TAG, "Failed to save slot %u to flash", (unsigned) (index + 1));
  }
  this->publish_slot_(index);
}

void SecplusReceiverComponent::start_learn() {
  if (!this->auth_enabled_) {
    ESP_LOGW(TAG, "Learn requested, but 'authentication' is not configured");
    return;
  }
  this->store_.start_learn();
  ESP_LOGI(TAG, "Learn mode open for %u s: press the new remote's button twice",
           (unsigned) (this->learn_timeout_ms_ / 1000));
  this->set_timeout("learn", this->learn_timeout_ms_, [this]() {
    if (this->store_.learning()) {
      this->store_.stop_learn();
      ESP_LOGI(TAG, "Learn mode timed out, nothing learned");
    }
  });
}

void SecplusReceiverComponent::stop_learn() {
  this->cancel_timeout("learn");
  if (this->store_.learning()) {
    this->store_.stop_learn();
    ESP_LOGI(TAG, "Learn mode closed");
  }
}

void SecplusReceiverComponent::forget(int slot) {
  if (!this->auth_enabled_) {
    ESP_LOGW(TAG, "Forget requested, but 'authentication' is not configured");
    return;
  }
  if (slot < 1 || (size_t) slot > this->store_.size()) {
    ESP_LOGW(TAG, "Cannot forget slot %d: there are only slots 1 to %u", slot, (unsigned) this->store_.size());
    return;
  }
  const size_t index = (size_t) slot - 1;
  if (this->store_.is_static(index)) {
    // A static remote stays enrolled; forgetting it discards its counter so
    // a remote that has fallen behind the stored counter can be used again.
    if (this->store_.reset_counter(index)) {
      this->save_slot_(index);
      ESP_LOGI(TAG, "Slot %d is a static remote: counter reset, its next code will be trusted", slot);
    } else {
      ESP_LOGI(TAG, "Slot %d is a static remote with no stored counter: nothing to reset", slot);
    }
    return;
  }
  if (!this->store_.forget(index)) {
    ESP_LOGW(TAG, "Cannot forget slot %d: it is empty", slot);
    return;
  }
  this->save_slot_(index);
  ESP_LOGI(TAG, "Forgot remote in slot %d", slot);
}

void SecplusReceiverComponent::forget_all() {
  if (!this->auth_enabled_) {
    ESP_LOGW(TAG, "Forget requested, but 'authentication' is not configured");
    return;
  }
  for (size_t i = 0; i < this->store_.learn_slots(); i++) {
    if (this->store_.forget(i)) {
      this->save_slot_(i);
    }
  }
  ESP_LOGI(TAG, "Forgot all learned remotes");
}

void SecplusReceiverComponent::handle_learn_(uint64_t fixed, uint32_t rolling) {
  int slot = -1;
  switch (this->store_.learn(fixed, rolling, &slot)) {
    case LearnResult::PENDING:
      ESP_LOGI(TAG, "Learn: heard fixed=%" PRIu64 ", press the same button again to confirm", fixed);
      break;
    case LearnResult::LEARNED: {
      this->cancel_timeout("learn");
      this->save_slot_(slot);
      ESP_LOGI(TAG, "Learned remote fixed=%" PRIu64 " into slot %d", fixed, slot + 1);
      this->remote_learned_callback_.call(fixed, (uint8_t) (slot + 1));
      if (this->fire_event_) {
        char buf[24];
        std::map<std::string, std::string> data;
        snprintf(buf, sizeof(buf), "%" PRIu64, fixed);
        data["fixed_data"] = buf;
        snprintf(buf, sizeof(buf), "%d", slot + 1);
        data["slot"] = buf;
        this->fire_homeassistant_event("esphome.secplus_remote_learned", data);
      }
      break;
    }
    case LearnResult::RESYNCED:
      this->cancel_timeout("learn");
      this->save_slot_(slot);
      ESP_LOGI(TAG, "Remote in slot %d was already enrolled; counter resynchronised", slot + 1);
      break;
    case LearnResult::REJECTED:
      ESP_LOGW(TAG, "Learn: remote in slot %d sent a counter that is not ahead of the stored one, ignored", slot + 1);
      break;
    case LearnResult::FULL:
      this->cancel_timeout("learn");
      ESP_LOGW(TAG, "Learn failed: no free slot (%u learnable slots), forget one first",
               (unsigned) this->store_.learn_slots());
      break;
    case LearnResult::NOT_LEARNING:
      break;
  }
}

bool SecplusReceiverComponent::on_receive(remote_base::RemoteReceiveData data) {
  const std::vector<int32_t> &raw = data.get_raw_data();
  this->decode_raw_(raw.data(), raw.size());
  return true;
}

void SecplusReceiverComponent::decode_raw_(const int32_t *pulses, int n_pulses) {

  ESP_LOGD(TAG, "Decoder received %d pulses to process...", n_pulses );

  for (int i = 0; i < n_pulses; i++) {
    int p = pulses[i];
    int dur = (p < 0) ? -p : p;
    int lvl = (p > 0) ? 1 : 0;

    bool flush = (this->manchester_pos_ >= MAX_MANCHESTER - 4);

    // ── Gap or buffer-full: the frame is over ──
    if (dur >= LONG_THRESH || flush) {
      if (flush) {
        ESP_LOGW(TAG, "Manchester buffer full (%d bits), flushing. Could mean a noisy radio, try another frequency",
                 this->manchester_pos_);
      }
      this->end_frame_(lvl);
      continue;
    }

    // ── Accumulate: round to 1 or 2 clock slots ──
    int slots = (dur + CLOCK_US / 2) / CLOCK_US;
    if (slots < 1) {
      slots = 1;
    }
    if (slots > 2) {
      slots = 2;
    }
    for (int s = 0; s < slots && this->manchester_pos_ < MAX_MANCHESTER; s++) {
      bitarr_set(this->manchester_, this->manchester_pos_++, lvl);
    }
  }

  // The end of a receive burst ends the frame too: either the radio went
  // quiet, or the receiver cut the burst and dropped what followed. Nothing
  // is carried into the next burst, where it would be joined to unrelated
  // pulses.
  if (this->manchester_pos_ > 0) {
    this->end_frame_(0);
  }
}

void SecplusReceiverComponent::end_frame_(int gap_level) {
  if (this->manchester_pos_ < MAX_MANCHESTER) {
    bitarr_set(this->manchester_, this->manchester_pos_++, gap_level);
  }

  // Decode into a zeroed scratch buffer, so a short frame can never be
  // completed with bits left over from an earlier one.
  uint8_t frame[MAX_DECODED_BYTES] = {0};
  int frame_bits = 0;
  int expected_bits = 0;
  int frame_id = process_packet(this->manchester_, this->manchester_pos_, frame, &frame_bits, &expected_bits);

  // Print out the raw Manchester for VERBOSE logging
  ESP_LOGV(TAG, "%s", this->get_bits_string().c_str());
  this->manchester_pos_ = 0;

  if (frame_id == -1) {
    return;  // no preamble: noise between frames, keep waiting
  }
  if (frame_id < 0 || frame_bits != expected_bits) {
    if (frame_id > 0) {
      ESP_LOGD(TAG, "frame #%d incomplete (%d of %d bits), discarding", frame_id, frame_bits, expected_bits);
    }
    this->frame1_bits_ = 0;  // whatever was pending is not followed by its frame 2
    return;
  }
  ESP_LOGD(TAG, "frame #%d decoded (%d bits)", frame_id, frame_bits);

  if (frame_id == 1) {
    memcpy(this->frame1_, frame, sizeof(this->frame1_));
    this->frame1_bits_ = frame_bits;
    this->frame1_ms_ = millis();
    return;
  }

  // Frame 2: usable only straight after a complete frame 1 of the same size.
  const bool have_frame1 = this->frame1_bits_ != 0;
  const bool fresh = have_frame1 && (millis() - this->frame1_ms_) <= PAIR_TIMEOUT_MS;
  const bool same_size = this->frame1_bits_ == frame_bits;
  this->frame1_bits_ = 0;  // a frame 1 is used once, whatever happens next
  if (!have_frame1) {
    ESP_LOGD(TAG, "got frame 2 without frame 1, discarding");
    return;
  }
  if (!fresh || !same_size) {
    ESP_LOGD(TAG, "frame 2 does not belong to the stored frame 1 (%s), discarding",
             fresh ? "different size" : "too old");
    return;
  }

  uint8_t frame_type = (frame_bits == 40) ? 0 : 1;
  uint32_t rolling = 0;
  uint64_t fixed = 0;
  uint32_t payload = 0;

  int ret = decode_v2(frame_type, this->frame1_, frame, &rolling, &fixed, &payload);
  if (ret < 0) {
    ESP_LOGE(TAG, "secplus decode_v2 failed and returned: %d", ret);
    return;
  }

  if (this->require_repeat_) {
    // decode_v2 has little validation, so a corrupted pair can decode to a
    // plausible wrong code. Act only on a code seen in two packets in a row.
    const uint32_t now = millis();
    const bool repeated = this->pending_valid_ && this->pending_fixed_ == fixed &&
                          this->pending_rolling_ == rolling && this->pending_data_ == payload &&
                          (now - this->pending_ms_) <= REPEAT_WINDOW_MS;
    this->pending_valid_ = true;
    this->pending_fixed_ = fixed;
    this->pending_rolling_ = rolling;
    this->pending_data_ = payload;
    this->pending_ms_ = now;
    if (!repeated) {
      ESP_LOGD(TAG, "fixed=%" PRIu64 " rolling=%u decoded once, waiting for a repeat", fixed, (unsigned) rolling);
      return;
    }
  }

  this->publish_(rolling, fixed, payload, frame_type);
}

void SecplusReceiverComponent::publish_(uint32_t rolling, uint64_t fixed, uint32_t data, uint8_t frame_type) {

    // Attempt to get the remote id and the button from the fixed data
    uint8_t button = (fixed >> 32) & 0xf;
    uint64_t remote_id = fixed & 0xf0ffffffffULL;

    // Pull out the keypad, if any. This implementation is incomplete.
    // This isn't great: the keypad = 0 means close all doors.
    std::string pincode = "";
    if (frame_type == 1) {   // 64-bit packet: data field is present
        uint32_t byte1 = data >> 24;
        uint32_t byte2 = (data >> 16) & 0xff;
        uint32_t pin  = (byte2 << 8) | byte1;
        uint32_t tail = data & 0xfff;   // unused for now; log or publish if useful

        if (button == 3) {
            pincode = "enter";
        } else {
            const char *suffix = (button == 1) ? "*" : (button == 2) ? "#" : "";
            // snprintf rather than esphome::str_sprintf, which ESPHome 2026.11.0 removed.
            char pin_buf[16];
            snprintf(pin_buf, sizeof(pin_buf), "%04u%s", (unsigned) pin, suffix);
            pincode = pin_buf;
        }
    }


    // Have we seen this fixed+rolling?
    // TODO: decode data and include in last seen
    bool already_seen = rolling == this->last_rolling && fixed == this->last_fixed;

    ESP_LOGD(TAG, "fixed=%" PRIu64 " remote_id=%" PRIu64 " rolling=%u button=%u %s%s data=0x%08X frame_type=%u %s",
        fixed,
        remote_id,
        rolling,
        button,
        pincode.length() > 0 ? "pincode=" : "",
        pincode.c_str(),
        data,
        frame_type,
        already_seen ? "[DUPLICATE]" : ""
    );

    if (already_seen) {
        return;
    }

    // To catch duplicates
    this->last_fixed = fixed;
    this->last_rolling = rolling;

    // Authentication: allowlist + rolling-counter replay protection.
    // While learn mode is open nothing authenticates, so enrolling a remote
    // never triggers whatever the remote normally controls.
    bool authenticated = false;
    bool rejected = false;
    int slot = -1;  // 0-based index into the store
    if (this->auth_enabled_) {
        if (this->store_.learning()) {
            this->handle_learn_(fixed, rolling);
        } else {
            switch (this->store_.check(fixed, rolling, this->window_, &slot)) {
                case AuthResult::AUTHENTICATED:
                    authenticated = true;
                    this->save_slot_(slot);
                    ESP_LOGI(TAG, "Authenticated remote in slot %d (rolling=%u)", slot + 1, (unsigned) rolling);
                    break;
                case AuthResult::REJECTED: {
                    const uint32_t stored = this->store_.slot(slot).rolling;
                    const uint32_t ahead = (rolling - stored) & ROLLING_MASK;
                    if (ahead >= 1 && ahead < FORWARD_LIMIT) {
                        ESP_LOGW(TAG,
                                 "Rejected remote in slot %d: rolling=%u is more than %u ahead of stored %u. "
                                 "Open learn mode and press the remote twice to resync",
                                 slot + 1, (unsigned) rolling, (unsigned) this->window_, (unsigned) stored);
                    } else {
                        ESP_LOGW(TAG,
                                 "Rejected remote in slot %d: rolling=%u is not ahead of stored %u (replay or old "
                                 "code). If this is the real remote, forget the slot to start over",
                                 slot + 1, (unsigned) rolling, (unsigned) stored);
                    }
                    rejected = true;
                    break;
                }
                case AuthResult::UNKNOWN:
                    break;
            }
        }
    }

    // Call the on_remote_received callback
    // TODO: send pincode, too?
    this->remote_received_callback_.call(fixed, remote_id, button, rolling);

    if (authenticated) {
        this->authenticated_callback_.call(fixed, remote_id, button, rolling, (uint8_t) (slot + 1));
    } else if (rejected) {
        this->rejected_callback_.call(fixed, rolling, (uint8_t) (slot + 1));
    }

    char buf[24];
    if (this->fixed_data_sensor_ != nullptr) {
        snprintf(buf, sizeof(buf), "%" PRIu64, fixed);
        this->fixed_data_sensor_->publish_state(buf);
    }

    if (this->rolling_code_sensor_ != nullptr) {
        snprintf(buf, sizeof(buf), "%u", (unsigned) rolling);
        this->rolling_code_sensor_->publish_state(buf);
    }

    if (this->remote_id_sensor_ != nullptr) {
        snprintf(buf, sizeof(buf), "%" PRIu64, remote_id);
        this->remote_id_sensor_->publish_state(buf);
    }

    if (this->button_sensor_ != nullptr) {
        snprintf(buf, sizeof(buf), "%u", (unsigned) button);
        this->button_sensor_->publish_state(buf);
    }


    if (this->fire_event_) {
        ESP_LOGD(TAG,"Sent 'esphome.secplus_received' event to Home Assistant.");

        std::map<std::string, std::string> data;
        char buf[24];

        snprintf(buf, sizeof(buf), "%" PRIu64, fixed);
        data["fixed_data"] = buf;

        snprintf(buf, sizeof(buf), "%u", (unsigned) rolling);
        data["rolling_code"] = buf;

        snprintf(buf, sizeof(buf), "%" PRIu64, remote_id);
        data["remote_id"] = buf;

        snprintf(buf, sizeof(buf), "%u", (unsigned) button);
        data["button"] = buf;

        if ( pincode.length() > 0 ) {
            data["pincode"] = pincode.c_str();
        }

        if (this->auth_enabled_) {
            data["authenticated"] = authenticated ? "true" : "false";
            if (authenticated) {
                snprintf(buf, sizeof(buf), "%d", slot + 1);
                data["slot"] = buf;
            }
        }

        this->fire_homeassistant_event("esphome.secplus_received", data);
    }


}

#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
    // No parameters needed; uses internal state safely
    std::string SecplusReceiverComponent::get_bits_string() {
        int count = this->manchester_pos_;
        if (count <= 0) return "[]";

        // Safety cap to prevent reading past your 64-byte (512-bit) buffer
        if (count > 512) count = 512; 

        std::string result;
        result.reserve(count * 2 + 1);
        result += "[";
        for (int i = 0; i < count; ++i) {
            result += (bitarr_get(this->manchester_, i) ? '1' : '0');
            if (i < count - 1) result += ",";
        }
        result += "]";
        return result;
    }
#endif
}  // namespace secplus_receiver
}  // namespace esphome
