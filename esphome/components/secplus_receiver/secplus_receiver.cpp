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
#include "esphome/core/log.h"

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
// unknown header values / decode overflow.
static int process_packet(const uint8_t *bits, int nbits, uint8_t *packet, int *packet_length) {
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
  if (!((b0 == 1 && b1 == 0 && b2 == 1 && b3 == 0) || (b0 == 1 && b1 == 0 && b2 == 0 && b3 == 1))) {
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
    this->slot_prefs_.push_back(global_preferences->make_preference<RemoteSlot>(key));
    RemoteSlot saved;
    if (this->slot_prefs_[i].load(&saved)) {
      this->store_.restore(i, saved);
    }
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

void SecplusReceiverComponent::forget(uint8_t slot) {
  if (slot < 1 || !this->store_.forget((size_t) slot - 1)) {
    ESP_LOGW(TAG, "Cannot forget slot %u: not a learned remote", slot);
    return;
  }
  this->save_slot_((size_t) slot - 1);
  ESP_LOGI(TAG, "Forgot remote in slot %u", slot);
}

void SecplusReceiverComponent::forget_all() {
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
      ESP_LOGW(TAG, "Learn failed: all %u slots are in use, forget one first", (unsigned) this->store_.learn_slots());
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

    // ── Gap or buffer-full: attempt decode ──
    if (dur >= LONG_THRESH || flush) {
      if (flush) {
        ESP_LOGW(TAG, "Manchester buffer full (%d bits), flushing. Could mean a noisy radio, try another frequency",
                 this->manchester_pos_);
      }

      if (this->manchester_pos_ < MAX_MANCHESTER) {
        bitarr_set(this->manchester_, this->manchester_pos_++, lvl);
      }

      bool is_packet_1 = (this->packet1_len_ == 0);



      int frame_id = is_packet_1
                         ? process_packet(this->manchester_, this->manchester_pos_, this->packet1_, &this->packet1_len_)
                         : process_packet(this->manchester_, this->manchester_pos_, this->packet2_, &this->packet2_len_);

      if (frame_id > 0) {
        ESP_LOGD(TAG, "frame #%d decoded (%d bits)", frame_id,
                 frame_id == 1 ? this->packet1_len_ : this->packet2_len_);
      }

      // Print out the raw Manchester for VERBOSE logging
      ESP_LOGV(TAG, "%s", this->get_bits_string().c_str());

      // Reject out-of-order frame 2 arriving before frame 1.
      if (is_packet_1 && frame_id == 2) {
        ESP_LOGW(TAG, "got frame 2 before frame 1, discarding");
        this->packet2_len_ = 0;
        frame_id = -1;
      }

      // Both frames received: run decode_v2 and publish.
      if (frame_id == 2) {
        uint8_t frame_type = (this->packet1_len_ == 40) ? 0 : 1;

        uint32_t rolling = 0;
        uint64_t fixed = 0;
        uint32_t data = 0;

        int ret = decode_v2(frame_type, this->packet1_, this->packet2_, &rolling, &fixed, &data);
        if (ret < 0) {
          ESP_LOGE(TAG, "secplus decode_v2 failed and returned: %d", ret);
        } else {
          this->publish_(rolling, fixed, data, frame_type);
        }

        this->packet1_len_ = this->packet2_len_ = 0;
      }

      this->manchester_pos_ = 0;
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
            pincode = esphome::str_sprintf("%04u%s", pin, suffix);
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
                case AuthResult::REJECTED:
                    ESP_LOGW(TAG,
                             "Rejected remote in slot %d: rolling=%u is not within %u ahead of stored %u "
                             "(replay, or remote too far ahead; re-learn to resync)",
                             slot + 1, (unsigned) rolling, (unsigned) this->window_,
                             (unsigned) this->store_.slot(slot).rolling);
                    this->rejected_callback_.call(fixed, rolling, (uint8_t) (slot + 1));
                    break;
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
