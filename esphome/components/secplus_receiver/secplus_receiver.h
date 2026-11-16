/*
 * secplus_receiver.h
 *
 * ESPHome component that decodes Security+ 2.0 (Manchester-encoded OOK)
 * transmissions from Chamberlain / LiftMaster / Craftsman garage door
 * remotes.
 *
 * The component registers itself directly as a remote_receiver listener and
 * receives decoded pulse trains via on_receive(), so no on_raw lambda wiring
 * is needed in YAML. This mirrors the structure of the acurite component:
 *   https://github.com/swoboda1337/acurite-esphome
 *
 * The Manchester / Security+ v2 decode is implemented in secplus_receiver.cpp
 * on top of the upstream argilo/secplus C library (secplus.c / secplus.h),
 * which is downloaded at build time by __init__.py.
 */

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/remote_base/remote_base.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/api/custom_api_device.h"
#include "esphome/core/automation.h"
#include "esphome/core/preferences.h"

#include <vector>

#include "remote_store.h"

namespace esphome {
namespace secplus_receiver {

class SecplusReceiverComponent
    : public Component,
      public remote_base::RemoteReceiverListener,
      public api::CustomAPIDevice {

 public:
  void set_fixed_data_sensor(text_sensor::TextSensor *sensor) { this->fixed_data_sensor_ = sensor; }
  void set_rolling_code_sensor(text_sensor::TextSensor *sensor) { this->rolling_code_sensor_ = sensor; }
  void set_remote_id_sensor(text_sensor::TextSensor *sensor) { this->remote_id_sensor_ = sensor; }
  void set_button_sensor(text_sensor::TextSensor *sensor) { this->button_sensor_ = sensor; }
  void set_fire_event(bool fire) { this->fire_event_ = fire; }

  // ── Authentication (optional) ──────────────────────────────────────────────
  // Enables the enrolled-remote table with `learn_slots` learnable slots.
  void set_authentication(uint8_t learn_slots, uint32_t window, uint32_t learn_timeout_ms) {
    this->auth_enabled_ = true;
    this->window_ = window;
    this->learn_timeout_ms_ = learn_timeout_ms;
    this->store_.init(learn_slots);
  }
  // Remote fixed in YAML; must be called after set_authentication().
  void add_static_remote(uint64_t fixed) { this->store_.add_static(fixed); }
  // One sensor per learnable slot, in slot order.
  void add_slot_sensor(text_sensor::TextSensor *sensor) { this->slot_sensors_.push_back(sensor); }

  // Open learn mode: the next remote pressed twice is enrolled. Closes itself
  // after one remote or after the learn timeout.
  void start_learn();
  void stop_learn();
  bool is_learning() const { return this->store_.learning(); }
  // Remove the remote in a learnable slot. `slot` is 1-based, as shown in
  // sensor names, logs and events.
  void forget(uint8_t slot);
  void forget_all();

  void setup() override;
  void dump_config() override;
  bool on_receive(remote_base::RemoteReceiveData data) override;

  // Automations subscribe here (via the trigger class below)
  void add_on_remote_received_callback(
      std::function<void(uint64_t, uint64_t, uint8_t, uint32_t)> &&callback) {
    this->remote_received_callback_.add(std::move(callback));
  }

  // (fixed_data, remote_id, button, rolling, slot)
  void add_on_authenticated_callback(std::function<void(uint64_t, uint64_t, uint8_t, uint32_t, uint8_t)> &&callback) {
    this->authenticated_callback_.add(std::move(callback));
  }
  // (fixed_data, rolling, slot): enrolled remote whose counter was not accepted
  void add_on_rejected_callback(std::function<void(uint64_t, uint32_t, uint8_t)> &&callback) {
    this->rejected_callback_.add(std::move(callback));
  }
  // (fixed_data, slot)
  void add_on_remote_learned_callback(std::function<void(uint64_t, uint8_t)> &&callback) {
    this->remote_learned_callback_.add(std::move(callback));
  }


 protected:
  // Feed a burst of signed pulse durations (us) through the Manchester decoder.
  // Negative = signal LOW (space), positive = signal HIGH (mark).
  void decode_raw_(const int32_t *pulses, int n_pulses);

  // Fire the sensors / log line once a Security+ v2 packet pair is decoded.
  void publish_(uint32_t rolling, uint64_t fixed, uint32_t data, uint8_t frame_type);

  bool fire_event_{false};

  // To prevent/limit duplicates
  uint64_t last_fixed{0};
  uint32_t last_rolling{0};

  CallbackManager<void(uint64_t, uint64_t, uint8_t, uint32_t)> remote_received_callback_{};
  CallbackManager<void(uint64_t, uint64_t, uint8_t, uint32_t, uint8_t)> authenticated_callback_{};
  CallbackManager<void(uint64_t, uint32_t, uint8_t)> rejected_callback_{};
  CallbackManager<void(uint64_t, uint8_t)> remote_learned_callback_{};

  // ── Authentication state ───────────────────────────────────────────────────
  // Handle a packet while learn mode is open.
  void handle_learn_(uint64_t fixed, uint32_t rolling);
  // Write one slot to flash immediately and refresh its sensor.
  void save_slot_(size_t index);
  void publish_slot_(size_t index);

  bool auth_enabled_{false};
  uint32_t window_{1024};
  uint32_t learn_timeout_ms_{30000};
  RemoteStore store_;
  std::vector<ESPPreferenceObject> slot_prefs_;
  std::vector<text_sensor::TextSensor *> slot_sensors_;



#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
    std::string get_bits_string(); 
#endif


  // ── Manchester decoder state ───────────────────────────────────────────────
  // Retained across on_receive() calls so the two Security+ frames can be
  // accumulated even when they arrive in separate receive bursts.
  uint8_t manchester_[64]{};  // MANCHESTER_BYTES
  int manchester_pos_{0};
  uint8_t packet1_[32]{};  // MAX_DECODED_BYTES
  int packet1_len_{0};
  uint8_t packet2_[32]{};
  int packet2_len_{0};

  text_sensor::TextSensor *fixed_data_sensor_{nullptr};
  text_sensor::TextSensor *rolling_code_sensor_{nullptr};
  text_sensor::TextSensor *remote_id_sensor_{nullptr};
  text_sensor::TextSensor *button_sensor_{nullptr};
};


class RemoteReceivedTrigger : public Trigger<uint64_t, uint64_t, uint8_t, uint32_t> {
 public:
  explicit RemoteReceivedTrigger(SecplusReceiverComponent *parent) {
    parent->add_on_remote_received_callback(
        [this](uint64_t fixed_data, uint64_t remote_id, uint8_t button, uint32_t rolling) {
          this->trigger(fixed_data, remote_id, button, rolling);
        });
  }
};

class AuthenticatedTrigger : public Trigger<uint64_t, uint64_t, uint8_t, uint32_t, uint8_t> {
 public:
  explicit AuthenticatedTrigger(SecplusReceiverComponent *parent) {
    parent->add_on_authenticated_callback(
        [this](uint64_t fixed_data, uint64_t remote_id, uint8_t button, uint32_t rolling, uint8_t slot) {
          this->trigger(fixed_data, remote_id, button, rolling, slot);
        });
  }
};

class RejectedTrigger : public Trigger<uint64_t, uint32_t, uint8_t> {
 public:
  explicit RejectedTrigger(SecplusReceiverComponent *parent) {
    parent->add_on_rejected_callback(
        [this](uint64_t fixed_data, uint32_t rolling, uint8_t slot) { this->trigger(fixed_data, rolling, slot); });
  }
};

class RemoteLearnedTrigger : public Trigger<uint64_t, uint8_t> {
 public:
  explicit RemoteLearnedTrigger(SecplusReceiverComponent *parent) {
    parent->add_on_remote_learned_callback(
        [this](uint64_t fixed_data, uint8_t slot) { this->trigger(fixed_data, slot); });
  }
};

template<typename... Ts> class LearnAction : public Action<Ts...>, public Parented<SecplusReceiverComponent> {
 public:
  void play(const Ts &...x) override { this->parent_->start_learn(); }
};

template<typename... Ts> class StopLearnAction : public Action<Ts...>, public Parented<SecplusReceiverComponent> {
 public:
  void play(const Ts &...x) override { this->parent_->stop_learn(); }
};

template<typename... Ts> class ForgetAction : public Action<Ts...>, public Parented<SecplusReceiverComponent> {
 public:
  TEMPLATABLE_VALUE(uint8_t, slot)
  void play(const Ts &...x) override { this->parent_->forget(this->slot_.value(x...)); }
};

template<typename... Ts> class ForgetAllAction : public Action<Ts...>, public Parented<SecplusReceiverComponent> {
 public:
  void play(const Ts &...x) override { this->parent_->forget_all(); }
};

}  // namespace secplus_receiver
}  // namespace esphome
