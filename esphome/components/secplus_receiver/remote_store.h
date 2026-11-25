/*
 * remote_store.h
 *
 * Table of enrolled Security+ 2.0 remotes with rolling-counter replay
 * protection and a two-press learn mode.
 *
 * This header deliberately has no ESPHome dependencies so the logic can be
 * unit-tested on a host machine (see tests/test_remote_store.cpp). The
 * component owns persistence: after any call that reports a change, it saves
 * the affected slot to flash.
 *
 * Security note: Security+ 2.0 has no secret key. This table provides an
 * allowlist plus replay protection, which is the same protection a real
 * opener offers. It cannot stop an attacker who captures a transmission and
 * forges the next counter value.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace esphome {
namespace secplus_receiver {

// The Security+ 2.0 rolling code is a 28-bit counter.
static const uint32_t ROLLING_MASK = 0x0FFFFFFFu;

// In learn mode the second press must be at most this many counts after the
// first. Remotes increment by one per press; a little slack tolerates a
// press the receiver missed.
static const uint32_t LEARN_MAX_STEP = 16;

// A counter more than half the counter space "ahead" is really behind.
static const uint32_t FORWARD_LIMIT = (ROLLING_MASK + 1) / 2;

static const uint8_t SLOT_OCCUPIED = 0x01;     // slot holds a remote
static const uint8_t SLOT_HAS_COUNTER = 0x02;  // rolling field is valid

// Persisted as-is; keep the layout stable.
struct RemoteSlot {
  uint64_t fixed;
  uint32_t rolling;
  uint8_t flags;
} __attribute__((packed));

enum class AuthResult : uint8_t {
  UNKNOWN,        // fixed code is not enrolled
  REJECTED,       // enrolled, but counter is not ahead within the window (replay or too far ahead)
  AUTHENTICATED,  // enrolled and counter accepted; slot counter was advanced
};

enum class LearnResult : uint8_t {
  NOT_LEARNING,  // learn mode is closed
  PENDING,       // first press recorded, waiting for a confirming press
  LEARNED,       // new remote stored in a free slot
  RESYNCED,      // already-enrolled remote: counter moved forward
  REJECTED,      // already-enrolled remote presented a counter that is not ahead
  FULL,          // no free learn slot
};

class RemoteStore {
 public:
  // Create `learn_slots` empty learnable slots. Call once, before add_static().
  void init(size_t learn_slots) {
    this->slots_.assign(learn_slots, RemoteSlot{0, 0, 0});
    this->learn_slots_ = learn_slots;
  }

  // Append a remote that is fixed in configuration. It cannot be forgotten at
  // runtime; its first counter is trusted on first use. Returns its index.
  size_t add_static(uint64_t fixed) {
    this->slots_.push_back(RemoteSlot{fixed, 0, SLOT_OCCUPIED});
    return this->slots_.size() - 1;
  }

  size_t size() const { return this->slots_.size(); }
  size_t learn_slots() const { return this->learn_slots_; }
  bool is_static(size_t i) const { return i >= this->learn_slots_; }
  bool occupied(size_t i) const { return i < this->slots_.size() && (this->slots_[i].flags & SLOT_OCCUPIED); }
  const RemoteSlot &slot(size_t i) const { return this->slots_[i]; }

  // Apply a slot loaded from flash. Learn slots take the saved value whole.
  // Static slots only take the counter, and only if it was saved for the
  // same fixed code that is configured now.
  //
  // Returns -1 normally. If the saved learn slot duplicates a remote that is
  // already in the table (typically one that has since been added to the
  // static list), the copy is dropped, its counter is carried over so replay
  // protection is not lost, and the index of the slot that absorbed it is
  // returned. The caller should then persist both slot `i` (now empty) and
  // the returned slot.
  int restore(size_t i, const RemoteSlot &saved) {
    if (i >= this->slots_.size() || !(saved.flags & SLOT_OCCUPIED)) {
      return -1;
    }
    if (this->is_static(i)) {
      if (saved.fixed == this->slots_[i].fixed) {
        this->adopt_counter_(this->slots_[i], saved);
      }
      return -1;
    }
    int existing = this->find(saved.fixed);
    if (existing >= 0) {
      this->adopt_counter_(this->slots_[existing], saved);
      return existing;
    }
    // A learned remote always has a counter. A record without one is corrupt
    // and would be trusted at any counter, so it is not loaded.
    if (!(saved.flags & SLOT_HAS_COUNTER)) {
      return -1;
    }
    this->slots_[i] = RemoteSlot{saved.fixed, saved.rolling & ROLLING_MASK, (uint8_t) (SLOT_OCCUPIED | SLOT_HAS_COUNTER)};
    return -1;
  }

  // Index of the slot holding `fixed`, or -1.
  int find(uint64_t fixed) const {
    for (size_t i = 0; i < this->slots_.size(); i++) {
      if ((this->slots_[i].flags & SLOT_OCCUPIED) && this->slots_[i].fixed == fixed) {
        return (int) i;
      }
    }
    return -1;
  }

  // Check a received code. On AUTHENTICATED the slot counter is advanced and
  // the caller must persist `*slot`. `*slot` is set whenever the remote is
  // enrolled (AUTHENTICATED or REJECTED), otherwise -1.
  AuthResult check(uint64_t fixed, uint32_t rolling, uint32_t window, int *slot) {
    rolling &= ROLLING_MASK;
    int idx = this->find(fixed);
    *slot = idx;
    if (idx < 0) {
      return AuthResult::UNKNOWN;
    }
    RemoteSlot &s = this->slots_[idx];
    if (s.flags & SLOT_HAS_COUNTER) {
      uint32_t ahead = (rolling - s.rolling) & ROLLING_MASK;
      if (ahead < 1 || ahead > window) {
        return AuthResult::REJECTED;
      }
    }
    s.rolling = rolling;
    s.flags |= SLOT_HAS_COUNTER;
    return AuthResult::AUTHENTICATED;
  }

  void start_learn() {
    this->learning_ = true;
    this->candidate_valid_ = false;
  }
  void stop_learn() {
    this->learning_ = false;
    this->candidate_valid_ = false;
  }
  bool learning() const { return this->learning_; }

  // Feed a received code while learn mode is open. A remote is accepted only
  // after two presses: the same fixed code with the counter moving forward.
  // On LEARNED / RESYNCED learn mode closes and the caller must persist `*slot`.
  LearnResult learn(uint64_t fixed, uint32_t rolling, int *slot) {
    *slot = -1;
    if (!this->learning_) {
      return LearnResult::NOT_LEARNING;
    }
    rolling &= ROLLING_MASK;

    uint32_t step = (rolling - this->candidate_rolling_) & ROLLING_MASK;
    bool confirmed =
        this->candidate_valid_ && this->candidate_fixed_ == fixed && step >= 1 && step <= LEARN_MAX_STEP;
    if (!confirmed) {
      this->candidate_valid_ = true;
      this->candidate_fixed_ = fixed;
      this->candidate_rolling_ = rolling;
      return LearnResult::PENDING;
    }

    int idx = this->find(fixed);
    if (idx >= 0) {
      // Already enrolled: allow the counter to move forward only, so a pair of
      // old recordings cannot be used to wind it back.
      RemoteSlot &s = this->slots_[idx];
      *slot = idx;
      if (s.flags & SLOT_HAS_COUNTER) {
        uint32_t ahead = (rolling - s.rolling) & ROLLING_MASK;
        if (ahead < 1 || ahead >= FORWARD_LIMIT) {
          this->candidate_valid_ = false;
          return LearnResult::REJECTED;
        }
      }
      s.rolling = rolling;
      s.flags |= SLOT_HAS_COUNTER;
      this->stop_learn();
      return LearnResult::RESYNCED;
    }

    for (size_t i = 0; i < this->learn_slots_; i++) {
      if (!(this->slots_[i].flags & SLOT_OCCUPIED)) {
        this->slots_[i] = RemoteSlot{fixed, rolling, (uint8_t) (SLOT_OCCUPIED | SLOT_HAS_COUNTER)};
        *slot = (int) i;
        this->stop_learn();
        return LearnResult::LEARNED;
      }
    }
    this->stop_learn();
    return LearnResult::FULL;
  }

  // Clear one learnable slot. Returns false if the index is out of range,
  // static, or already empty. The caller must persist the slot on true.
  bool forget(size_t i) {
    if (i >= this->learn_slots_ || !(this->slots_[i].flags & SLOT_OCCUPIED)) {
      return false;
    }
    this->slots_[i] = RemoteSlot{0, 0, 0};
    return true;
  }

  // Discard the stored counter of a static remote, so its next code is
  // trusted again as on first use. This is the only way to recover a static
  // remote whose stored counter has been pushed ahead of the real remote.
  // Returns false if the index is not an occupied static slot with a counter.
  // The caller must persist the slot on true.
  bool reset_counter(size_t i) {
    if (i >= this->slots_.size() || !this->is_static(i) || !(this->slots_[i].flags & SLOT_HAS_COUNTER)) {
      return false;
    }
    this->slots_[i].rolling = 0;
    this->slots_[i].flags &= (uint8_t) ~SLOT_HAS_COUNTER;
    return true;
  }

 protected:
  // Take over a saved counter, but never move an existing counter backwards.
  static void adopt_counter_(RemoteSlot &s, const RemoteSlot &saved) {
    if (!(saved.flags & SLOT_HAS_COUNTER)) {
      return;
    }
    uint32_t rolling = saved.rolling & ROLLING_MASK;
    if (s.flags & SLOT_HAS_COUNTER) {
      uint32_t ahead = (rolling - s.rolling) & ROLLING_MASK;
      if (ahead < 1 || ahead >= FORWARD_LIMIT) {
        return;
      }
    }
    s.rolling = rolling;
    s.flags |= SLOT_HAS_COUNTER;
  }

  std::vector<RemoteSlot> slots_;
  size_t learn_slots_{0};

  bool learning_{false};
  bool candidate_valid_{false};
  uint64_t candidate_fixed_{0};
  uint32_t candidate_rolling_{0};
};

}  // namespace secplus_receiver
}  // namespace esphome
