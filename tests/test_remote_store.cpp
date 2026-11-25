/*
 * Host unit tests for remote_store.h.
 *
 * Build and run from the repository root:
 *   c++ -std=c++17 -Wall -Wextra -o /tmp/test_remote_store tests/test_remote_store.cpp && /tmp/test_remote_store
 */

#include <cstdio>

#include "../esphome/components/secplus_receiver/remote_store.h"

using namespace esphome::secplus_receiver;

static int failures = 0;
static int checks = 0;

#define CHECK(cond) \
  do { \
    checks++; \
    if (!(cond)) { \
      failures++; \
      printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
  } while (0)

static const uint64_t REMOTE_A = 70678577664ULL;
static const uint64_t REMOTE_B = 62088643072ULL;
static const uint32_t WINDOW = 1024;

// Enrol `fixed` via two presses at `rolling` and `rolling + 1`.
static int enrol(RemoteStore &store, uint64_t fixed, uint32_t rolling) {
  int slot = -1;
  store.start_learn();
  store.learn(fixed, rolling, &slot);
  store.learn(fixed, rolling + 1, &slot);
  return slot;
}

static void test_unknown_remote_is_not_authenticated() {
  RemoteStore store;
  store.init(4);
  int slot = 99;
  CHECK(store.check(REMOTE_A, 100, WINDOW, &slot) == AuthResult::UNKNOWN);
  CHECK(slot == -1);
}

static void test_learn_needs_two_presses() {
  RemoteStore store;
  store.init(4);
  int slot = -1;

  CHECK(store.learn(REMOTE_A, 100, &slot) == LearnResult::NOT_LEARNING);

  store.start_learn();
  CHECK(store.learn(REMOTE_A, 100, &slot) == LearnResult::PENDING);
  CHECK(store.find(REMOTE_A) == -1);
  CHECK(store.learning());

  CHECK(store.learn(REMOTE_A, 101, &slot) == LearnResult::LEARNED);
  CHECK(slot == 0);
  CHECK(store.find(REMOTE_A) == 0);
  CHECK(!store.learning());  // one remote per learn window
  CHECK(store.slot(0).rolling == 101);
}

static void test_learn_ignores_repeat_of_same_frame() {
  RemoteStore store;
  store.init(4);
  int slot = -1;
  store.start_learn();
  CHECK(store.learn(REMOTE_A, 100, &slot) == LearnResult::PENDING);
  // Same counter again (remote repeats a frame while the button is held).
  CHECK(store.learn(REMOTE_A, 100, &slot) == LearnResult::PENDING);
  CHECK(store.find(REMOTE_A) == -1);
}

static void test_learn_different_remote_restarts_candidate() {
  RemoteStore store;
  store.init(4);
  int slot = -1;
  store.start_learn();
  CHECK(store.learn(REMOTE_A, 100, &slot) == LearnResult::PENDING);
  CHECK(store.learn(REMOTE_B, 500, &slot) == LearnResult::PENDING);  // neighbour's stray press
  CHECK(store.learn(REMOTE_A, 101, &slot) == LearnResult::PENDING);  // A must start over
  CHECK(store.learn(REMOTE_A, 102, &slot) == LearnResult::LEARNED);
  CHECK(store.find(REMOTE_B) == -1);

  // A different remote whose counter happens to be one ahead confirms nothing.
  RemoteStore adjacent;
  adjacent.init(4);
  adjacent.start_learn();
  CHECK(adjacent.learn(REMOTE_A, 100, &slot) == LearnResult::PENDING);
  CHECK(adjacent.learn(REMOTE_B, 101, &slot) == LearnResult::PENDING);
  CHECK(adjacent.find(REMOTE_A) == -1 && adjacent.find(REMOTE_B) == -1);

  // The confirming press may be at most LEARN_MAX_STEP ahead.
  RemoteStore edge;
  edge.init(4);
  edge.start_learn();
  CHECK(edge.learn(REMOTE_A, 100, &slot) == LearnResult::PENDING);
  CHECK(edge.learn(REMOTE_A, 100 + LEARN_MAX_STEP, &slot) == LearnResult::LEARNED);
}

static void test_learn_rejects_large_or_backward_step() {
  RemoteStore store;
  store.init(4);
  int slot = -1;
  store.start_learn();
  store.learn(REMOTE_A, 100, &slot);
  CHECK(store.learn(REMOTE_A, 100 + LEARN_MAX_STEP + 1, &slot) == LearnResult::PENDING);
  CHECK(store.learn(REMOTE_A, 50, &slot) == LearnResult::PENDING);
  CHECK(store.find(REMOTE_A) == -1);
}

static void test_authenticate_within_window_and_reject_replay() {
  RemoteStore store;
  store.init(4);
  enrol(store, REMOTE_A, 100);  // stored counter is 101
  int slot = -1;

  // The enrolment presses themselves are replays.
  CHECK(store.check(REMOTE_A, 100, WINDOW, &slot) == AuthResult::REJECTED);
  CHECK(store.check(REMOTE_A, 101, WINDOW, &slot) == AuthResult::REJECTED);
  CHECK(slot == 0);

  CHECK(store.check(REMOTE_A, 102, WINDOW, &slot) == AuthResult::AUTHENTICATED);
  CHECK(slot == 0);
  CHECK(store.slot(0).rolling == 102);

  // Replaying the accepted code fails.
  CHECK(store.check(REMOTE_A, 102, WINDOW, &slot) == AuthResult::REJECTED);

  // Presses made out of range skip counts; still accepted inside the window.
  CHECK(store.check(REMOTE_A, 102 + WINDOW, WINDOW, &slot) == AuthResult::AUTHENTICATED);
  // An older recording captured in between is now a replay.
  CHECK(store.check(REMOTE_A, 500, WINDOW, &slot) == AuthResult::REJECTED);
  // Beyond the window is rejected and leaves the counter alone.
  uint32_t last = store.slot(0).rolling;
  CHECK(store.check(REMOTE_A, last + WINDOW + 1, WINDOW, &slot) == AuthResult::REJECTED);
  CHECK(store.slot(0).rolling == last);
}

static void test_counter_wraps_at_28_bits() {
  RemoteStore store;
  store.init(4);
  enrol(store, REMOTE_A, ROLLING_MASK - 2);  // stored counter is ROLLING_MASK - 1
  int slot = -1;
  CHECK(store.check(REMOTE_A, ROLLING_MASK, WINDOW, &slot) == AuthResult::AUTHENTICATED);
  CHECK(store.check(REMOTE_A, 0, WINDOW, &slot) == AuthResult::AUTHENTICATED);
  CHECK(store.check(REMOTE_A, ROLLING_MASK, WINDOW, &slot) == AuthResult::REJECTED);
}

static void test_slots_fill_and_full() {
  RemoteStore store;
  store.init(2);
  CHECK(enrol(store, REMOTE_A, 10) == 0);
  CHECK(enrol(store, REMOTE_B, 20) == 1);

  int slot = -1;
  store.start_learn();
  store.learn(12345, 1, &slot);
  CHECK(store.learn(12345, 2, &slot) == LearnResult::FULL);
  CHECK(slot == -1);
  CHECK(!store.learning());
}

static void test_forget_frees_slot_and_revokes() {
  RemoteStore store;
  store.init(2);
  enrol(store, REMOTE_A, 10);
  enrol(store, REMOTE_B, 20);
  int slot = -1;

  CHECK(store.forget(0));
  CHECK(!store.occupied(0));
  CHECK(store.check(REMOTE_A, 12, WINDOW, &slot) == AuthResult::UNKNOWN);
  CHECK(store.check(REMOTE_B, 22, WINDOW, &slot) == AuthResult::AUTHENTICATED);  // untouched

  CHECK(!store.forget(0));   // already empty
  CHECK(!store.forget(99));  // out of range

  CHECK(enrol(store, 12345, 1) == 0);  // freed slot is reused
}

static void test_static_remote_trusts_first_use_and_cannot_be_forgotten() {
  RemoteStore store;
  store.init(2);
  size_t idx = store.add_static(REMOTE_A);
  CHECK(idx == 2);
  CHECK(store.is_static(idx));
  int slot = -1;

  CHECK(store.check(REMOTE_A, 5000, WINDOW, &slot) == AuthResult::AUTHENTICATED);
  CHECK(slot == 2);
  CHECK(store.check(REMOTE_A, 5000, WINDOW, &slot) == AuthResult::REJECTED);
  CHECK(store.check(REMOTE_A, 5001, WINDOW, &slot) == AuthResult::AUTHENTICATED);
  CHECK(!store.forget(idx));
  CHECK(store.occupied(idx));
}

static void test_resync_moves_forward_only() {
  RemoteStore store;
  store.init(2);
  enrol(store, REMOTE_A, 100);  // stored counter is 101
  int slot = -1;

  // Far beyond the window: normal presses are rejected...
  CHECK(store.check(REMOTE_A, 50000, WINDOW, &slot) == AuthResult::REJECTED);
  // ...but two presses in learn mode move the counter forward.
  store.start_learn();
  CHECK(store.learn(REMOTE_A, 50001, &slot) == LearnResult::PENDING);
  CHECK(store.learn(REMOTE_A, 50002, &slot) == LearnResult::RESYNCED);
  CHECK(slot == 0);
  CHECK(store.slot(0).rolling == 50002);
  CHECK(!store.learning());
  CHECK(store.find(REMOTE_A) == 0);  // no duplicate slot

  // Two old consecutive recordings cannot wind the counter back.
  store.start_learn();
  CHECK(store.learn(REMOTE_A, 200, &slot) == LearnResult::PENDING);
  CHECK(store.learn(REMOTE_A, 201, &slot) == LearnResult::REJECTED);
  CHECK(store.slot(0).rolling == 50002);
}

static void test_restore_from_flash() {
  RemoteStore saved_from;
  saved_from.init(2);
  enrol(saved_from, REMOTE_A, 100);
  RemoteSlot learned = saved_from.slot(0);

  RemoteStore store;
  store.init(2);
  size_t st = store.add_static(REMOTE_B);
  int slot = -1;

  store.restore(0, learned);
  store.restore(1, RemoteSlot{0, 0, 0});  // empty slot stays empty
  CHECK(store.find(REMOTE_A) == 0);
  CHECK(!store.occupied(1));
  CHECK(store.check(REMOTE_A, 101, WINDOW, &slot) == AuthResult::REJECTED);  // counter survived
  CHECK(store.check(REMOTE_A, 102, WINDOW, &slot) == AuthResult::AUTHENTICATED);

  // Static slot: counter restored only when saved for the same fixed code.
  store.restore(st, RemoteSlot{12345, 900, (uint8_t) (SLOT_OCCUPIED | SLOT_HAS_COUNTER)});
  CHECK(store.check(REMOTE_B, 10, WINDOW, &slot) == AuthResult::AUTHENTICATED);  // still first use
  store.restore(st, RemoteSlot{REMOTE_B, 900, (uint8_t) (SLOT_OCCUPIED | SLOT_HAS_COUNTER)});
  CHECK(store.check(REMOTE_B, 900, WINDOW, &slot) == AuthResult::REJECTED);
  CHECK(store.check(REMOTE_B, 901, WINDOW, &slot) == AuthResult::AUTHENTICATED);

  // A learned copy of a code that is now static is dropped, not duplicated.
  RemoteStore store2;
  store2.init(2);
  store2.add_static(REMOTE_A);
  store2.restore(0, learned);
  CHECK(!store2.occupied(0));
  CHECK(store2.find(REMOTE_A) == 2);
}

static void test_static_remote_recovers_after_counter_pushed_ahead() {
  RemoteStore store;
  store.init(2);
  size_t idx = store.add_static(REMOTE_A);
  int slot = -1;
  CHECK(!store.reset_counter(idx));  // no counter yet
  CHECK(store.check(REMOTE_A, 100, WINDOW, &slot) == AuthResult::AUTHENTICATED);

  // Forged codes walk the stored counter ahead of the real remote.
  CHECK(store.check(REMOTE_A, 100 + WINDOW, WINDOW, &slot) == AuthResult::AUTHENTICATED);
  CHECK(store.check(REMOTE_A, 100 + 2 * WINDOW, WINDOW, &slot) == AuthResult::AUTHENTICATED);
  // The real remote is now behind: rejected, and learn mode will not resync it.
  CHECK(store.check(REMOTE_A, 101, WINDOW, &slot) == AuthResult::REJECTED);
  store.start_learn();
  CHECK(store.learn(REMOTE_A, 102, &slot) == LearnResult::PENDING);
  CHECK(store.learn(REMOTE_A, 103, &slot) == LearnResult::REJECTED);
  CHECK(store.learning());  // a rejected resync leaves learn mode open
  store.stop_learn();

  // Resetting the counter returns the slot to trust-on-first-use, at any
  // counter: the real remote may be far beyond the window from zero.
  CHECK(store.reset_counter(idx));
  CHECK(store.occupied(idx));
  CHECK(store.check(REMOTE_A, 50000, WINDOW, &slot) == AuthResult::AUTHENTICATED);
  CHECK(store.check(REMOTE_A, 50000, WINDOW, &slot) == AuthResult::REJECTED);
  CHECK(store.check(REMOTE_A, 50001 + WINDOW, WINDOW, &slot) == AuthResult::REJECTED);  // window is back

  CHECK(!store.reset_counter(0));   // learn slots are forgotten, not reset
  CHECK(!store.reset_counter(99));  // out of range
}

static void test_static_remote_resyncs_forward_in_learn_mode() {
  RemoteStore store;
  store.init(1);
  size_t idx = store.add_static(REMOTE_A);
  int slot = -1;
  CHECK(store.check(REMOTE_A, 100, WINDOW, &slot) == AuthResult::AUTHENTICATED);
  store.start_learn();
  CHECK(store.learn(REMOTE_A, 70000, &slot) == LearnResult::PENDING);
  CHECK(store.learn(REMOTE_A, 70001, &slot) == LearnResult::RESYNCED);
  CHECK(slot == (int) idx);
  CHECK(store.slot(idx).rolling == 70001);
}

static void test_resync_forward_limit_boundary() {
  const uint32_t stored = 101;
  for (int over = 0; over <= 1; over++) {
    RemoteStore store;
    store.init(1);
    enrol(store, REMOTE_A, 100);  // stored counter is 101
    int slot = -1;
    // Second press lands FORWARD_LIMIT - 1 (allowed) or FORWARD_LIMIT (refused) ahead.
    uint32_t second = (stored + FORWARD_LIMIT - 1 + (uint32_t) over) & ROLLING_MASK;
    store.start_learn();
    CHECK(store.learn(REMOTE_A, (second - 1) & ROLLING_MASK, &slot) == LearnResult::PENDING);
    CHECK(store.learn(REMOTE_A, second, &slot) == (over ? LearnResult::REJECTED : LearnResult::RESYNCED));
    CHECK(store.slot(0).rolling == (over ? stored : second));
  }
}

static void test_resync_rejects_counter_equal_to_stored() {
  RemoteStore store;
  store.init(1);
  enrol(store, REMOTE_A, 100);  // stored counter is 101
  int slot = -1;
  store.start_learn();
  CHECK(store.learn(REMOTE_A, 100, &slot) == LearnResult::PENDING);
  CHECK(store.learn(REMOTE_A, 101, &slot) == LearnResult::REJECTED);  // not ahead of 101
}

static void test_restore_sanitises_saved_records() {
  const uint8_t both = SLOT_OCCUPIED | SLOT_HAS_COUNTER;
  RemoteStore store;
  store.init(2);
  int slot = -1;

  // Bits above the 28-bit counter are dropped.
  store.restore(0, RemoteSlot{REMOTE_A, 0xF0000064u, both});
  CHECK(store.slot(0).rolling == 100);
  CHECK(store.check(REMOTE_A, 101, WINDOW, &slot) == AuthResult::AUTHENTICATED);

  // A learned record without a counter would be trusted at any counter: not loaded.
  store.restore(1, RemoteSlot{REMOTE_B, 0, SLOT_OCCUPIED});
  CHECK(!store.occupied(1));
  CHECK(store.check(REMOTE_B, 999999, WINDOW, &slot) == AuthResult::UNKNOWN);
}

static void test_restore_carries_counter_from_learned_copy_to_static() {
  const uint8_t both = SLOT_OCCUPIED | SLOT_HAS_COUNTER;

  // REMOTE_A was learned (counter 500), then added to the static list.
  RemoteStore store;
  store.init(2);
  size_t idx = store.add_static(REMOTE_A);
  CHECK(store.restore(0, RemoteSlot{REMOTE_A, 500, both}) == (int) idx);
  CHECK(!store.occupied(0));
  int slot = -1;
  CHECK(store.check(REMOTE_A, 400, WINDOW, &slot) == AuthResult::REJECTED);  // old capture
  CHECK(store.check(REMOTE_A, 501, WINDOW, &slot) == AuthResult::AUTHENTICATED);

  // The static slot's own saved counter never moves the counter backwards...
  RemoteStore newer_learned;
  newer_learned.init(2);
  idx = newer_learned.add_static(REMOTE_A);
  newer_learned.restore(0, RemoteSlot{REMOTE_A, 500, both});
  CHECK(newer_learned.restore(idx, RemoteSlot{REMOTE_A, 300, both}) == -1);
  CHECK(newer_learned.slot(idx).rolling == 500);

  // ...but does move it forwards.
  RemoteStore newer_static;
  newer_static.init(2);
  idx = newer_static.add_static(REMOTE_A);
  newer_static.restore(0, RemoteSlot{REMOTE_A, 500, both});
  newer_static.restore(idx, RemoteSlot{REMOTE_A, 900, both});
  CHECK(newer_static.slot(idx).rolling == 900);

  // The same remote saved in two learn slots keeps one copy, newest counter.
  RemoteStore dup;
  dup.init(2);
  CHECK(dup.restore(0, RemoteSlot{REMOTE_B, 10, both}) == -1);
  CHECK(dup.restore(1, RemoteSlot{REMOTE_B, 20, both}) == 0);
  CHECK(!dup.occupied(1));
  CHECK(dup.slot(0).rolling == 20);
}

int main() {
  test_unknown_remote_is_not_authenticated();
  test_learn_needs_two_presses();
  test_learn_ignores_repeat_of_same_frame();
  test_learn_different_remote_restarts_candidate();
  test_learn_rejects_large_or_backward_step();
  test_authenticate_within_window_and_reject_replay();
  test_counter_wraps_at_28_bits();
  test_slots_fill_and_full();
  test_forget_frees_slot_and_revokes();
  test_static_remote_trusts_first_use_and_cannot_be_forgotten();
  test_resync_moves_forward_only();
  test_restore_from_flash();
  test_static_remote_recovers_after_counter_pushed_ahead();
  test_static_remote_resyncs_forward_in_learn_mode();
  test_resync_forward_limit_boundary();
  test_resync_rejects_counter_equal_to_stored();
  test_restore_sanitises_saved_records();
  test_restore_carries_counter_from_learned_copy_to_static();

  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
