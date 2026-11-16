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

  printf("%d checks, %d failures\n", checks, failures);
  return failures == 0 ? 0 : 1;
}
