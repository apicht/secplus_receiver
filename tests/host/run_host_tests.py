#!/usr/bin/env python3
"""End-to-end tests for secplus_receiver on ESPHome's native `host` platform.

Compiles tests/host/secplus_host.yaml into an ordinary program, runs it, and
drives it over the real ESPHome native API the way Home Assistant would:
pressing buttons, calling actions, and reading sensor states and events.
Remote presses are injected as genuine Security+ 2.0 pulse trains, so the
Manchester decoder, authentication, learn mode, and flash persistence are all
exercised. No ESP32 or radio is needed. The radio itself is NOT tested.

Requirements: Python with `esphome` installed (it brings `aioesphomeapi`) and
a C++ compiler. Linux or macOS.

    python tests/host/run_host_tests.py            # build, then test
    python tests/host/run_host_tests.py --no-build # reuse the last build
"""

from __future__ import annotations

import argparse
import asyncio
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile

from aioesphomeapi import APIClient

HERE = Path(__file__).resolve().parent
CONFIG = HERE / "secplus_host.yaml"
BINARY = HERE / ".esphome/build/secplus-host/.pioenvs/secplus-host/program"
PORT = 16053  # must match api.port in secplus_host.yaml
TIMEOUT = 5.0
LEARN_TIMEOUT = 2.0  # must match learn_timeout in secplus_host.yaml
WINDOW = 16  # must match window in secplus_host.yaml
REPEAT_WINDOW = 1.5  # must match REPEAT_WINDOW_MS in secplus_receiver.cpp

REMOTE_A = 70678577664
REMOTE_B = 62088643072
STATIC_REMOTE = 123456789  # listed under `remotes:`; slot 3 after the 2 learn slots


class Device:
    """One running instance of the host build plus an API connection to it."""

    def __init__(self, prefdir: str) -> None:
        self.prefdir = prefdir
        self.proc: subprocess.Popen | None = None
        self.client: APIClient | None = None
        self.states: dict[int, str] = {}
        self.events: list = []
        self.keys: dict[str, int] = {}
        self.services: dict[str, object] = {}

    async def start(self) -> None:
        env = dict(os.environ, ESPHOME_PREFDIR=self.prefdir)
        self.proc = subprocess.Popen(
            [str(BINARY)], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        self.states.clear()
        self.events.clear()
        deadline = asyncio.get_running_loop().time() + TIMEOUT
        while True:
            self.client = APIClient("127.0.0.1", PORT, None)
            try:
                await self.client.connect(login=True, log_errors=False)
                break
            except Exception:  # noqa: BLE001 - device still starting
                if self.proc.poll() is not None:
                    raise RuntimeError("host program exited during startup") from None
                if asyncio.get_running_loop().time() > deadline:
                    raise
                await asyncio.sleep(0.1)
        entities, services = await self.client.list_entities_services()
        self.keys = {e.name: e.key for e in entities}
        self.services = {s.name: s for s in services}
        self.client.subscribe_states(self._on_state)
        self.client.subscribe_service_calls(self.events.append)

    def _on_state(self, state) -> None:
        if hasattr(state, "state"):
            self.states[state.key] = state.state

    async def stop(self, kill: bool = False) -> None:
        if self.client is not None:
            await self.client.disconnect(force=True)
            self.client = None
        if self.proc is not None:
            if kill:
                self.proc.kill()  # no chance to flush anything: like a power cut
            else:
                self.proc.terminate()
            self.proc.wait(timeout=TIMEOUT)
            self.proc = None

    async def restart(self, kill: bool = False) -> None:
        await self.stop(kill=kill)
        await self.start()

    async def wait_for(self, predicate, what: str):
        deadline = asyncio.get_running_loop().time() + TIMEOUT
        while True:
            result = predicate()
            if result:
                return result
            if asyncio.get_running_loop().time() > deadline:
                raise AssertionError(f"timed out waiting for {what}")
            await asyncio.sleep(0.02)

    def slot(self, number: int) -> str | None:
        return self.states.get(self.keys[f"Remote Slot {number}"])

    async def wait_slot(self, number: int, expected: str) -> None:
        await self.wait_for(
            lambda: self.slot(number) == expected,
            f"Remote Slot {number} == {expected!r} (is {self.slot(number)!r})",
        )

    def push_button(self, name: str) -> None:
        self.client.button_command(self.keys[name])

    async def forget(self, slot: int) -> None:
        await self.client.execute_service(self.services["forget_remote"], {"slot": slot})

    async def press(self, fixed: int, rolling: int, pin: int | None = None) -> dict[str, str]:
        """Simulate a remote press; return the esphome.secplus_received event data.

        With `pin`, simulate a keypad entry (64-bit frame) instead.
        """
        start = len(self.events)
        if pin is None:
            await self.client.execute_service(
                self.services["press"], {"fixed": str(fixed), "rolling": rolling}
            )
        else:
            await self.client.execute_service(
                self.services["press_keypad"],
                {"fixed": str(fixed), "rolling": rolling, "pin": pin},
            )

        def find():
            for event in self.events[start:]:
                if (
                    event.service == "esphome.secplus_received"
                    and event.data.get("fixed_data") == str(fixed)
                    and event.data.get("rolling_code") == str(rolling)
                ):
                    return event.data
            return None

        return await self.wait_for(find, f"event for fixed={fixed} rolling={rolling}")

    async def inject(self, fixed: int, rolling: int, frames: str) -> None:
        """Feed chosen frames of one transmission as a single burst (see host_harness.h)."""
        await self.client.execute_service(
            self.services["inject"], {"fixed": str(fixed), "rolling": rolling, "frames": frames}
        )

    async def set_require_repeat(self, enabled: bool) -> None:
        await self.client.execute_service(self.services["set_require_repeat"], {"enabled": enabled})

    async def decoded_since(self, start: int) -> list[tuple[int, int]]:
        """(fixed, rolling) of every code published since event index `start`."""
        await asyncio.sleep(0.3)  # let anything unexpected arrive too
        return [
            (int(e.data["fixed_data"]), int(e.data["rolling_code"]))
            for e in self.events[start:]
            if e.service == "esphome.secplus_received"
        ]

    def learned_events(self) -> list[dict[str, str]]:
        return [e.data for e in self.events if e.service == "esphome.secplus_remote_learned"]

    def trigger_events(self, rolling: int) -> list[tuple[str, dict[str, str]]]:
        """Trigger automations that fired for the press with this counter, in order."""
        return [
            (e.service.removeprefix("esphome.test_"), dict(e.data))
            for e in self.events
            if e.service.startswith("esphome.test_") and e.data.get("rolling") == str(rolling)
        ]

    async def wait_triggers(self, rolling: int, expected: list[tuple[str, dict[str, str]]]) -> None:
        await self.wait_for(
            lambda: len(self.trigger_events(rolling)) >= len(expected),
            f"{len(expected)} trigger(s) for rolling={rolling}",
        )
        await asyncio.sleep(0.1)  # let any unexpected extra trigger arrive
        assert self.trigger_events(rolling) == expected, self.trigger_events(rolling)


def expect_auth(event: dict[str, str], slot: int | None) -> None:
    """Assert the event is authenticated for `slot`, or unauthenticated if None."""
    if slot is None:
        assert event.get("authenticated") == "false", event
        assert "slot" not in event, event
    else:
        assert event.get("authenticated") == "true", event
        assert event.get("slot") == str(slot), event


async def learn(dev: Device, fixed: int, rolling: int) -> None:
    """Enrol a remote: open learn mode and press twice."""
    dev.push_button("Learn Remote")
    await asyncio.sleep(0.2)
    expect_auth(await dev.press(fixed, rolling), None)
    expect_auth(await dev.press(fixed, rolling + 1), None)


# ── Scenarios. They run in order against one device and share its state. ─────


async def test_boot_state(dev: Device) -> None:
    await dev.wait_slot(1, "empty")
    await dev.wait_slot(2, "empty")
    assert "Remote Slot 3" not in dev.keys  # static remotes get no sensor


async def test_decode_and_unknown_remote(dev: Device) -> None:
    event = await dev.press(REMOTE_A, 100)  # proves the pulse train decodes
    expect_auth(event, None)
    await dev.wait_for(
        lambda: dev.states.get(dev.keys["Fixed Data"]) == str(REMOTE_A), "Fixed Data sensor"
    )
    await dev.wait_for(
        lambda: dev.states.get(dev.keys["Rolling Code"]) == "100", "Rolling Code sensor"
    )


# Decoder frame pairing. A packet is two frames; these feed awkward
# combinations and check that nothing but the genuine code is published.
DECODER_REMOTE = 900001
PAIR_TIMEOUT = 0.3  # must exceed PAIR_TIMEOUT_MS in secplus_receiver.cpp


async def test_decoder_ignores_stale_frame(dev: Device) -> None:
    await dev.set_require_repeat(False)  # observe single packets
    start = len(dev.events)
    await dev.inject(DECODER_REMOTE, 1000, "1")  # frame 2 never arrives
    await asyncio.sleep(PAIR_TIMEOUT + 0.2)
    await dev.inject(DECODER_REMOTE, 1001, "12")
    assert await dev.decoded_since(start) == [(DECODER_REMOTE, 1001)]


async def test_decoder_handles_frame_2_first(dev: Device) -> None:
    start = len(dev.events)
    await dev.inject(DECODER_REMOTE, 1010, "212")  # burst starts mid-packet
    assert await dev.decoded_since(start) == [(DECODER_REMOTE, 1010)]


async def test_decoder_rejects_truncated_frame(dev: Device) -> None:
    start = len(dev.events)
    await dev.inject(DECODER_REMOTE, 1020, "12")
    await dev.inject(DECODER_REMOTE, 1021, "1t")  # frame 2 cut short
    assert await dev.decoded_since(start) == [(DECODER_REMOTE, 1020)]
    await dev.inject(DECODER_REMOTE, 1021, "12")
    assert await dev.decoded_since(start) == [(DECODER_REMOTE, 1020), (DECODER_REMOTE, 1021)]


async def test_decoder_pairs_frames_only_within_timeout(dev: Device) -> None:
    start = len(dev.events)
    await dev.inject(DECODER_REMOTE, 1030, "1")
    await asyncio.sleep(PAIR_TIMEOUT + 0.2)
    await dev.inject(DECODER_REMOTE, 1030, "2")
    assert await dev.decoded_since(start) == []
    await dev.inject(DECODER_REMOTE, 1031, "1")  # split over two bursts, close together
    await dev.inject(DECODER_REMOTE, 1031, "2")
    assert await dev.decoded_since(start) == [(DECODER_REMOTE, 1031)]


async def test_decoder_decodes_frame_at_end_of_burst(dev: Device) -> None:
    start = len(dev.events)
    await dev.inject(DECODER_REMOTE, 1040, "12x")  # no gap after the last frame
    assert await dev.decoded_since(start) == [(DECODER_REMOTE, 1040)]
    await dev.set_require_repeat(True)


async def test_require_repeat(dev: Device) -> None:
    start = len(dev.events)
    await dev.inject(DECODER_REMOTE, 1050, "12")  # one packet is not enough
    await dev.inject(DECODER_REMOTE, 1051, "12")  # a different code does not confirm it
    assert await dev.decoded_since(start) == []
    assert not [e for e in dev.events[start:] if e.service.startswith("esphome.test_")]
    await dev.inject(DECODER_REMOTE, 1051, "12")  # same code again, in a later burst
    assert await dev.decoded_since(start) == [(DECODER_REMOTE, 1051)]
    await dev.inject(DECODER_REMOTE, 1052, "12")
    await asyncio.sleep(REPEAT_WINDOW + 0.3)
    await dev.inject(DECODER_REMOTE, 1052, "12")  # too late to count as a repeat
    assert await dev.decoded_since(start) == [(DECODER_REMOTE, 1051)]


async def test_single_packet_does_not_advance_counter(dev: Device) -> None:
    # REMOTE_A is enrolled and 121 is its next valid code. A lone packet (which
    # could be a corrupted decode) must not authenticate or move the counter:
    # test_stop_learn_closes_learn_mode later authenticates 121 itself.
    start = len(dev.events)
    await dev.inject(REMOTE_A, 121, "12")
    assert await dev.decoded_since(start) == []
    assert not [e for e in dev.events[start:] if e.service.startswith("esphome.test_")]
    await asyncio.sleep(REPEAT_WINDOW + 0.3)  # so the later press starts afresh


async def test_learn_needs_two_presses(dev: Device) -> None:
    dev.push_button("Learn Remote")
    await asyncio.sleep(0.2)
    expect_auth(await dev.press(REMOTE_A, 101), None)
    await asyncio.sleep(0.2)
    assert dev.slot(1) == "empty"  # one press is not enough
    expect_auth(await dev.press(REMOTE_A, 102), None)  # enrolling never authenticates
    await dev.wait_slot(1, str(REMOTE_A))
    learned = await dev.wait_for(dev.learned_events, "learned event")
    assert learned[-1] == {"fixed_data": str(REMOTE_A), "slot": "1"}, learned
    # on_remote_learned fired; enrolling presses reach on_remote_received only.
    await dev.wait_for(
        lambda: [e.data for e in dev.events if e.service == "esphome.test_learned"],
        "on_remote_learned",
    )
    assert [e.data for e in dev.events if e.service == "esphome.test_learned"] == [{"slot": "1"}]
    await dev.wait_triggers(102, [("received", {"rolling": "102"})])


async def test_authenticates_and_rejects_replay(dev: Device) -> None:
    expect_auth(await dev.press(REMOTE_A, 103), 1)
    expect_auth(await dev.press(REMOTE_B, 500), None)  # unknown remote in between
    expect_auth(await dev.press(REMOTE_A, 103), None)  # replay of an accepted code
    expect_auth(await dev.press(REMOTE_A, 101), None)  # replay of an older code

    # Triggers: an accepted press fires on_remote_received then on_authenticated;
    # a replay fires on_remote_received then on_rejected, never on_authenticated;
    # an unknown remote fires on_remote_received only.
    await dev.wait_triggers(
        103,
        [
            ("received", {"rolling": "103"}),
            ("authenticated", {"rolling": "103", "slot": "1"}),
            ("received", {"rolling": "103"}),
            ("rejected", {"rolling": "103", "slot": "1"}),
        ],
    )
    await dev.wait_triggers(500, [("received", {"rolling": "500"})])


async def test_counter_window(dev: Device) -> None:
    expect_auth(await dev.press(REMOTE_A, 103 + WINDOW), 1)  # edge of the window
    last = 103 + WINDOW
    expect_auth(await dev.press(REMOTE_A, last + WINDOW + 1), None)  # just beyond it
    expect_auth(await dev.press(REMOTE_A, last + 1), 1)  # counter was not disturbed


async def test_static_remote(dev: Device) -> None:
    expect_auth(await dev.press(STATIC_REMOTE, 7), 3)  # first use is trusted
    expect_auth(await dev.press(REMOTE_B, 501), None)
    expect_auth(await dev.press(STATIC_REMOTE, 7), None)
    expect_auth(await dev.press(STATIC_REMOTE, 8), 3)


async def test_static_remote_recovers_from_lockout(dev: Device) -> None:
    # Forged codes push the stored counter ahead of the real remote (at 8).
    expect_auth(await dev.press(STATIC_REMOTE, 8 + WINDOW), 3)
    expect_auth(await dev.press(STATIC_REMOTE, 8 + 2 * WINDOW), 3)
    expect_auth(await dev.press(STATIC_REMOTE, 9), None)  # real remote is locked out
    await learn(dev, STATIC_REMOTE, 10)  # learn mode will not wind a counter back
    expect_auth(await dev.press(STATIC_REMOTE, 12), None)
    dev.push_button("Stop Learning")  # the refused resync left learn mode open
    await asyncio.sleep(0.2)

    await dev.forget(3)  # on a static remote: keeps it enrolled, drops its counter
    # The forget call is not acknowledged. Wait for a later press to come back
    # so the kill cannot land in the middle of the host's preferences write.
    expect_auth(await dev.press(REMOTE_B, 502), None)
    await dev.restart(kill=True)  # the reset itself must be on disk
    # Trusted again at any counter, here far outside the window from any old value.
    expect_auth(await dev.press(STATIC_REMOTE, 5000), 3)
    expect_auth(await dev.press(STATIC_REMOTE, 5000 + WINDOW + 1), None)  # window applies again
    expect_auth(await dev.press(STATIC_REMOTE, 5001), 3)


async def test_keypad_frame(dev: Device) -> None:
    # A keypad entry is a 64-bit frame; it must decode and carry the PIN.
    event = await dev.press(REMOTE_B, 550, pin=1234)
    assert event.get("pincode") == "1234", event
    expect_auth(event, None)


async def test_learn_mode_times_out(dev: Device) -> None:
    dev.push_button("Learn Remote")
    await asyncio.sleep(0.2)
    expect_auth(await dev.press(REMOTE_B, 600), None)
    await asyncio.sleep(LEARN_TIMEOUT + 0.5)
    expect_auth(await dev.press(REMOTE_B, 601), None)
    await asyncio.sleep(0.2)
    assert dev.slot(2) == "empty"


async def test_stop_learn_closes_learn_mode(dev: Device) -> None:
    dev.push_button("Learn Remote")
    await asyncio.sleep(0.2)
    expect_auth(await dev.press(REMOTE_B, 610), None)
    dev.push_button("Stop Learning")
    await asyncio.sleep(0.2)
    expect_auth(await dev.press(REMOTE_B, 611), None)  # would have enrolled it
    await asyncio.sleep(0.2)
    assert dev.slot(2) == "empty"
    expect_auth(await dev.press(REMOTE_A, 121), 1)  # authentication is back on at once


async def test_learn_mode_suppresses_authentication(dev: Device) -> None:
    dev.push_button("Learn Remote")
    await asyncio.sleep(0.2)
    expect_auth(await dev.press(REMOTE_A, 122), None)  # enrolled, but learn mode is open
    await asyncio.sleep(LEARN_TIMEOUT + 0.5)
    expect_auth(await dev.press(REMOTE_A, 123), 1)


async def test_resync_in_learn_mode(dev: Device) -> None:
    expect_auth(await dev.press(REMOTE_A, 5000), None)  # far beyond the window
    await learn(dev, REMOTE_A, 5001)
    await asyncio.sleep(0.2)
    assert dev.slot(1) == str(REMOTE_A) and dev.slot(2) == "empty"  # no duplicate slot
    expect_auth(await dev.press(REMOTE_A, 5003), 1)


async def test_survives_restart(dev: Device) -> None:
    # SIGKILL, not a clean exit: counters must already be on disk after each press.
    await dev.restart(kill=True)
    await dev.wait_slot(1, str(REMOTE_A))
    expect_auth(await dev.press(REMOTE_A, 5003), None)  # counter persisted: still a replay
    expect_auth(await dev.press(REMOTE_A, 5004), 1)
    expect_auth(await dev.press(STATIC_REMOTE, 5001), None)  # static counter persisted too
    expect_auth(await dev.press(STATIC_REMOTE, 5002), 3)


async def test_forget_slot(dev: Device) -> None:
    await learn(dev, REMOTE_B, 700)
    await dev.wait_slot(2, str(REMOTE_B))
    for bad_slot in (0, 4, 257, -255):  # must not wrap onto slot 1
        await dev.forget(bad_slot)
    await asyncio.sleep(0.2)
    assert dev.slot(1) == str(REMOTE_A)
    await dev.forget(1)
    await dev.wait_slot(1, "empty")
    expect_auth(await dev.press(REMOTE_A, 5005), None)  # revoked immediately
    expect_auth(await dev.press(REMOTE_B, 702), 2)  # other slot untouched
    await dev.restart()
    await dev.wait_slot(1, "empty")  # still forgotten after a reboot
    await dev.wait_slot(2, str(REMOTE_B))


async def test_slots_full_then_forget_all(dev: Device) -> None:
    await learn(dev, REMOTE_A, 6000)  # reuses the freed slot 1
    await dev.wait_slot(1, str(REMOTE_A))
    await learn(dev, 55555, 10)  # no free slot left
    await asyncio.sleep(0.2)
    expect_auth(await dev.press(55555, 12), None)

    dev.push_button("Forget All Remotes")
    await dev.wait_slot(1, "empty")
    await dev.wait_slot(2, "empty")
    expect_auth(await dev.press(REMOTE_A, 6002), None)
    expect_auth(await dev.press(STATIC_REMOTE, 5003), 3)  # static remote remains


def fnv1(text: str) -> int:
    """ESPHome's fnv1_hash, used for preference keys."""
    value = 2166136261
    for byte in text.encode():
        value = ((value * 16777619) & 0xFFFFFFFF) ^ byte
    return value


def read_prefs(path: Path) -> dict[int, bytes]:
    """Host preferences file: repeated (key u32, length u8, data)."""
    raw, records, pos = path.read_bytes(), {}, 0
    while pos + 5 <= len(raw):
        key, length = struct.unpack_from("<IB", raw, pos)
        records[key] = raw[pos + 5 : pos + 5 + length]
        pos += 5 + length
    return records


def write_prefs(path: Path, records: dict[int, bytes]) -> None:
    path.write_bytes(
        b"".join(struct.pack("<IB", key, len(data)) + data for key, data in records.items())
    )


SLOT_RECORD = "<QIB"  # RemoteSlot: fixed, rolling, flags
OCCUPIED_WITH_COUNTER = 0x03


async def test_learned_copy_of_static_remote_is_merged_at_boot(dev: Device) -> None:
    # As if STATIC_REMOTE had been learned into slot 2 (counter 9000) before it
    # was added to `remotes:`. Its own saved counter is 5003.
    prefs = Path(dev.prefdir) / "secplus-host.prefs"
    slot2_key = (fnv1("secplus_receiver_slot") + 1) & 0xFFFFFFFF
    static_key = fnv1("secplus_receiver_static")
    for byte in struct.pack("<Q", STATIC_REMOTE):
        static_key = ((static_key * 16777619) & 0xFFFFFFFF) ^ byte

    await dev.stop()
    records = read_prefs(prefs)
    assert struct.unpack(SLOT_RECORD, records[static_key])[1] == 5003, "test setup"
    records[slot2_key] = struct.pack(SLOT_RECORD, STATIC_REMOTE, 9000, OCCUPIED_WITH_COUNTER)
    write_prefs(prefs, records)

    await dev.start()
    await dev.wait_slot(2, "empty")  # the learned copy does not take a slot
    await dev.stop(kill=True)  # no press yet: boot alone must have written both slots back
    records = read_prefs(prefs)
    assert struct.unpack(SLOT_RECORD, records[slot2_key]) == (0, 0, 0), records[slot2_key]
    assert struct.unpack(SLOT_RECORD, records[static_key])[1] == 9000, records[static_key]

    await dev.start()
    expect_auth(await dev.press(STATIC_REMOTE, 5004), None)  # replay protection carried over
    expect_auth(await dev.press(STATIC_REMOTE, 9001), 3)


SCENARIOS = [
    test_boot_state,
    test_decode_and_unknown_remote,
    test_decoder_ignores_stale_frame,
    test_decoder_handles_frame_2_first,
    test_decoder_rejects_truncated_frame,
    test_decoder_pairs_frames_only_within_timeout,
    test_decoder_decodes_frame_at_end_of_burst,
    test_require_repeat,
    test_learn_needs_two_presses,
    test_authenticates_and_rejects_replay,
    test_counter_window,
    test_single_packet_does_not_advance_counter,
    test_static_remote,
    test_static_remote_recovers_from_lockout,
    test_keypad_frame,
    test_learn_mode_times_out,
    test_stop_learn_closes_learn_mode,
    test_learn_mode_suppresses_authentication,
    test_resync_in_learn_mode,
    test_survives_restart,
    test_forget_slot,
    test_slots_full_then_forget_all,
    test_learned_copy_of_static_remote_is_merged_at_boot,
]


async def run() -> int:
    failures = 0
    with tempfile.TemporaryDirectory() as prefdir:
        dev = Device(prefdir)
        await dev.start()
        try:
            for scenario in SCENARIOS:
                try:
                    await scenario(dev)
                    print(f"PASS  {scenario.__name__}")
                except Exception as exc:  # noqa: BLE001 - report and stop
                    failures += 1
                    print(f"FAIL  {scenario.__name__}: {exc!r}")
                    print("      later scenarios depend on this one; stopping")
                    break
        finally:
            await dev.stop()
    print(f"{len(SCENARIOS)} scenarios, {failures} failed")
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--no-build", action="store_true", help="reuse the last host build")
    args = parser.parse_args()

    if not args.no_build:
        print("Building host program (first run takes a minute)...")
        build = subprocess.run(
            [sys.executable, "-m", "esphome", "compile", str(CONFIG)],
            capture_output=True,
            text=True,
            check=False,
        )
        if build.returncode != 0:
            print(build.stdout[-4000:])
            print(build.stderr[-4000:])
            print("Host build failed")
            return 1
    if not BINARY.exists():
        print(f"Host program not found at {BINARY}; run without --no-build")
        return 1

    return asyncio.run(run())


if __name__ == "__main__":
    sys.exit(main())
