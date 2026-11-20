# secplus_receiver — ESPHome External Component

An ESPHome external component that decodes **Security+ 2.0** rolling-code
transmissions from Chamberlain, LiftMaster, Craftsman and other knock-off garage door remotes
using a **CC1101** sub-GHz transceiver and ESPHome's `remote_receiver`.

Purpose: Remote transmitters are widely available, very inexpensive and can have
a good range. By knowing a remote's fixed identifier (as well as its rolling code) lights, gates, etc.
can be controlled by Home Assistant automations using these remotes.

Decoded results are exposed as Home Assistant sensor entities 
and/or via an event sent to Home Assistant.  

Zero or more text entities can be set up, and their names can be changed via a `substitutions`
secton in your YAML.  If using the `packages` approach (as shown below) only "Fixed Data"
and "Rolling Code" entities are set up.

| Entity Name | Value |
|---|---|
| **Fixed Data** | 40-bit fixed remote identifier |
| **Rolling Code** | Current rolling code counter |
| **Remote ID** | Attempted parse of the remote's specific ID |
| **Button ID** | Attempted parse of the individual button pressed |

The "Fixed Data" should be unique to the remote + specific button pushed.
The fixed data may be broken into two parts which typically holds the remote's ID
and the specific button pressed, but decoding those may be inconsistent across different
remote manufactures. That is why those entites are disabled by default.

This can also send an event to Home Assistant when a button press is detected. This is disabled by default.
The event is "esphome.secplus_received" and includes in the event data:
"fixed_data", "rolling_code", "remote_id", and "button". This is useful if you want to trigger on a button
press AND have all the data at the same time. (Triggering on a sensor may mean the other sensors are not in sync at
that instant.) This is the best way to see if a button has been pressed since just triggering on the fixed data
will only trigger a state change if a _different_ button/remote was pushed first.

To enable events add this to your YAML:

```
secplus_receiver:
  fire_homeassistant_event: true
```

See [Notes](#notes) below on this being beta software and how AI was used.

---

## How it works

Security+ 2.0 transmits on 310 / 315 / 390 MHz using **OOK modulation** at
4000 baud with **Manchester encoding**. The CC1101 demodulates the OOK signal
on its `GDO0` pin; ESPHome's `remote_receiver` captures the raw pulse timing
and passes it to the decoder. The decoder assembles two-frame packet pairs and
calls [`decode_v2()`](https://github.com/argilo/secplus) from the
[argilo/secplus](https://github.com/argilo/secplus) C library to extract the
remote ID and rolling code.

```
CC1101 GDO0 ──► remote_receiver (on_raw) ──► id(receiver).process(x)
                                                      │
                                           SecplusReceiverComponent
                                           (secplus_receiver.h)
                                                      │
                                         secplus_parser.c + secplus.c
                                                      │
                                    ┌─────────────────┴────────────────┐
                                    │                                  │
                             remote_id_sensor                 rolling_code_sensor
                             (text_sensor)                    (text_sensor)
```

---

## Hardware

| Part | Notes |
|---|---|
| ESP32 (any variant) | Tested on ESP32-S3 |
| CC1101 module | 310 / 315 / 390 MHz |

### Wiring

| CC1101 pin | ESP32 pin | Notes |
|---|---|---|
| VCC | 3.3 V | |
| GND | GND | |
| MOSI | GPIO9 | |
| MISO | GPIO8 | |
| SCK | GPIO7 | |
| CS / CSN | GPIO4 | SPI chip select |
| GDO0 | GPIO2 | Demodulated OOK output → `remote_receiver` |

Adjust GPIO numbers to match your board.

---

## Installation

See [`secplus_receiver_example.yaml`](secplus_receiver_example.yaml) for a
complete working configuration.


### 1 - Add the remote package to your YAML configuration:

```yaml
packages:
  secplus: github://greenbus4/secplus_receiver/secplus_receiver.yaml@main
```

### 2 — Optionally add `substitutions` section in your YAML

Adjust depending on your ESP device's wiring and the CC1101 you use.

```yaml
substitutions:
  remote_fixed_data_name:   "Fixed Data"    # Name of fixed data returned by remote as a t
  remote_rolling_code_name: "Rolling Code"  # Name of the rolling code text sensor
  remote_id_name:           "Remote ID"     # Name of the remote ID text sensor
  remote_button_name:       "Button ID"     # Name of the button text sensor

  # Pin connections to CC1101
  spi_clk_pin: GPIO07
  spi_miso_pin: GPIO08
  spi_mosi_pin: GPIO09

  # CC1101 Receiver
  cc1101_cs_pin: GPIO04
  cc1101_frequency: 390.00MHz  # 310MHz, 315Mhz, 390Mhz

  receiver_pin: GPIO02

```
### 3 - Optionally add a `secplus_receiver:` section to enable or disable entites and enable HA events:

```
secplus_receiver:
  remote_id_sensor:
    name: ${remote_id_name}

  button_sensor:
    name: ${remote_button_name}

  fire_homeassistant_event: true
```


---

## Authentication (optional)

By default every decoded remote is reported, and a recorded transmission can
simply be replayed. Enabling `authentication` adds an allowlist of enrolled
remotes and rolling-counter replay protection, the same kind of check a real
opener performs.

> **Security limit:** Security+ 2.0 has no secret key. Anyone who captures a
> single transmission can compute the next valid code, and no receiver can
> prevent that. Treat this as replay protection, not strong authentication.
>
> In practice that means:
>
> * The fixed code and rolling counter are all an attacker needs, and this
>   component publishes both: in the sensors, the `esphome.secplus_received`
>   event, the "Remote Slot N" sensors and the device log. Anyone who can read
>   your Home Assistant or the device log can forge a remote. Leave out the
>   sensors and events you do not use.
> * Anyone in radio range while learn mode is open can enrol a forged remote.
>   Keep `learn_timeout` short and check which slot was filled.
> * Someone forging codes for an enrolled remote can push its stored counter
>   ahead, so the real remote stops working until it is re-enrolled (see
>   below).
> * A remote listed under `remotes:` has no stored counter until its first
>   press, so the first code heard for it is accepted whatever its counter,
>   including an old recording. This applies on a fresh install, after a flash
>   erase, and after `secplus_receiver.forget` on its slot. Learned remotes do
>   not have this gap.
> * A keypad is authenticated by its fixed code alone. The PIN typed on it is
>   not checked; compare `pincode` from the event yourself if you rely on it.

```yaml
secplus_receiver:
  id: secplus_receiver_id
  authentication:
    max_remotes: 8       # learnable slots (0-32), default 8
    window: 1024         # accepted distance ahead of the stored counter, default 1024
    learn_timeout: 30s   # how long learn mode stays open, default 30s, minimum 1s
    slot_sensors: true   # create a "Remote Slot N" text sensor per slot, default true
    remotes:             # optional: fixed codes that are always enrolled
      - 70678577664
  on_authenticated:
    then:
      - logger.log:
          format: "Authenticated remote in slot %u, button %u"
          args: [slot, button]
```

To accept every default, write `authentication: {}`.

### How a press is checked

* The fixed code must be enrolled, either learned into a slot or listed under
  `remotes:`.
* The rolling code must be 1 to `window` counts ahead of the last accepted
  code for that remote. Anything else, including a replay, is rejected.
* The accepted counter is written to flash immediately, so it survives a
  power cut.

Two things this does not do:

* Replaying the most recent transmission straight away is dropped as a
  duplicate before authentication runs. It never authenticates, but it does
  not fire `on_rejected` either.
* A remote that gets more than `window` presses ahead is not accepted again
  on its own. Open learn mode and press it twice to resync.

Each button on a remote has its own fixed code and is enrolled separately.

### Learning and forgetting remotes

| Action | Effect |
|---|---|
| `secplus_receiver.learn` | Opens learn mode for `learn_timeout`. Press the new remote's button **twice**; it is stored in the first free slot and learn mode closes. |
| `secplus_receiver.stop_learn` | Closes learn mode early. |
| `secplus_receiver.forget` | Clears one slot (`slot:` is 1-based and templatable). On a `remotes:` entry it keeps the remote enrolled and only discards its stored counter. |
| `secplus_receiver.forget_all` | Clears every learned slot. Does not touch `remotes:` entries or their counters. |

While learn mode is open no remote authenticates, so enrolling never triggers
your automations. Pressing an already-enrolled remote twice in learn mode
moves its stored counter forward, which recovers a remote that was pressed
more than `window` times out of range.

Learn mode never moves a counter backwards. If a remote's stored counter has
ended up *ahead* of the real remote (see the security limit above), forget
its slot and learn it again. For a remote listed under `remotes:`, run
`secplus_receiver.forget` on its slot: the remote stays enrolled and its next
press is trusted, the same as on first use.

Expose the actions as Home Assistant entities, or wire `learn` to a GPIO button:

```yaml
button:
  - platform: template
    name: "Learn Remote"
    on_press:
      - secplus_receiver.learn: secplus_receiver_id
  - platform: template
    name: "Forget All Remotes"
    on_press:
      - secplus_receiver.forget_all: secplus_receiver_id

api:
  actions:
    - action: forget_remote      # esphome.<device>_forget_remote in Home Assistant
      variables:
        slot: int
      then:
        - secplus_receiver.forget:
            id: secplus_receiver_id
            slot: !lambda "return slot;"
```

Each "Remote Slot N" sensor shows the fixed code stored in that slot, or
`empty`. Slots hold codes, not names; map slots to people in your automations.
Remotes listed under `remotes:` are numbered after the learnable slots, have
no sensor, and can only be removed by editing the YAML. Their slot numbers
therefore change if you change `max_remotes`.

If you add a learned remote's code to `remotes:`, its learned slot is freed
on the next boot and its counter is carried over.

On ESP32, learned remotes and counters survive firmware updates and
configuration changes; a full flash erase removes them. Authentication has
only been tested on ESP32. On ESP8266 and RP2040, ESPHome stores saved state
in creation order rather than by name, so changing `max_remotes`, the
`remotes:` list, or other components that save state can discard the stored
remotes and counters.

Lowering `max_remotes` hides the remotes in the removed slots without erasing
them. Raising it again brings them back with the counters they had, so forget
those slots first if the remotes should stay removed.

Every accepted press writes one small record to flash. At normal use that is
negligible wear. Someone who has captured a press can forge codes that are
accepted, and each one causes a write. On ESP32 the writes are spread over a
wear-levelled partition, so this is not a practical concern. On ESP8266 every
write erases the same flash sector, which is typically rated for about
100,000 erase cycles, so a sustained stream of forged presses could wear it
out within hours.

### Triggers and events

| Trigger | Variables |
|---|---|
| `on_authenticated` | `fixed_data`, `remote_id`, `button`, `rolling`, `slot` |
| `on_rejected` | `fixed_data`, `rolling`, `slot` (an enrolled remote sent a code outside the window, for example a replay of an older code) |
| `on_remote_learned` | `fixed_data`, `slot` |

`on_remote_received`, the sensors and the `esphome.secplus_received` event
still fire for every remote. With authentication enabled the event also
carries `authenticated` (`true` / `false`) and, when authenticated, `slot`.
With `fire_homeassistant_event: true`, learning a remote fires
`esphome.secplus_remote_learned` with `fixed_data` and `slot`.

### Tests

Neither test suite needs an ESP32 or a radio.

**Unit tests** for the slot and counter logic:

```
c++ -std=c++17 -Wall -Wextra -o /tmp/test_remote_store tests/test_remote_store.cpp && /tmp/test_remote_store
```

**End-to-end tests** build the component for ESPHome's native `host` platform
and drive it over the real native API, the way Home Assistant would. Remote
presses are injected as genuine Security+ 2.0 pulse trains, so decoding,
authentication, learn mode, forgetting, the triggers, keypad frames, and
persistence across a hard kill are all covered. Requires `esphome` and a C++
compiler on Linux or macOS:

```
python tests/host/run_host_tests.py
```

These tests do not cover the CC1101, `remote_receiver`, or real-world signal
timing and noise.

---

## CC1101 frequency

Set the `cc1101:` block's `frequency:` to match your remotes:

| Frequency | Common use |
|---|---|
| `390.00MHz` | LiftMaster / Chamberlain 891LM, 893LM, 895LM (most common in North America) |
| `315.00MHz` | Older LiftMaster and Craftsman remotes |
| `310.00MHz` | Some regional and older variants |

Security+ 2.0 can use all three frequencies.

### Troubleshooting: presses are received but never decode

If the log shows `Decoder received N pulses to process...` with a few hundred
pulses on each press, but no `fixed=...` line ever follows, the remote may be
overloading the CC1101. With the default gain settings and a strong signal,
the radio chops each 250 / 500 µs pulse into fragments of roughly 100 µs, so
the decoder never finds a preamble. Adding `dump: raw` to `remote_receiver:`
shows the pulse widths.

Lowering the gain fixes it. Add this to your own YAML; it merges with the
`cc1101:` block from the package:

```yaml
cc1101:
  magn_target: 33dB
  rx_attenuation: 18dB
```

These values were tested with one setup only: a XIAO ESP32-C6, a CC1101 module
and a 390 MHz remote. There they decoded every press from a few inches away to
the far end of a house through two walls, where the defaults decoded none.
Attenuation costs range, so if distant presses start dropping out, lower
`rx_attenuation` (for example to `6dB` or `12dB`) and test again.


---

## `secplus.c` / `secplus.h` — fetched automatically

At build time, `__init__.py` downloads **`secplus.c`** and **`secplus.h`**
from [argilo/secplus](https://github.com/argilo/secplus) (`master` branch)
into the component directory. An internet connection is required on the first
build; subsequent builds use the cached copy.

---

## Repository layout

```
secplus_receiver/
├── README.md
├── secplus_receiver_example.yaml
└── esphome/
    └── components/
        └── secplus_receiver/
            ├── __init__.py           ← config schema + build-time downloader
            ├── secplus_receiver.h    ← SecplusReceiverComponent C++ class
            ├── remote_store.h        ← enrolled-remote table (authentication)
            ├── secplus_parser.c      ← Manchester / Security+ 2.0 frame decoder
            ├── secplus_parser.h
            └── secplus_shim.h        ← ManchesterDecoder C++ wrapper
            # secplus.c & secplus.h are downloaded here at build time
```
---
## Notes:

### Beta code

This is currently still being tested and subject to change.
One change might be include the optional "data" provided by the remote
that can include, for example, pin codes.

### AI use

A fair amount of AI was used here.

This code started out as a Python implementation modeled after 
[secplus_rx_secplus_v2_decode.py](https://github.com/argilo/secplus/blob/master/secplus_rx_secplus_v2_decode.py)
from the [secplus](https://github.com/argilo/secplus) repository.

I rewrote the code in C, but still using byte arrays to represent binary data.
Claude AI (Sonnet 4.6) was used to convert it to using bit data to save space
for using on ESP32. Claude was also used to build the C++ glue to connect ESPHome
to the C code due to my lack of C++ knowledge.
AI (Claude and Gemini) were used to speed up creating this external_component.

As a result, much of this README looks AI generated, because it is.

---

## Credits

- Security+ 2.0 decoding: [argilo/secplus](https://github.com/argilo/secplus)
  by Clayton Smith (GPL-3.0)

## License

`secplus_parser.c`, `secplus_parser.h`, `secplus_receiver.h`, and
`secplus_shim.h` are provided under the MIT License. `secplus.c` and
`secplus.h` (downloaded at build time) are © Clayton Smith and licensed
under the **GPL-3.0**.
