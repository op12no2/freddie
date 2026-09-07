# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for Freddie, an autonomous ESP32-S3 robot (see README.md for the
parts list). He watches the room through an 8x8 thermal camera and plays
tag: spin on the spot, slowing as warmth crosses his view; when something
warm sits near the middle of the frame, chase it flat out, steering on its
centroid, until he loses it or runs into it; either way look around again
(a run-in earns a back-off and a turn away first). He does the tagging;
getting out of his way is the game. There is deliberately no slowing or
stopping short — that made the child the chaser, which is backwards. The firmware is fully autonomous and deliberately minimal: no
radio, no logging — the LEDs are his whole interface. There is a small
serial console for the bench (`idf.py monitor`, `?` for help): `p` prints
the thermal frame with target pixels starred, `s` streams it, `x` freezes
the motors while sensing continues. The status line carries pack volts
and mA and the commanded duties, for setting `STALL_MA`. It's for tuning thresholds, not part
of the behaviour.

This is a deliberate reset. An earlier, much richer firmware (sleep/wake
rhythm, gestures, performances, a cautious hop-and-observe approach) lives
in git history before the reset commit; the idea is to re-evolve from this
base slowly, picking behaviours back out of those commits one at a time,
only once the basic scan-and-approach is right.

The entire firmware is one file: `main/freddie_main.c` (~630 lines). There
is no test suite — this is embedded C for one physical device, and
correctness is checked by flashing it and watching him.

## Build / flash

Requires the ESP-IDF toolchain installed at `~/esp/esp-idf`.

```sh
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash
```

There's no flashing script or CI — build/flash via `idf.py` is the whole
workflow.

## Architecture

**Target**: ESP32-S3 (`sdkconfig.defaults`), 8 MB flash.

**Peripherals**, all initialized in `app_main()`:
- AMG8833 thermal camera, INA219 power/current monitor, LSM6DSOX IMU — all
  on one I2C bus (`i2c_init`, then per-device `*_init`/`*_read`). The
  camera drives the behaviour; the INA219's pack current is the tag
  sensor (a stall reads as a push) and backs `pack_live()`, which refuses
  to drive the motors on USB power alone; the gyro Z axis meters the
  turn-away after a tag.
- DRV8833 dual motor driver via LEDC PWM (`motors_init`, `motor_set`,
  `drive`). Note the wiring is physically crossed and inverted versus the
  DRV8833's own pin names — this is corrected once in the `MOTOR_*_GPIO`
  macros near the top of the file, so `drive(left, right)` downstream is
  sane; don't "fix" the apparent crossing there.
- Onboard WS2812 RGB status LED via RMT (`rgb_init`/`rgb_set`). Colour
  meanings are the `RGB_*` macros: red = booting/failed check, green =
  scanning, blue = following, violet = tagged.
- A discrete GPIO LED (`RUN_LED_GPIO`) is lit whenever the checks passed
  and he's running.

**Concurrency model**: one FreeRTOS task, `tick_task`, at `TICK_HZ`
(10 Hz). Each tick it reads the pack, the IMU and the thermal frame and
steps a four-state machine (`state_t`: `SCAN`, `FOLLOW`, `BACK`, `TURN`). `app_main()` initialises the
peripherals, starts the task if every check passed, then runs the console
loop on UART0 forever. The task holds still for `SETTLE_S` before its
first scan: the AMG8833's first frames after reset are junk and once
locked him onto a wall. The tick task publishes its latest frame (`last_t`)
for the console's `print_frame`; the console's `x` sets `frozen`, which
the tick honours by stopping the motors and skipping the state machine.

**The behaviour**:
- The target, every tick: `blob()` = pixels `BLOB_C` over `ambient_of()`,
  at least `BLOB_MIN_PX` of them, with a centroid column. `ambient_of()`
  is the mean of the coldest half of the frame — stateless, and valid
  until a target covers half the view, at which point he can't tell them
  from the wall, loses them and looks around, which is how a chase ends.
  (The frame mean fails because a target that fills the frame *is* the
  mean; a reference frozen at lock-on, tried earlier, went stale by the
  time he'd crossed the room. Both are in git history.)
- `SCAN`: spin at `SPIN_PCT`, shedding duty in proportion to the frame's
  max-minus-mean contrast (`GAZE_K`, `GAZE_DEAD_C`, floor `SPIN_MIN_PCT`)
  so the gaze lingers on warmth. This slow-on-heat sweep is the one piece
  carried over unchanged from the old firmware; it works well, don't
  fiddle with it. Target centroid within `LOCK_COLS` of boresight
  (`CENTER_COL`) for `LOCK_TICKS` running = `FOLLOW`.
- `FOLLOW`: drive at `GO_PCT` (100), shedding `STEER_K` of the inner
  wheel's duty per column the centroid sits off boresight. Image columns
  run mirrored to the drive sign (field tested); the sign in the code is
  right. No target = stand still (never charge blind), and after
  `LOST_TICKS` of that, `SCAN` — which is also how a chase ends up close:
  past half the frame the coldest-half ambient can't separate the target
  from the scene, so it vanishes. Motors commanded but pack current over
  `STALL_MA` for `STALL_TICKS` = he's pushing on something: a tag, `BACK`.
  `STALL_MA` has to sit high or launches on carpet trip it; a missed tag
  is harmless, he just loses them and scans.
- `BACK`: reverse at `BACK_PCT` for `BACK_MS`, violet, then `TURN`.
- `TURN`: spin at `TURN_PCT` through a random `TURN_MIN_DEG`..`TURN_MAX_DEG`,
  gyro-metered with `TURN_TIMEOUT_S` as the backstop, then `SCAN`.

Thresholds carry comments citing measured logs (`gestures.log`,
`quiet_room_sat.log`, cal runs) from the old firmware; the numbers are
load-bearing even though the recording machinery that produced them is
gone.

## Repo layout

- `main/freddie_main.c` — the entire firmware.
- `main/CMakeLists.txt` / `CMakeLists.txt` — standard ESP-IDF component/
  project registration; there's nothing else to configure here unless a new
  source file or ESP-IDF component dependency is added.
- `sdkconfig` / `sdkconfig.defaults` — target chip and flash config.
  `sdkconfig` is regenerated by `idf.py` from `sdkconfig.defaults`; prefer
  editing `sdkconfig.defaults` for durable changes.
- `freddie.pdf` / `freddie.drawio` — hardware schematic.
- `README.md` — parts list, behavior description, build instructions.
