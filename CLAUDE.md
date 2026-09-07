# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for Freddie, an autonomous ESP32-S3 robot (see README.md for the
parts list). He watches the room through an 8x8 thermal camera and does one
thing: spin one circle on the spot, slowing as warmth crosses his view,
then rest half a minute and spin again; when something warm sits near the
middle of the frame, drive at it, steering on its centroid, until it fills
the frame; sit there while it does; lose it and go back to circling. The
firmware is fully autonomous and deliberately minimal: no radio, no
logging — the LEDs are his whole interface. There is a small serial
console for the bench (`idf.py monitor`, `?` for help): `p` prints the
thermal frame with target pixels starred, `s` streams it, `x` freezes
the motors while sensing continues. It's for tuning thresholds, not part
of the behaviour.

This is a deliberate reset. An earlier, much richer firmware (sleep/wake
rhythm, gestures, performances, a cautious hop-and-observe approach) lives
in git history before the reset commit; the idea is to re-evolve from this
base slowly, picking behaviours back out of those commits one at a time,
only once the basic scan-and-approach is right.

The entire firmware is one file: `main/freddie_main.c` (~540 lines). There
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
  camera drives the behaviour; the gyro Z axis meters the scan's circle;
  the INA219 backs `pack_live()`, which refuses to drive the motors on USB
  power alone.
- DRV8833 dual motor driver via LEDC PWM (`motors_init`, `motor_set`,
  `drive`). Note the wiring is physically crossed and inverted versus the
  DRV8833's own pin names — this is corrected once in the `MOTOR_*_GPIO`
  macros near the top of the file, so `drive(left, right)` downstream is
  sane; don't "fix" the apparent crossing there.
- Onboard WS2812 RGB status LED via RMT (`rgb_init`/`rgb_set`). Colour
  meanings are the `RGB_*` macros: red = booting/failed check, green =
  scanning (dim green = resting between scans), blue = approaching, amber =
  arrived.
- A discrete GPIO LED (`WAKE_LED_GPIO`) is lit whenever the checks passed
  and he's running.

**Concurrency model**: one FreeRTOS task, `tick_task`, at `TICK_HZ`
(10 Hz). Each tick it reads the thermal frame and the IMU and steps a
four-state machine (`state_t`: `SCAN`, `REST`, `APPROACH`, `ARRIVED`).
`app_main()` initialises the peripherals, starts the task if every check
passed, then runs the console loop on UART0 forever. The tick task
publishes its latest frame (`last_t`, `last_mean`) for the console's
`print_frame`; the console's `x` sets `frozen`, which the tick honours by
stopping the motors and skipping the state machine.

**The behaviour**:
- `SCAN`: spin at `SPIN_PCT`, shedding duty in proportion to the frame's
  max-minus-mean contrast (`GAZE_K`, `GAZE_DEAD_C`, floor `SPIN_MIN_PCT`)
  so the gaze lingers on warmth. This slow-on-heat sweep is the one piece
  carried over unchanged from the old firmware; it works well, don't
  fiddle with it. When a blob (`blob()`: pixels `BLOB_C` over the frame
  mean, at least `BLOB_MIN_PX` of them) has its centroid within
  `LOCK_COLS` of boresight (`CENTER_COL`), freeze the non-blob mean as
  `ambient` and enter `APPROACH`. The circle is gyro-metered
  (`SCAN_TURN_DEG`, with `SCAN_TIMEOUT_S` as the backstop); a circle that
  locks nothing ends in `REST`.
- `REST`: motors off for `REST_S`, dim green. Not blind: the same lock
  check runs, so something warm walking up mid-rest gets approached.
- `APPROACH`: drive at `GO_PCT`, steering by `STEER_K` per column the blob
  centroid sits off boresight. The blob is measured against `ambient`, not
  the live frame mean, because a target that fills the frame *is* the
  mean. `ambient` is seeded from the non-blob mean at lock-on and then
  eased toward the current non-blob mean (`ambient_track`, `AMBIENT_ALPHA`)
  while at least half the frame is background (`AMBIENT_MAX_PX`); with the
  target filling the frame it stays frozen. A stale ambient made the local
  scene read as a target after the visitor left, which held him amber. Image columns run mirrored to the drive sign (field
  tested); the sign in the code is right. `arrived()` = the blob's nearest
  row has reached the floor line (`FLOOR_ROW`, row 0: the bottom of the
  frame, which a warm thing on the floor reaches at ~10 cm regardless of
  its size — the one distance cue the sensor offers) or its weight has hit
  `FILL_PX` (backstop for warmth held off the floor). No blob for
  `LOST_TICKS` = back to `SCAN`.
- `ARRIVED`: motors off. `receded()` = clear of the floor line by
  `FLOOR_HYST_ROWS` and under `FILL_PX - FILL_HYST_PX` = follow
  (`APPROACH`); gone for `LOST_TICKS` = `SCAN`.

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
