# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for Freddie, an autonomous ESP32-S3 robot (see README.md for the
parts list). He watches the room through an 8x8 thermal camera and plays
tag: spin on the spot, slowing as warmth crosses his view; when something
warm sits near the middle of the frame, chase it flat out, steering on its
centroid; stand still when it's right at his wheels; lose it and spin
again. The firmware is fully autonomous and deliberately minimal: no
radio, no logging — the LEDs are his whole interface. There is a small
serial console for the bench (`idf.py monitor`, `?` for help): `p` prints
the thermal frame with target pixels starred, `s` streams it, `x` freezes
the motors while sensing continues. It's for tuning thresholds, not part
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
  on one I2C bus (`i2c_init`, then per-device `*_init`). The camera drives
  the behaviour; the INA219 backs `pack_live()`, which refuses to drive
  the motors on USB power alone; the IMU is initialised for the boot check
  only and otherwise unused for now.
- DRV8833 dual motor driver via LEDC PWM (`motors_init`, `motor_set`,
  `drive`). Note the wiring is physically crossed and inverted versus the
  DRV8833's own pin names — this is corrected once in the `MOTOR_*_GPIO`
  macros near the top of the file, so `drive(left, right)` downstream is
  sane; don't "fix" the apparent crossing there.
- Onboard WS2812 RGB status LED via RMT (`rgb_init`/`rgb_set`). Colour
  meanings are the `RGB_*` macros: red = booting/failed check, green =
  scanning, blue = following.
- A discrete GPIO LED (`RUN_LED_GPIO`) is lit whenever the checks passed
  and he's running.

**Concurrency model**: one FreeRTOS task, `tick_task`, at `TICK_HZ`
(10 Hz). Each tick it reads the thermal frame and steps a two-state
machine (`state_t`: `SCAN`, `FOLLOW`). `app_main()` initialises the
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
  right. Target at `FULL_PX` or more = stand still until it backs off. No
  target = stand still (never charge blind at full duty), and after
  `LOST_TICKS` of that, `SCAN`.

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
