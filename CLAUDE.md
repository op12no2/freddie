# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for Freddie, an autonomous ESP32-S3 robot (see README.md for the full
behavioral description). He watches a room through an 8x8 thermal camera,
notices changes in warmth, and reacts with LED colour, small motor
"performances", and — if something warm lingers and moves like a person — a
cautious hopping approach. He has a sleep/wake rhythm and can be picked up
and set down as a gestural interface (double lift-down toggles the watcher;
holding him upside down for a second arms a "sprint"). The firmware is fully
autonomous and deliberately minimal: there is no console, no radio, no
logging — the LEDs and the gestures are his whole interface.

The entire firmware is one file: `main/freddie_main.c` (~1300 lines). There
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
  on one I2C bus (`i2c_init`, then per-device `*_init`/`*_read`).
- DRV8833 dual motor driver via LEDC PWM (`motors_init`, `motor_set`,
  `drive`). Note the wiring is physically crossed and inverted versus the
  DRV8833's own pin names — this is corrected once in the `MOTOR_*_GPIO`
  macros near the top of the file, so `drive(left, right)` downstream is
  sane; don't "fix" the apparent crossing there.
- Onboard WS2812 RGB status LED via RMT (`rgb_init`/`rgb_set`) — the robot's
  only expressive output besides motion. Colour meanings are enumerated as
  `RGB_*` macros (red = booting/failed check, green = idle awake/asleep
  graded by brightness, blue = performing, violet = held, amber = noticing).
- A discrete GPIO LED (`WAKE_LED_GPIO`) mirrors awake/asleep for
  sunlight-readability where the dim RGB ember isn't visible.

**Concurrency model**: one FreeRTOS task does everything — `tick_task`,
running at `TICK_HZ` (10 Hz). Each tick it reads all sensors, runs
hold/gesture detection (`held_check`), acts on any completed gesture, and
advances the autonomous watch state machine (`watch_step`). `app_main()`
initializes the peripherals, starts the task, runs the switch-on
performance, and returns.

**Two state machines drive behavior, both stepped from `tick_task`:**

1. **Hold/gesture detection** (`held_check`, static state near the
   `HELD_*` macros): distinguishes "picked up" from normal driving-induced
   rotation using gyro magnitude when idle vs. Z-axis tilt when driving.
   Tracks a flip (held upside-down) gesture that arms a sprint performance
   on set-down, and a double lift-down gesture (`gest_toggle`) that toggles
   the watcher on/off. A hold that spikes but doesn't linger long enough
   registers as `knock_felt` — a poke, not a pickup — which the watch state
   machine turns into a startle.

2. **Autonomous watcher** (`watch_step`, `watch_state_t`: `WATCH_OFF`,
   `REST`, `LOOK`, `ORIENT`, `DWELL`, `NERVE`, `CTURN`, `HOP`, `OBS`):
   periodically wakes from `REST` to spin and scan the thermal frame against
   a slowly-adapting background (`watch_bg`), settles facing the warmest/
   most-changed direction, and — if a heat blob persists and moves like a
   person rather than drifting like ambient warmth — works through the
   `coax_*` functions (blob centroid tracking, hop odds, nerve) to hop
   closer in short bursts, backing off if the target moves away. A separate
   slow cycle (`watch_cycle_at`, `watch_toggle`) drives sleep/wake spans
   whose length is modulated by "tiredness", itself derived from an EMA of
   resting pack voltage (`watch_vrest`/`watch_mood`/`watch_tired`) — a
   flatter battery means shorter looks and shorter awake spans, closer to
   the real reason batteries call it a night. Timings throughout this
   machine are drawn from a log-normal distribution
   (`watch_lognormal_s`/`watch_frand`) rather than fixed intervals, so the
   rhythm doesn't feel mechanical.

**Performances** (`perform_alive`, `perform_hello`, `perform_awake`,
`perform_found`, `perform_sprint`): short canned LED+motor sequences
triggered by the state machines above. `perform_sprint` is fenced by
`sprint_safe()` checks (handling, tilt, unexpected rotation) that abort
the run rather than fight a robot that's tipped or in hands.

Many thresholds carry comments citing measured logs (`gestures.log`,
`quiet_room_sat.log`, cal runs) from before the firmware was simplified;
the numbers are load-bearing even though the recording machinery that
produced them is gone.

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
