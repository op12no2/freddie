# Freddie

A little autonomous robot that chases you. He spins on the spot looking through an 8x8 thermal camera then drives in your direction after detection. He can follow you through small changes in direction but if he loses you, he stops and looks around again etc.

Freddie handles stalls and bumps by reversing out and spinning; looking.

An alternative game is to coax Freddie around a course that you set.

## LED

- Green - spinning, looking for you.
- Blue - chasing you.
- Violet - stall or bump.

## Parts

Excluding build choices and stock components: capacitors, resistors, LEDs, boards, wire, plugs/sockets, switches, batteries, fixings etc.

All from The Pi Hut:-

- [Octagon Chassis Frame - Blue Plastic - 16cm x 16cm x 4cm](https://thepihut.com/products/octagon-chassis-frame-blue-plastic-16cm-x-16cm-x-4cm)
- [2 off TT Motor Bi-Metal Gearbox - 1:90](https://thepihut.com/products/tt-motor-bi-metal-gearbox-1-90-gear-ratio)
- [or 2 off TT Motor Plastic Gearbox - 1:48](https://thepihut.com/products/dc-gearbox-motor-tt-motor-200rpm-3-to-6vdc)
- [2 off Orange and Clear TT Motor Wheel for TT DC Gearbox Motor](https://thepihut.com/products/orange-and-clear-tt-motor-wheel-for-tt-dc-gearbox-motor)
- [20mm Height Metal Caster Bearing Wheel](https://thepihut.com/products/20mm-height-metal-caster-bearing-wheel)
- [ESP32-S3-DevKitC-1 Development Board](https://thepihut.com/products/esp32-s3-devkitc-1-development-board)
- [Pololu DRV8833 Dual Motor Driver Carrier](https://thepihut.com/products/pololu-drv8833-dual-motor-driver-carrier)
- [Adafruit AMG8833 IR Thermal Camera Breakout (STEMMA QT)](https://thepihut.com/products/adafruit-amg8833-ir-thermal-camera-breakout)
- [Adafruit INA219 High Side DC Current Sensor Breakout (STEMMA QT)](https://thepihut.com/products/adafruit-ina219-high-side-dc-current-sensor-breakout-26v-3-2a-max)
- [Adafruit LSM6DSOX 6 DoF Accelerometer and Gyroscope (STEMMA QT)](https://thepihut.com/products/adafruit-lsm6dsox-6-dof-accelerometer-and-gyroscope)

## Docs

- [Schematic](./freddie.pdf)

## Build

One repo, several Freddies. `FREDDIE_BUILD` picks which robot the firmware
is for: it selects a block near the top of `main/freddie_main.c` holding
everything that differs between them (motor wiring and polarity, motor
deadband and left/right trim, whether an INA219 is fitted, and the GPIO of
the shelf LED, or -1 for none). Nothing else in the firmware knows which
robot it is. The console prints the build number at boot.

| Build | Motors | INA219 | Shelf LED | Notes |
|-------|--------|--------|-----------|-------|
| 1 (default) | 1:90 bi-metal TT | yes | GPIO 11 | the original; wiring and calibration measured |
| 2 | 1:48 plastic TT (200 rpm) | no | none | wiring and calibration copied from build 1, not yet measured |

Give each robot its own build directory with `-B`. The `-D` flag sticks in
that directory's CMake cache, so it only has to be passed the first time.

Build 1 (the default, plain `build/`):

```sh
cd freddie
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Build 2:

```sh
cd freddie
source ~/esp/esp-idf/export.sh
idf.py -B build-2 -DFREDDIE_BUILD=2 build
idf.py -B build-2 -p /dev/ttyUSB0 flash monitor
```

Without an INA219 a build has no stall detection (bumps still count as a
tag) and cannot tell it is on USB power alone, so don't run that robot's
motors on USB with the pack off.

### Adding a Freddie

Add an `#elif FREDDIE_BUILD == N` block next to the others and build with
`-B build-N -DFREDDIE_BUILD=N`. Then, in this order:

1. Wheel direction. If a wheel runs backwards, swap that wheel's IN1 and
   IN2 GPIOs; if left and right are swapped, swap the L and R pairs.
2. Deadband (`MOTOR_MIN_PCT`): the lowest duty that reliably moves him
   from rest.
3. Trim (`DRIVE_TRIM_PCT`): added to the left duty until he drives straight.

A motor much faster or slower than build 1's also shifts the feel of the
behaviour knobs (`SPIN_PCT`, `GO_PCT`, `STEER_K`, `BACK_PCT`, `TURN_PCT`,
`STALL_MIN_MA`); those are shared between builds and worth a look.
