# Freddie

A shy little autonomous robot.

## Parts

Excluding build choices and stock components: capacitors, resistors, LEDs, boards, wire, plugs/sockets, switches, batteries, fixings etc.

All from The Pi Hut:-

- [Octagon Chassis Frame - Blue Plastic - 16cm x 16cm x 4cm](https://thepihut.com/products/octagon-chassis-frame-blue-plastic-16cm-x-16cm-x-4cm)
- [2 off TT Motor Bi-Metal Gearbox - 1:90 Gear Ratio](https://thepihut.com/products/tt-motor-bi-metal-gearbox-1-90-gear-ratio)
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

```sh
cd freddie
source ~/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```
